# الاختبار: التحقق من تحليل رسائل ESP32 وتنسيقها.
# الغرض: كشف الرسائل غير الصالحة واختبار تحويل اتجاه BNO إلى رباعي.

import json
import math
import unittest

from slam_robot_sim import esp32_link


def make_frame(**overrides):
    frame = {
        's1': [1000 + i for i in range(64)],
        's2': [-1] * 64,
        's3': [2000] * 64,
        'bno': {'ok': 1, 'h': 45.0, 'r': 1.0, 'p': 2.0, 'cal': [3, 3, 3, 3]},
    }
    frame.update(overrides)
    return json.dumps(frame)


class ParseLineTest(unittest.TestCase):

    def test_valid_frame(self):
        message = esp32_link.parse_line(make_frame().encode())
        self.assertEqual(message.kind, 'frame')
        self.assertEqual(len(message.tof), 3)
        self.assertEqual(message.tof[0][0], 1000)
        self.assertEqual(message.tof[1], (-1,) * 64)
        self.assertEqual(message.bno.ok, 1)
        self.assertEqual(message.bno.heading, 45.0)
        self.assertEqual(message.bno.cal, (3, 3, 3, 3))

    def test_status_and_error(self):
        status = esp32_link.parse_line('{"status":"ALL_READY"}')
        self.assertEqual((status.kind, status.text), ('status', 'ALL_READY'))
        error = esp32_link.parse_line('{"error":"ESP-NOW init failed"}')
        self.assertEqual((error.kind, error.text), ('error', 'ESP-NOW init failed'))

    def test_line_printed_like_the_c3_sketch(self):
        zones = ','.join(['1500'] * 63 + ['-1'])
        line = ('{"s1":[%s],"s2":[%s],"s3":[%s],'
                '"bno":{"ok":1,"h":12.3,"r":-1.5,"p":0.0,"cal":[3,2,1,0]}}\r\n'
                % (zones, zones, zones))
        message = esp32_link.parse_line(line.encode())
        self.assertEqual(message.tof[2][63], -1)
        self.assertEqual(message.bno.roll, -1.5)
        self.assertEqual(message.bno.cal, (3, 2, 1, 0))

    def test_rejects_bad_input(self):
        bad_frames = {
            'not json': 'hello',
            'not an object': '[1, 2, 3]',
            'empty': '   ',
            'missing s2': json.dumps({'s1': [1] * 64, 's3': [1] * 64}),
            'short array': make_frame(s1=[1] * 63),
            'long array': make_frame(s1=[1] * 65),
            'zero distance': make_frame(s1=[0] * 64),
            'negative distance': make_frame(s1=[-2] * 64),
            'fractional distance': make_frame(s1=[1.5] * 64),
            'boolean distance': make_frame(s1=[True] * 64),
            'text distance': make_frame(s1=['5'] * 64),
            'missing bno': json.dumps({'s1': [1] * 64, 's2': [1] * 64, 's3': [1] * 64}),
            'bno key missing': make_frame(bno={'ok': 1, 'h': 0, 'r': 0, 'p': 0}),
            'bno ok 2': make_frame(bno={'ok': 2, 'h': 0, 'r': 0, 'p': 0, 'cal': [0] * 4}),
            'cal too short': make_frame(bno={'ok': 1, 'h': 0, 'r': 0, 'p': 0, 'cal': [3]}),
            'cal out of range': make_frame(
                bno={'ok': 1, 'h': 0, 'r': 0, 'p': 0, 'cal': [4, 0, 0, 0]}),
            'nan heading': make_frame().replace('45.0', 'NaN'),
            'infinite roll': make_frame().replace('"r": 1.0', '"r": Infinity'),
        }
        for name, line in bad_frames.items():
            with self.subTest(name):
                with self.assertRaises(esp32_link.ProtocolError):
                    esp32_link.parse_line(line)

    def test_rejects_invalid_utf8(self):
        with self.assertRaises(esp32_link.ProtocolError):
            esp32_link.parse_line(b'\xff\xfe{')

    def test_rejects_huge_number_without_crashing(self):
        line = make_frame().replace('"h": 45.0', '"h": 1' + '0' * 400)
        with self.assertRaises(esp32_link.ProtocolError):
            esp32_link.parse_line(line)


class EulerToQuaternionTest(unittest.TestCase):

    def test_identity(self):
        self.assertEqual(esp32_link.euler_to_quaternion(0, 0, 0), (0.0, 0.0, 0.0, 1.0))

    def test_yaw_90_degrees(self):
        x, y, z, w = esp32_link.euler_to_quaternion(0, 0, 90)
        self.assertAlmostEqual(z, math.sqrt(0.5))
        self.assertAlmostEqual(w, math.sqrt(0.5))
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_roll_and_pitch_axes(self):
        x, y, _, _ = esp32_link.euler_to_quaternion(90, 0, 0)
        self.assertAlmostEqual(x, math.sqrt(0.5))
        self.assertAlmostEqual(y, 0.0)
        x, y, _, _ = esp32_link.euler_to_quaternion(0, 90, 0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, math.sqrt(0.5))

    def test_unit_length(self):
        x, y, z, w = esp32_link.euler_to_quaternion(12.0, -33.0, 170.0)
        self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0)


class LineBufferTest(unittest.TestCase):

    def test_line_split_across_chunks(self):
        buffer = esp32_link.LineBuffer()
        self.assertEqual(buffer.feed(b'{"a":'), [])
        self.assertEqual(buffer.feed(b'1}\n{"b"'), [b'{"a":1}'])
        self.assertEqual(buffer.feed(b':2}\n'), [b'{"b":2}'])

    def test_crlf_and_empty_lines(self):
        buffer = esp32_link.LineBuffer()
        self.assertEqual(buffer.feed(b'one\r\n\r\n\ntwo\n'), [b'one', b'two'])

    def test_oversized_line_is_dropped_then_recovers(self):
        buffer = esp32_link.LineBuffer(max_line_bytes=10)
        self.assertEqual(buffer.feed(b'x' * 8), [])
        self.assertEqual(buffer.feed(b'x' * 8), [])
        self.assertEqual(buffer.overflows, 1)
        # The tail of the oversized line must not be returned as a line.
        self.assertEqual(buffer.feed(b'yyy\nok\n'), [b'ok'])
        self.assertEqual(buffer.overflows, 1)

    def test_oversized_line_in_one_chunk(self):
        buffer = esp32_link.LineBuffer(max_line_bytes=10)
        self.assertEqual(buffer.feed(b'x' * 50 + b'\nfine\n'), [b'fine'])
        self.assertEqual(buffer.overflows, 1)


if __name__ == '__main__':
    unittest.main()
