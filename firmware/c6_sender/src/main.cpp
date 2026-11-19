// الوظيفة: قراءة حساسات VL53L5CX ووحدة BNO055 على ESP32-C6.
// الغرض: إرسال القياسات إلى ESP32-C3 باستخدام ESP-NOW.

#include <Arduino.h>
/*
  3x SparkFun VL53L5CX (8x8 ToF) + Adafruit BNO055 -> ESP-NOW sender (ESP32-C6)
  Same sensor code as your working sketch. The only change is HOW data leaves:
  instead of a JSON line over USB every 50 ms, each ToF sensor sends its own
  small packet over ESP-NOW the moment it has a new reading.

  All four I2C devices share ONE bus (Wire). BNO055 must be at 0x28 (ADR to GND).
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

// ------------------------------ ESP-NOW settings ------------------------------
// MAC address of the C3 receiver: 58:8C:81:AD:9F:B0
// (check it matches what the C3 prints in its serial monitor)
uint8_t RECEIVER_MAC[6] = {0x58, 0x8C, 0x81, 0xAD, 0x9F, 0xB0};
constexpr uint8_t ESPNOW_CHANNEL = 1; // must match the receiver

// Must be identical in the receiver sketch
struct __attribute__((packed)) TofPacket {
  uint8_t  sensor;     // 0, 1 or 2
  uint8_t  bnoOk;
  uint16_t seq;        // per-sensor counter
  float    heading, roll, pitch;
  uint8_t  cal[4];     // sys, gyro, accel, mag
  int16_t  mm[64];     // distances in mm, -1 = invalid
};

uint16_t txSeq[3] = {0, 0, 0};
uint32_t sentOk = 0, sendFail = 0;
uint32_t lastStatusMs = 0;

// ------------------------------ Shared I2C bus (BNO055 + 3x ToF) ------------------------------
constexpr int I2C_SDA = 22; // XIAO ESP32-C6 pad D4 (SDA)
constexpr int I2C_SCL = 23; // XIAO ESP32-C6 pad D5 (SCL)
TwoWire &TOF_WIRE = Wire;
TwoWire &BNO_WIRE = Wire;

// ------------------------------ ToF Settings ------------------------------
constexpr int NUM_SENSORS = 3;
constexpr int RST_PIN[NUM_SENSORS] = {0, 1, 2}; // XIAO pads D0, D1, D2
constexpr uint8_t DEFAULT_ADDR = 0x29;
constexpr uint8_t FINAL_ADDR[NUM_SENSORS] = {0x30, 0x31, 0x32};

constexpr uint32_t TOF_I2C_HZ = 400000;
constexpr uint8_t RANGING_HZ = 15; // Max frequency for 8x8 resolution

SparkFun_VL53L5CX imager[NUM_SENSORS];
VL53L5CX_ResultsData tofData[NUM_SENSORS];
int gridMm[NUM_SENSORS][64];

// ------------------------------ BNO055 Settings ------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &BNO_WIRE);
bool bnoOk = false;

constexpr int BOOT_BTN_PIN = 9; // Physical BOOT button on the ESP32-C6 to erase calibration

// Latest BNO055 reading, refreshed at most every 20 ms
float bnoHeading = 0.0f, bnoRoll = 0.0f, bnoPitch = 0.0f;
uint8_t bnoCal[4] = {0, 0, 0, 0};
uint32_t lastBnoMs = 0;
constexpr uint32_t BNO_MIN_INTERVAL_MS = 20;

// ------------------------------ BNO055 Calibration Helpers ------------------------------
Preferences bnoPrefs;
bool bnoCalStored = false; // true once a calibration is in flash (loaded or saved)

// Reads the saved offsets from flash and writes them into the BNO055.
bool bnoLoadCalibration() {
  bool restored = false;
  bnoPrefs.begin("bno055", false);
  if (bnoPrefs.isKey("cal") && bnoPrefs.getBytesLength("cal") == sizeof(adafruit_bno055_offsets_t)) {
    adafruit_bno055_offsets_t offsets;
    bnoPrefs.getBytes("cal", &offsets, sizeof(offsets));
    bno.setSensorOffsets(offsets);
    restored = true;
  }
  bnoPrefs.end();
  return restored;
}

// Reads the offsets out of the BNO055 and stores them in flash.
bool bnoSaveCalibration() {
  adafruit_bno055_offsets_t offsets;
  if (!bno.getSensorOffsets(offsets)) return false;
  bnoPrefs.begin("bno055", false);
  bnoPrefs.putBytes("cal", &offsets, sizeof(offsets));
  bnoPrefs.end();
  return true;
}

// Call in setup() right after bno.begin() succeeded.
void bnoRestoreCalibration() {
  bnoCalStored = bnoLoadCalibration();
  bno.setExtCrystalUse(true); // must come AFTER the offsets have been written
  Serial.println(bnoCalStored ? "{\"status\":\"BNO055 calibration restored\"}"
                              : "{\"status\":\"BNO055 has no saved calibration\"}");
}

// Call after bno.getCalibration(). Saves once when all four values hit 3.
void bnoAutoSave(uint8_t sys, uint8_t gyro, uint8_t accel, uint8_t mag) {
  if (bnoCalStored) return;
  if (sys == 3 && gyro == 3 && accel == 3 && mag == 3 && bnoSaveCalibration()) {
    bnoCalStored = true;
    Serial.println("{\"status\":\"CAL_SAVED\"}");
  }
}

void bnoEraseAndReboot() {
  bnoPrefs.begin("bno055", false);
  bnoPrefs.remove("cal");
  bnoPrefs.end();
  Serial.println("{\"status\":\"CAL_ERASED - rebooting\"}");
  Serial.flush();
  delay(100);
  ESP.restart();
}

// Call at the top of loop().
void bnoHandleCommands() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 's') {
      if (bnoOk && bnoSaveCalibration()) {
        bnoCalStored = true;
        Serial.println("{\"status\":\"CAL_SAVED\"}");
      } else {
        Serial.println("{\"status\":\"CAL_NOT_SAVED - BNO055 not fully calibrated yet\"}");
      }
    } else if (c == 'e') {
      bnoEraseAndReboot();
    }
  }
  // Also allow pressing the physical BOOT button to erase & reboot
  if (digitalRead(BOOT_BTN_PIN) == LOW) {
    delay(50);
    if (digitalRead(BOOT_BTN_PIN) == LOW) {
      bnoEraseAndReboot();
    }
  }
}

// Refresh the cached BNO055 values (at most every 20 ms)
void updateBno() {
  if (!bnoOk) return;
  const uint32_t now = millis();
  if (now - lastBnoMs < BNO_MIN_INTERVAL_MS) return;
  lastBnoMs = now;

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bnoHeading = orientationData.orientation.x;
  bnoRoll    = orientationData.orientation.y;
  bnoPitch   = orientationData.orientation.z;

  uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  bnoCal[0] = sys; bnoCal[1] = gyro; bnoCal[2] = accel; bnoCal[3] = mag;
  bnoAutoSave(sys, gyro, accel, mag);
}

// ------------------------------ ESP-NOW helpers ------------------------------
bool startEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return false;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendGrid(int i) {
  TofPacket p;
  p.sensor  = (uint8_t)i;
  p.bnoOk   = bnoOk ? 1 : 0;
  p.seq     = txSeq[i]++;
  p.heading = bnoHeading;
  p.roll    = bnoRoll;
  p.pitch   = bnoPitch;
  memcpy(p.cal, bnoCal, 4);
  for (int z = 0; z < 64; z++) p.mm[z] = (int16_t)gridMm[i][z];

  if (esp_now_send(RECEIVER_MAC, (const uint8_t *)&p, sizeof(p)) == ESP_OK) sentOk++;
  else sendFail++;
}

// ------------------------------ I2C Helpers ------------------------------
bool ack(uint8_t addr) {
  TOF_WIRE.beginTransmission(addr);
  return TOF_WIRE.endTransmission() == 0;
}

const uint8_t PROBE[4] = {DEFAULT_ADDR, FINAL_ADDR[0], FINAL_ADDR[1], FINAL_ADDR[2]};

uint8_t answering() {
  uint8_t mask = 0;
  for (int k = 0; k < 4; k++) {
    if (ack(PROBE[k])) mask |= (1 << k);
  }
  return mask;
}

void halt() {
  Serial.println("{\"error\":\"HALTED - Check Serial log above and power-cycle.\"}");
  while (true) delay(1000);
}

bool startSensor(int i) {
  const int n = i + 1;
  const uint8_t before = answering();

  digitalWrite(RST_PIN[i], LOW);

  uint8_t appeared = 0;
  const uint32_t t0 = millis();
  while (appeared == 0 && millis() - t0 < 500) {
    delay(10);
    appeared = answering() & ~before;
  }

  if (appeared == 0 || (appeared & (appeared - 1))) {
    Serial.printf("{\"status\":\"Sensor %d failed to appear uniquely on bus\"}\n", n);
    return false;
  }

  uint8_t current = DEFAULT_ADDR;
  for (int k = 0; k < 4; k++) {
    if (appeared & (1 << k)) current = PROBE[k];
  }

  Serial.printf("{\"status\":\"Sensor %d awake at 0x%02X, loading firmware...\"}\n", n, current);
  if (!imager[i].begin(current, TOF_WIRE)) {
    Serial.printf("{\"status\":\"Sensor %d begin() failed\"}\n", n);
    return false;
  }

  if (current != FINAL_ADDR[i]) {
    if (!imager[i].setAddress(FINAL_ADDR[i])) {
      Serial.printf("{\"status\":\"Sensor %d setAddress failed\"}\n", n);
      return false;
    }
  }

  Serial.printf("{\"status\":\"Sensor %d ready at 0x%02X\"}\n", n, imager[i].getAddress());
  return true;
}

// ------------------------------ Setup ------------------------------
void setup() {
  Serial.begin(921600); // Ignored on native USB CDC, used on a UART port
  delay(1000);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  Serial.println("\n{\"status\":\"Starting 3x VL53L5CX + BNO055 (ESP-NOW sender)...\"}");

  for (int s = 0; s < NUM_SENSORS; s++) {
    for (int z = 0; z < 64; z++) gridMm[s][z] = -1;
  }

  // 1) Start the shared I2C bus
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(TOF_I2C_HZ);

  // 2) Initialize BNO055 (address 0x28, ADR pin to GND)
  if (bno.begin()) {
    bnoOk = true;
    bnoRestoreCalibration();
  } else {
    Serial.println("{\"status\":\"WARNING: BNO055 not found at 0x28!\"}");
  }

  // bno.begin() may reset the bus speed, so set it again
  Wire.setClock(TOF_I2C_HZ);

  // 3) Initialize 3x VL53L5CX on the same bus
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(RST_PIN[i], OUTPUT);
    digitalWrite(RST_PIN[i], HIGH);
  }
  delay(100);

  if (answering() != 0) {
    Serial.println("{\"status\":\"ERROR: A ToF sensor still answers while RST is HIGH.\"}");
    halt();
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!startSensor(i)) halt();
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    const bool ok = imager[i].setResolution(8 * 8)
                 && imager[i].setRangingFrequency(RANGING_HZ)
                 && imager[i].setIntegrationTime(20)  // 20ms fits inside 15Hz (66ms) window
                 && imager[i].setSharpenerPercent(15) // 15% reduces light bleeding between zones
                 && imager[i].startRanging();
    if (!ok) {
      Serial.printf("{\"status\":\"Sensor %d startRanging failed\"}\n", i + 1);
      halt();
    }
  }

  // 4) Start ESP-NOW last, so the radio starting up can't disturb sensor init
  if (!startEspNow()) {
    Serial.println("{\"error\":\"ESP-NOW start failed (check MAC address / channel)\"}");
    halt();
  }

  Serial.println(bnoCalStored ? "{\"status\":\"ALL_READY (Calibration restored!) - sending over ESP-NOW\"}"
                              : "{\"status\":\"ALL_READY (Calibrate to 3/3/3/3 to auto-save) - sending over ESP-NOW\"}");
}

// ------------------------------ Loop ------------------------------
void loop() {
  bnoHandleCommands();

  // Poll all 3 ToF sensors; send each one the moment it has a new reading
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (imager[i].isDataReady() && imager[i].getRangingData(&tofData[i])) {
      int idx = 0;
      for (int y = 0; y <= 8 * (8 - 1); y += 8) {
        for (int x = 8 - 1; x >= 0; x--) {
          int z = x + y;
          uint8_t st = tofData[i].target_status[z];
          int16_t d  = tofData[i].distance_mm[z];

          // Accept only 5 (100% valid) or 9 (50% confidence valid); reset everything else to -1
          if ((st == 5 || st == 9) && d > 0) {
            gridMm[i][idx] = d;
          } else {
            gridMm[i][idx] = -1;
          }
          idx++;
        }
      }
      updateBno();
      sendGrid(i);
    }
  }

  // Small status line every 5 s (visible only if the C6 is connected to a laptop)
  const uint32_t now = millis();
  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    Serial.printf("{\"status\":\"TX ok=%lu fail=%lu\"}\n", (unsigned long)sentOk, (unsigned long)sendFail);
  }
}
