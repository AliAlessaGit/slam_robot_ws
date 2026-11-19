// الوظيفة: استقبال بيانات الحساسات عبر ESP-NOW على ESP32-C3.
// الغرض: دمج قراءات الحساسات وطباعتها بصيغة JSON عبر USB التسلسلي.

#include <Arduino.h>
/*
  ESP32-C3 ESP-NOW RECEIVER
  Receives per-sensor packets from the C6 and prints the SAME JSON line your
  laptop code already understands:
    {"s1":[64],"s2":[64],"s3":[64],"bno":{"ok":1,"h":..,"r":..,"p":..,"cal":[..]}}

  Arduino IDE: Board "ESP32C3 Dev Module"
               USB CDC On Boot: Enabled (if the board uses its native USB port)
  Serial monitor / laptop code: 921600 baud
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

constexpr uint8_t  ESPNOW_CHANNEL    = 1;    // must match the sender
constexpr uint32_t PRINT_INTERVAL_MS = 50;   // 20 Hz output to the laptop
constexpr uint32_t STALE_MS          = 500;  // a sensor silent this long prints as -1

// Must be identical in the sender sketch
struct __attribute__((packed)) TofPacket {
  uint8_t  sensor;     // 0, 1 or 2
  uint8_t  bnoOk;
  uint16_t seq;        // per-sensor counter, used to count lost packets
  float    heading, roll, pitch;
  uint8_t  cal[4];     // sys, gyro, accel, mag
  int16_t  mm[64];     // distances in mm, -1 = invalid
};

QueueHandle_t rxQueue;

int16_t  grid[3][64];
uint32_t lastRxMs[3] = {0, 0, 0};
uint16_t lastSeq[3]  = {0, 0, 0};
bool     haveSeq[3]  = {false, false, false};

uint8_t bnoOk = 0;
float   bnoH = 0, bnoR = 0, bnoP = 0;
uint8_t bnoCal[4] = {0, 0, 0, 0};

uint32_t rxCount = 0, lostCount = 0;
uint32_t lastPrintMs = 0, lastMacMs = 0, lastLinkMs = 0;

// Runs in the Wi-Fi task: just hand the packet to the queue and return quickly
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(TofPacket)) return;
  xQueueSend(rxQueue, data, 0);
}

void handlePacket(const TofPacket &p) {
  if (p.sensor > 2) return;
  const int s = p.sensor;

  memcpy(grid[s], p.mm, sizeof(p.mm));
  lastRxMs[s] = millis();

  if (haveSeq[s]) {
    const uint16_t gap = (uint16_t)(p.seq - lastSeq[s]);
    if (gap > 1 && gap < 1000) lostCount += gap - 1;
  }
  haveSeq[s] = true;
  lastSeq[s] = p.seq;

  bnoOk = p.bnoOk;
  bnoH = p.heading;
  bnoR = p.roll;
  bnoP = p.pitch;
  memcpy(bnoCal, p.cal, 4);

  rxCount++;
}

void printFrame() {
  static char buf[1536];
  int n = 0;
  const uint32_t now = millis();

  for (int s = 0; s < 3; s++) {
    n += snprintf(buf + n, sizeof(buf) - n, "%s\"s%d\":[", s ? "," : "{", s + 1);
    const bool fresh = (now - lastRxMs[s]) < STALE_MS;
    for (int z = 0; z < 64; z++) {
      n += snprintf(buf + n, sizeof(buf) - n, z < 63 ? "%d," : "%d", fresh ? (int)grid[s][z] : -1);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]");
  }

  n += snprintf(buf + n, sizeof(buf) - n,
                ",\"bno\":{\"ok\":%d,\"h\":%.1f,\"r\":%.1f,\"p\":%.1f,\"cal\":[%d,%d,%d,%d]}}\n",
                bnoOk, bnoH, bnoR, bnoP, bnoCal[0], bnoCal[1], bnoCal[2], bnoCal[3]);

  Serial.write((uint8_t *)buf, n);
}

void setup() {
  Serial.begin(921600); // ignored on native USB, used on a UART port
  delay(1000);

  rxQueue = xQueueCreate(24, sizeof(TofPacket));
  for (int s = 0; s < 3; s++)
    for (int z = 0; z < 64; z++) grid[s][z] = -1;

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("{\"error\":\"ESP-NOW init failed\"}");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.println("{\"status\":\"C3 receiver ready\"}");
}

void loop() {
  // 1) Move received packets from the queue into the latest-data arrays
  TofPacket p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) handlePacket(p);

  const uint32_t now = millis();

  // 2) Until the first packet arrives, repeat the MAC address so you can copy it
  if (rxCount == 0 && now - lastMacMs >= 2000) {
    lastMacMs = now;
    Serial.printf("{\"status\":\"RECEIVER_MAC\",\"mac\":\"%s\"}\n", WiFi.macAddress().c_str());
  }

  // 3) Send the combined JSON line to the laptop at 20 Hz
  if (rxCount > 0 && now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printFrame();
  }

  // 4) Link quality line every 5 s (useful for range tests)
  if (rxCount > 0 && now - lastLinkMs >= 5000) {
    lastLinkMs = now;
    Serial.printf("{\"status\":\"LINK rx=%lu lost=%lu\"}\n",
                  (unsigned long)rxCount, (unsigned long)lostCount);
  }
}
