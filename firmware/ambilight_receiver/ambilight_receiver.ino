#include <WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>

// ===== LOCAL SETTINGS =====
// Wi-Fi credentials, multicast group/port and LED pin live in secrets.h
// (gitignored). Copy secrets.example.h -> secrets.h and edit it.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "secrets.h not found: copy secrets.example.h to secrets.h and fill in your Wi-Fi details"
#endif

// ===== STRIP CONFIG =====
#define LED_TYPE  WS2812B
#define COLOR_ORDER GRB
#define MAX_LEDS  120
#define BRIGHTNESS 180
// ========================

// Debug (compile-time)
#define DEBUG_LOGS 1
#if DEBUG_LOGS
  #define DBG(x) Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// Packet constants
#define MAGIC 0xA1B2
#define VERSION 2

// Header v2: 6 bytes original + 4 bytes t_send_us
#define HDR_LEN 10

WiFiUDP udp;
IPAddress mcast;
CRGB leds[MAX_LEDS];

static uint8_t rxbuf[HDR_LEN + 3*MAX_LEDS];

uint32_t lastStatMs = 0;
uint32_t frames = 0;
uint32_t drops = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  DBGLN("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    DBG(".");
  }
  DBGLN("\nWiFi connected");
  DBGLN(WiFi.localIP());

  mcast.fromString(MCAST_IP);
  if (!udp.beginMulticast(mcast, UDP_PORT)) {
    DBGLN("[ERR] beginMulticast failed");
    while (1) delay(1000);
  }
  DBGLN("[OK] Multicast joined");

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, MAX_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);
}

static inline uint32_t read_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    uint32_t now = millis();
    if (now - lastStatMs > 1000) {
      lastStatMs = now;
      if (DEBUG_LOGS) {
        Serial.print("FPS=");
        Serial.print(frames);
        Serial.print(" drops=");
        Serial.println(drops);
      }
      frames = 0;
      drops = 0;
    }
    return;
  }

  if (packetSize > (int)sizeof(rxbuf)) {
    drops++;
    udp.flush();
    return;
  }

  int n = udp.read(rxbuf, packetSize);
  if (n < HDR_LEN) { drops++; return; }

  // Parse header
  uint16_t magic = rxbuf[0] | (rxbuf[1] << 8);
  uint8_t  ver   = rxbuf[2];
  uint8_t  nleds = rxbuf[3];
  uint16_t seq   = rxbuf[4] | (rxbuf[5] << 8);
  uint32_t t_send_us = read_u32_le(rxbuf + 6);

  if (magic != MAGIC || ver != VERSION || nleds > MAX_LEDS) {
    drops++;
    return;
  }

  int expected = HDR_LEN + 3 * nleds;
  if (n != expected) {
    drops++;
    return;
  }

  // Copy RGBs (payload order R,G,B; FastLED outputs in COLOR_ORDER)
  uint8_t *p = rxbuf + HDR_LEN;
  for (int i = 0; i < nleds; i++) {
    uint8_t r = *p++;
    uint8_t g = *p++;
    uint8_t b = *p++;
    leds[i].setRGB(r, g, b);
  }

  uint32_t t_rx_us = micros();
  uint32_t t_before_show = micros();
  FastLED.show();
  uint32_t t_after_show = micros();

  frames++;

  if (DEBUG_LOGS && ((seq % 5) == 0)) {
    Serial.print("[ESP timing] seq=");
    Serial.print(seq);
    Serial.print(" t_send_us=");
    Serial.print(t_send_us);
    Serial.print(" t_rx_us=");
    Serial.print(t_rx_us);
    Serial.print(" show_us=");
    Serial.print(t_after_show - t_before_show);
    Serial.println();
  }
}
