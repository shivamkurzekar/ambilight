// ===================================================================
//  Local settings for the ESP32 receiver.
//
//  1. Copy this file to "secrets.h" (same folder).
//  2. Fill in your own values.
//
//  secrets.h is gitignored, so your Wi-Fi credentials never get committed.
// ===================================================================
#pragma once

// ---- Wi-Fi ----
#define WIFI_SSID "your-wifi-ssid"
#define WIFI_PASS "your-wifi-password"

// ---- Network (must match MCAST_GRP / PORT in sender/config_local.py) ----
#define MCAST_IP  "239.1.1.1"
#define UDP_PORT  7777

// ---- Hardware ----
#define LED_PIN   5   // ESP32 GPIO wired to the strip's data line
