# Ambilight

Real-time ambient backlight for a PC monitor: a Python script captures the screen,
computes a color for each LED around the monitor's edges, and streams the colors
over Wi-Fi to an ESP32 driving a WS2812B LED strip. A WiZ smart bulb can optionally
follow the screen too.

```
Monitor ──MSS grab──► capture thread ──latest frame──► processing loop (30 fps)
                                                          │
   downscale ► edge bands ► chroma-weighted pooling ► 107 LED colors ► tune ► smooth
                                                          │
                        UDP multicast 239.1.1.1:7777 ─────┴──► ESP32 ► FastLED ► WS2812B
                        UDP JSON :38899 (optional) ──────────► WiZ bulb
```

## Repository layout

```
firmware/ambilight_receiver/
    ambilight_receiver.ino     ESP32 sketch: multicast receiver -> FastLED
    secrets.example.h          template for Wi-Fi / network / pin settings
sender/
    ambilight_sender.py        Windows screen-capture sender
    config_local.example.py    template for network / WiZ bulb settings
    requirements.txt
```

## Hardware

- ESP32 dev board
- WS2812B strip, 107 LEDs, data line on GPIO 5 (configurable)
- 5 V supply sized for the strip, with ground shared with the ESP32
- Windows PC on the same LAN as the ESP32 (multicast must be allowed on the network)

## Setup

### 1. Local settings (not committed)

Both halves read machine-specific values from a gitignored file. Copy the templates
and fill them in:

```bash
cp firmware/ambilight_receiver/secrets.example.h firmware/ambilight_receiver/secrets.h
cp sender/config_local.example.py sender/config_local.py
```

| File | Settings |
|---|---|
| `firmware/ambilight_receiver/secrets.h` | `WIFI_SSID`, `WIFI_PASS`, `MCAST_IP`, `UDP_PORT`, `LED_PIN` |
| `sender/config_local.py` | `MCAST_GRP`, `PORT`, `BULB_IP`, `BULB_PORT` |

The multicast group and port must match on both sides. If `config_local.py` is
missing, the sender falls back to the defaults (239.1.1.1:7777).

### 2. Flash the ESP32

1. Install the **ESP32** board package and the **FastLED** library in the Arduino IDE.
2. Open `firmware/ambilight_receiver/ambilight_receiver.ino`.
3. Select your ESP32 board and port, then upload.
4. Open the Serial Monitor at **115200** baud. You should see `[OK] Multicast joined`
   followed by `FPS=… drops=…` once a second.

### 3. Run the sender (Windows)

```bash
cd sender
pip install -r requirements.txt
python ambilight_sender.py
```

Press `Ctrl+C` to stop.

## Configuration

All tuning knobs live at the top of `sender/ambilight_sender.py`:

| Setting | Default | What it does |
|---|---|---|
| `MONITOR_PICK_MODE` | `external_largest` | Which screen to sample: `external_largest`, `primary` or `largest` |
| `TARGET_FPS` | 30 | Frame rate cap |
| `DOWNSCALE` | 4 | Pixel stride when sampling (higher = faster, coarser) |
| `BAND_THICKNESS_PX` | 60 | Edge band depth, in downscaled pixels |
| `BOTTOM_CENTER_SKIP_PX` | 200 | Gap in the bottom strip (e.g. monitor stand), downscaled px from center |
| `BOTTOM_SAMPLE_Y_OFFSET_PX` | 60 | Lift the bottom band above the taskbar / subtitles |
| `BRIGHTNESS_KNOB` | 0.2 | Overall strip brightness (0–1) |
| `SAT_BOOST` | 1.15 | Saturation multiplier |
| `WARMTH` | 0.10 | Shift toward red (positive) or blue (negative) |
| `SMOOTH_ALPHA` | 0.08 | Temporal smoothing; lower = smoother/slower |
| `ENABLE_CENTER_BLEND` | False | Mix the screen-center color into every LED |
| `ENABLE_WIZ_wiz` | False | Also drive a WiZ bulb |
| `USE_CAPTURE_THREAD` | True | Capture in a background thread |
| `FAST_WEIGHTS` | False | Plain mean instead of chroma-weighted pooling |
| `DEBUG` | False | Print per-stage timing and sample LED values |

### LED layout

The strip starts at the bottom center and runs clockwise (as seen from the front):

| Segment | LEDs | Direction |
|---|---|---|
| Bottom-left | 16 | center → left corner |
| Left | 20 | bottom → top |
| Top | 35 | left → right |
| Right | 20 | top → bottom |
| Bottom-right | 16 | right corner → center |

Change the `N_*` constants in the sender if your strip is laid out differently.

## Packet format (v2)

Little-endian, sent to the multicast group once per frame:

| Offset | Type | Field |
|---|---|---|
| 0 | `uint16` | magic `0xA1B2` |
| 2 | `uint8` | version `2` |
| 3 | `uint8` | LED count |
| 4 | `uint16` | sequence number |
| 6 | `uint32` | sender timestamp, µs (wraps) |
| 10 | `uint8[3 × count]` | R, G, B per LED |

The receiver drops any packet whose magic, version or length doesn't match.
