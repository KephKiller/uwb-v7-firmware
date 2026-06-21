# UWB V7 — Arduino/ESP32 firmware (DWM3000, DS-TWR)

Open-source firmware for the **UWB V7** project: centimetre-grade indoor positioning via
**DS-TWR** (Double-Sided Two-Way Ranging) with the **Qorvo DWM3000** module.

Project site: https://effervescent-baklava-5bfaf4.netlify.app/

## Contents

| Folder / file | Role |
|---|---|
| `dw3000_anchor/` | **ANCHOR** firmware (ESP32-DevKitC). DS-TWR responder. Flash 3 anchors, changing `ANCHOR_ID` (1, 2, 3). |
| `dw3000_anchor_esp32_s3/` | **ANCHOR for ESP32-S3** variant. |
| `dw3000_tag_esp32/` | Mobile **TAG** firmware (ESP32-DevKitC). Polls the 3 anchors, sends distances over WiFi/UDP. |
| `dw3000_uwb_ranging/` | Combined **TWR demo** (anchor + tag in a single sketch), educational. |
| `dw3000_trilat.h` | Optional module: moving-average filter, antenna calibration, trilateration + WiFi/UDP. |
| `relais_uwb/` | **Node.js** UDP→SSE relay (tag → browser gateway) + tag simulator. |

## Requirements

1. **Arduino IDE** with **ESP32** board support (Espressif).
2. **Makerfabs DW3000** library to install:
   https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
   (the sketches `#include "dw3000.h"` — this library is not included here).

## Wiring (ESP32-DevKitC)

```
GPIO18 → CLK   GPIO23 → MOSI   GPIO19 → MISO   GPIO5 → CS   GPIO27 → IRQ   GPIO26 → RST
3.3V → VCC     GND → GND       (100 nF capacitor as close as possible to the DWM3000)
```
Detailed diagrams (ESP32, ESP32-S3, NodeMCU, Mega) on the site, **Resources** section.

## Configuration

- **Anchor**: set `ANCHOR_ID` (1, 2 or 3) before flashing each anchor.
- **Tag**: fill in `WIFI_SSID`, `WIFI_PASSWORD` and `RELAY_IP` (IP of the PC running
  `relais_uwb/relais.js`) — **the shipped values are placeholders**.

## Real-time chain (optional)

```
Tag (ESP32) --WiFi/UDP--> relais_uwb/relais.js --SSE--> 3D viewer (browser)
```
Start the relay: `cd relais_uwb && node relais.js` (UDP 8080, HTTP 8090, no dependencies).
Without hardware: `node simulateur_tag.js` emits dummy frames.

## License

MIT — see [LICENSE](LICENSE).
