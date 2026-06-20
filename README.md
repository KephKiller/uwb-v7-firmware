# UWB V7 — Firmware Arduino/ESP32 (DWM3000, DS-TWR)

Firmware open-source du projet **UWB V7** : positionnement indoor centimétrique par
**DS-TWR** (Double-Sided Two-Way Ranging) avec le module **Qorvo DWM3000**.

Site du projet : https://effervescent-baklava-5bfaf4.netlify.app/

## Contenu

| Dossier / fichier | Rôle |
|---|---|
| `dw3000_anchor/` | Firmware **ANCRE** (ESP32-DevKitC). Répondeur DS-TWR. Flasher 3 ancres en changeant `ANCHOR_ID` (1, 2, 3). |
| `dw3000_anchor_esp32_s3/` | Variante **ANCRE pour ESP32-S3**. |
| `dw3000_tag_esp32/` | Firmware **TAG** mobile (ESP32-DevKitC). Interroge les 3 ancres, envoie les distances en WiFi/UDP. |
| `dw3000_uwb_ranging/` | Démo **TWR combinée** (ancre + tag dans un seul sketch), pédagogique. |
| `dw3000_trilat.h` | Module optionnel : filtre moyenneur, calibration d'antenne, trilatération + WiFi/UDP. |
| `relais_uwb/` | Relais **Node.js** UDP→SSE (passerelle tag → navigateur) + simulateur de tag. |

## Prérequis

1. **Arduino IDE** avec le support carte **ESP32** (Espressif).
2. **Bibliothèque Makerfabs DW3000** à installer :
   https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
   (les sketches font `#include "dw3000.h"` — cette bibliothèque n'est pas incluse ici).

## Câblage (ESP32-DevKitC)

```
GPIO18 → CLK   GPIO23 → MOSI   GPIO19 → MISO   GPIO5 → CS   GPIO27 → IRQ   GPIO26 → RST
3.3V → VCC     GND → GND       (condensateur 100 nF au plus près du DWM3000)
```
Schémas détaillés (ESP32, ESP32-S3, NodeMCU, Mega) sur le site, section **Ressources**.

## Configuration

- **Ancre** : régler `ANCHOR_ID` (1, 2 ou 3) avant de flasher chaque ancre.
- **Tag** : renseigner `WIFI_SSID`, `WIFI_PASSWORD` et `RELAY_IP` (IP du PC exécutant
  `relais_uwb/relais.js`) — **les valeurs livrées sont des placeholders**.

## Chaîne temps réel (optionnelle)

```
Tag (ESP32) --WiFi/UDP--> relais_uwb/relais.js --SSE--> visualiseur 3D (navigateur)
```
Lancer le relais : `cd relais_uwb && node relais.js` (UDP 8080, HTTP 8090, sans dépendances).
Sans matériel : `node simulateur_tag.js` émet des trames factices.

## Licence

MIT — voir [LICENSE](LICENSE).
