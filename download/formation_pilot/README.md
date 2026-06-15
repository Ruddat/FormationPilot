# FormationPilot 🛩️

**Platform-Agnostic Formation Flight Engine for INAV & ArduPilot**

Leader-Follower Formationsflug-System mit Lora-Funkverbindung. Der Leader fliegt normal, die Follower empfangen ihre Zielposition per Funk und folgen automatisch.

## Architektur

```
┌──────────────────────────────────┐
│         LEADER (Flugzeug 1)       │
│  ┌─────────┐    ┌──────────────┐ │
│  │ INAV oder│───>│ Raspberry Pi │ │
│  │ArduPilot │    │ Formation    │ │
│  │   FC     │    │ Engine       │ │
│  └─────────┘    └──────┬───────┘ │
│                        │ Lora    │
└────────────────────────┼─────────┘
                         │
          ┌──────────────┼──────────────┐
          ▼              ▼              ▼
   ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
   │  FOLLOWER 1  │ │  FOLLOWER 2  │ │  FOLLOWER 3  │
   │  ESP32+Lora  │ │  ESP32+Lora  │ │  ESP32+Lora  │
   │  ──> FC (WP) │ │  ──> FC (WP) │ │  ──> FC (WP) │
   └─────────────┘ └─────────────┘ └─────────────┘
```

## Features

- **INAV + ArduPilot Support** – Auto-Detection der FC-Firmware
- **6 Formationstypen** – V-Shape, Line, Echelon L/R, Circle, Custom
- **Lora Funkverbindung** – Bis zu 3km Reichweite (SF7), 10km (SF12)
- **Kompaktes Protokoll** – ~49 Bytes für Leader + 3 Follower
- **Failsafe-System** – Link-Lost → RTH, Geo-Fence, Min/Max-Distanz
- **Runtime Formation-Wechsel** – Formationstyp im Flug änderbar
- **MAVLink + MSP** – MAVLink für Position, MSP für INAV-Befehle

## Projektstruktur

```
formation_pilot/
├── main.py                     # Entry Point (mit Demo-Modus)
├── config.yaml                 # Konfiguration
├── requirements.txt            # Python Dependencies
└── formation/
    ├── __init__.py
    ├── formations.py           # Formation Calculator (Offset-Mathematik)
    ├── mavlink_adapter.py      # MAVLink Kommunikation
    ├── msp_adapter.py          # MSP Kommunikation (INAV)
    ├── fc_adapter.py           # Unified FC Interface + Auto-Detection
    ├── lora_broadcaster.py     # Lora Funkprotokoll
    ├── failsafe.py             # Failsafe Manager
    └── formation_engine.py     # Main Engine (Orchestrierung)
```

## Quick Start

### Demo (ohne Hardware)

```bash
cd formation_pilot
python3 main.py --demo
```

### Auf dem Raspberry Pi (Leader)

1. **Abhängigkeiten installieren:**
   ```bash
   pip3 install -r requirements.txt
   ```

2. **Hardware verkabeln:**
   - FC UART → Pi `/dev/serial0` (MAVLink, 57600 baud)
   - Lora Modul → Pi `/dev/serial1` (9600 baud)
   - Pi Stromversorgung (5V, 2A+)

3. **Konfiguration anpassen:**
   ```bash
   nano config.yaml
   ```
   - FC-Typ: `auto`, `inav`, oder `ardupilot`
   - Formation: `v_shape`, `line`, `echelon_right`, `echelon_left`, `circle`, `custom`
   - Follower-IDs und Offsets definieren
   - Lora-Kanal und SF anpassen

4. **Engine starten:**
   ```bash
   python3 main.py config.yaml
   ```

### INAV FC Konfiguration

Auf dem INAV Flight Controller muss folgendes konfiguriert sein:

1. **MAVLink aktivieren:**
   - CLI: `serial X 2 115200 57600 0 115200` (MAVLink auf UART X)
   - Oder in Configurator: Serial → Port X → MAVLink

2. **Navigation aktivieren:**
   - WP Mode aktivieren (NAV WP in Modes Tab)
   - GPS muss 3D-Fix haben
   - RTH als Failsafe konfigurieren

3. **Failsafe:**
   - Failsafe → RTH (nicht DROP oder LAND)
   - Das Formation-System verlässt sich auf INAV-eigene Failsafes

### ArduPilot FC Konfiguration

1. **MAVLink aktivieren:**
   - SERIALX_PROTOCOL = 1 (MAVLink v1) oder 2 (MAVLink v2)
   - SERIALX_BAUD = 57 (57600)

2. **NAV/RTL:**
   - RTL aktiv als Failsafe-Action
   - GPS → 3D Fix erforderlich

## Formationstypen

### V-Shape (Standard)
```
    F2          F1
      \       /
       \     /
        LEADER
          |
         F3
```
- F1/F2: spacing rechts/links, spacing*0.5 hinten
- F3: 2*spacing rechts, spacing hinten

### Line
```
    LEADER
      |
     F1
      |
     F2
      |
     F3
```
- Alle Follower in einer Linie hinter dem Leader

### Echelon Right/Left
```
    LEADER
       F1
          F2
             F3
```
- Alle Follower auf einer Seite gestaffelt

### Circle
```
       F2
    F3    F1
    LEADER
       F4
```
- Follower gleichmäßig auf einem Kreis um den Leader
- Kreis rotiert mit dem Leader-Heading

### Custom
- Frei definierbare Offsets pro Follower
- `offset_right`, `offset_behind`, `offset_above` in Metern

## Lora Protokoll

### Paketstruktur
```
┌──────┬──────┬──────┬──────┬─────────┬──────┐
│ 0xAA │ TYPE │ SEQ  │ LEN  │ PAYLOAD │ CRC8 │
│  1B  │  1B  │  1B  │  1B  │ var     │  1B  │
└──────┴──────┴──────┴──────┴─────────┴──────┘
```

### Position Encoding
| Feld | Encoding | Bereich | Präzision |
|------|----------|---------|-----------|
| Lat/Lon | deg × 1e7 (int32) | ±180° | ~1cm |
| Alt | dm (uint16) | 0-6553m | 10cm |
| Heading | cdeg (uint16) | 0-360° | 0.01° |
| Speed | dm/s (uint8) | 0-25.5 m/s | 0.1 m/s |

### Paket-Beispiel (Leader + 3 Follower)
- Header: 4 Bytes
- Leader: 16 Bytes
- 3 Follower: 33 Bytes
- CRC: 1 Byte
- **Total: 54 Bytes** (passt in ein Lora-Paket)

## Failsafe System

| Bedingung | Schwellwert | Aktion |
|-----------|-------------|--------|
| Lora-Link verloren | 3s | **RTH** |
| Position veraltet | 2s | **HOLD** |
| Follower zu weit | 100m | **HOLD** |
| Follower zu nah | 10m | **WARN** |
| Geo-Fence verletzt | 500m | **RTH** |
| Leader zu schnell | 25 m/s | **WARN** |
| Höhenabweichung | 30m | **WARN** |
| GPS schwach | <6 Sats | **WARN** |

**Wichtig:** Die FC-eigenen Failsafes (RTH, GPS-Failsafe) greifen IMMER zusätzlich und haben Priorität!

## Hardware

### Leader
| Komponente | Empfehlung | Preis |
|-----------|-----------|-------|
| Companion Computer | Raspberry Pi Zero 2W | ~15€ |
| Lora Modul | SX1278 / RFM95W 433MHz | ~5€ |
| Kabel | JST-SH 1.0mm | ~3€ |

### Follower
| Komponente | Empfehlung | Preis |
|-----------|-----------|-------|
| MCU | ESP32 + Lora (TTGO Lora32) | ~8€ |
| Alternativ | ESP32 + separates Lora-Modul | ~10€ |

### Wiring (Raspberry Pi)
```
Pi GPIO 14 (TXD) ──────── FC RX (MAVLink)
Pi GPIO 15 (RXD) ──────── FC TX (MAVLink)

Pi /dev/serial1 TX ────── Lora RX
Pi /dev/serial1 RX ────── Lora TX
Pi 3.3V ───────────────── Lora VCC
Pi GND ────────────────── Lora GND
```

## Nächste Schritte

1. **ESP32 Follower Firmware** – C/Arduino Code für die Follower-Module
2. **Web-Konfigurations-UI** – Flask-App auf dem Pi für Live-Monitoring
3. **Integrationstests** – Mit realen INAV/AP Flight Controllern
4. **Dokumentation** – Fotos, Wiring-Diagrams, Video-Tutorial

## Lizenz

MIT License – Frei nutzbar und modifizierbar.
