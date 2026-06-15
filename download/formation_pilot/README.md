# FormationPilot ✈️

**Platform-Agnostic Formation Flight Engine for INAV & ArduPilot**

Leader-Follower Formationsflug-System mit Lora-Funkverbindung und Live-Web-Dashboard. Der Leader fliegt normal, die Follower empfangen ihre Zielposition per Funk und folgen automatisch.

## Architektur

```
┌──────────────────────────────────────────────┐
│              LEADER (Flugzeug 1)               │
│  ┌──────────┐  ┌──────────────┐  ┌────────┐  │
│  │ INAV oder │─>│ Raspberry Pi │─>│  Lora  │  │
│  │ArduPilot  │  │ Formation    │  │ Sender │  │
│  │   FC      │  │ Engine       │  │        │  │
│  └──────────┘  │  + Web UI    │  └───┬────┘  │
│                └──────────────┘      │       │
└─────────────────────────────────────┼───────┘
                                      │ Lora 433MHz
           ┌──────────────────────────┼──────────────┐
           ▼                          ▼              ▼
    ┌─────────────┐          ┌─────────────┐  ┌─────────────┐
    │  FOLLOWER 1  │          │  FOLLOWER 2  │  │  FOLLOWER 3  │
    │  ESP32+Lora  │          │  ESP32+Lora  │  │  ESP32+Lora  │
    │  ──> INAV FC │          │  ──> AP FC   │  │  ──> INAV FC │
    └─────────────┘          └─────────────┘  └─────────────┘

    📱 Handy/Laptop ── WiFi ──> Pi Web Dashboard (Port 5000)
```

## Features

- **INAV + ArduPilot Support** – Auto-Detection der FC-Firmware
- **6 Formationstypen** – V-Shape, Line, Echelon L/R, Circle, Custom
- **Lora Funkverbindung** – Bis zu 3km Reichweite (SF7), 10km (SF12)
- **Kompaktes Protokoll** – ~54 Bytes für Leader + 3 Follower
- **Failsafe-System** – Link-Lost → RTH, Geo-Fence, Min/Max-Distanz
- **Runtime Formation-Wechsel** – Formationstyp im Flug änderbar
- **MAVLink + MSP** – MAVLink für Position, MSP für INAV-Befehle
- **Web Dashboard** – Live-Karte mit Flugzeug-Icons, Formation-Controls, Failsafe-Status
- **Interaktiver Demo-Modus** – Ohne Hardware testbar

## Projektstruktur

```
formation_pilot/
├── main.py                     # Entry Point (Demo + Web + Engine)
├── config.yaml                 # Konfiguration
├── requirements.txt            # Python Dependencies
├── README.md                   # Diese Datei
├── formation/
│   ├── __init__.py
│   ├── formations.py           # Formation Calculator (Offset-Mathematik)
│   ├── mavlink_adapter.py      # MAVLink Kommunikation
│   ├── msp_adapter.py          # MSP Kommunikation (INAV)
│   ├── fc_adapter.py           # Unified FC Interface + Auto-Detection
│   ├── lora_broadcaster.py     # Lora Funkprotokoll
│   ├── failsafe.py             # Failsafe Manager
│   └── formation_engine.py     # Main Engine (Orchestrierung)
└── web/
    ├── __init__.py             # Flask Web App + API
    ├── app.py                  # Standalone Web Start
    └── templates/
        └── index.html          # Dashboard (Karte + Controls)
```

## Quick Start

### 1. Projekt klonen

```bash
git clone https://github.com/Ruddat/FormationPilot.git
cd FormationPilot
```

### 2. Abhängigkeiten installieren

```bash
python -m venv venv

# Windows
.\venv\Scripts\activate
# Mac/Linux
source venv/bin/activate

pip install -r requirements.txt
```

### 3. Web-Demo starten (empfohlen!)

```bash
python main.py --web
```

Dann Browser auf **http://localhost:5000** – du siehst:
- Live-Karte mit animierten Flugzeug-Icons (Leader fliegt Kreis)
- 3 Follower in Formation mit gestrichelten Verbindungslinien
- Formation-Typ live wechselbar (V-Shape, Line, Echelon, Kreis)
- Spacing und Höhen-Offset per Slider einstellbar
- Failsafe-Status und Notfall-Buttons (HOLD, RTH, LAND)

### 4. Terminal-Demo (alternativ)

```bash
python main.py --demo
```

Interaktiver Modus mit Tastatursteuerung:
- `[1]`-`[6]` Formation wechseln
- `[+]`/`[-]` Spacing ändern
- `[a]`/`[z]` Höhen-Offset
- `[q]` Beenden

### 5. Echter Flugbetrieb (Raspberry Pi)

1. **Hardware verkabeln:**
   - FC UART → Pi `/dev/serial0` (MAVLink, 57600 baud)
   - Lora Modul → Pi `/dev/serial1` (9600 baud)
   - Pi Stromversorgung (5V, 2A+)

2. **Konfiguration anpassen:**
   ```bash
   nano config.yaml
   ```

3. **Engine starten:**
   ```bash
   python3 main.py config.yaml
   ```

4. **Dashboard öffnen:** Handy-Browser → `http://<pi-ip>:5000`

## Web Dashboard

Das Dashboard läuft auf dem Pi und ist von jedem Gerät im selben WLAN erreichbar:

| Feature | Beschreibung |
|---------|-------------|
| 🗺️ **Live-Karte** | Leaflet.js mit Flugzeug-SVG-Icons, Heading-Rotation, Trail |
| 🔀 **Formation-Selector** | 6 Formationen per Klick wechseln |
| 🎚️ **Spacing-Slider** | 5m bis 100m Abstand einstellen |
| 🎚️ **Höhen-Offset** | ±50m Höhenversatz |
| 🛩️ **Follower-Cards** | Distanz, Peilung, Offset pro Follower |
| 🛡️ **Failsafe-Status** | Lora, GPS, Geo-Fence, Abstand |
| 🚨 **Notfall-Buttons** | HOLD, RTH, LAND, RESUME |
| 📱 **Responsive** | Dark Theme, Handy-tauglich |
| ⚡ **Real-Time** | WebSocket Updates (5Hz) |

## FC Konfiguration

### INAV

1. **MAVLink aktivieren:**
   - CLI: `serial X 2 115200 57600 0 115200`
   - Oder Configurator: Serial → Port X → MAVLink

2. **Navigation:**
   - WP Mode aktivieren (NAV WP in Modes Tab)
   - GPS 3D-Fix erforderlich
   - RTH als Failsafe

3. **Failsafe:**
   - Failsafe → RTH (nicht DROP oder LAND)

### ArduPilot

1. **MAVLink aktivieren:**
   - SERIALX_PROTOCOL = 1 (MAVLink v1) oder 2 (MAVLink v2)
   - SERIALX_BAUD = 57 (57600)

2. **NAV/RTL:**
   - RTL als Failsafe-Action
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

### Echelon Right/Left
```
    LEADER
       F1
          F2
             F3
```

### Circle
```
       F2
    F3    F1
    LEADER
       F4
```
Kreis rotiert mit dem Leader-Heading.

### Custom
Frei definierbare Offsets pro Follower in `config.yaml`:
```yaml
followers:
  - id: 1
    offset_right: 20    # Meter rechts (negativ = links)
    offset_behind: 5     # Meter hinter (negativ = vor)
    offset_above: 0      # Meter über (negativ = unter)
```

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
- Header: 4 Bytes | Leader: 16 Bytes | 3 Follower: 33 Bytes | CRC: 1 Byte
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

Die FC-eigenen Failsafes (RTH, GPS-Failsafe) greifen IMMER zusätzlich und haben Priorität!

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

## API Endpunkte

Das Web-Dashboard nutzt folgende REST-API:

| Endpoint | Methode | Beschreibung |
|----------|---------|-------------|
| `/api/state` | GET | Aktueller Formation-Status (JSON) |
| `/api/formations` | GET | Verfügbare Formationstypen |
| `/api/formation/change` | POST | Formation wechseln |
| `/api/follower/<id>/command` | POST | Befehl an Follower |
| `/api/failsafe/rules` | GET | Failsafe-Regeln |
| `/api/config` | GET | Aktuelle Konfiguration |

WebSocket Events: `state_update`, `formation_changed`

## Nächste Schritte

- [ ] **ESP32 Follower Firmware** – Arduino Code für die Follower-Module
- [ ] **Integrationstests** – Mit realen INAV/AP Flight Controllern
- [ ] **Lora Module Konfiguration** – AT-Command Setup automatisieren
- [ ] **Wiring Diagrams** – Fotos und Fritzing-Pläne
- [ ] **Video-Tutorial** – Setup und Erstflug-Doku

## Lizenz

MIT License – Frei nutzbar und modifizierbar.
