/**
 * Generate wiring diagrams for FormationPilot as SVG files.
 * Leader: Raspberry Pi + FC + Lora
 * Follower: ESP32 + FC + Lora + LEDs + Button
 */

const fs = require('fs');
const path = require('path');

const outputDir = path.join(__dirname, '..', 'docs');

// ============================================================================
// Color palette
// ============================================================================
const COLORS = {
  bg: '#1a1a2e',
  card: '#16213e',
  cardStroke: '#0f3460',
  text: '#e0e0e0',
  textDim: '#8892a0',
  accent: '#e94560',
  accentBlue: '#0099ff',
  accentGreen: '#00cc66',
  accentOrange: '#ff9900',
  accentPurple: '#cc66ff',
  accentYellow: '#ffcc00',
  wire: '#555555',
  wireActive: '#00cc66',
  wireTx: '#e94560',
  wireRx: '#0099ff',
  wireSpi: '#cc66ff',
  wireGpio: '#ff9900',
  wirePower: '#ffcc00',
  wireGnd: '#888888',
};

function makeSvg(content, width = 900, height = 600) {
  return `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" width="${width}" height="${height}">
  <defs>
    <style>
      @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&amp;family=JetBrains+Mono:wght@400;600&amp;display=swap');
      text { font-family: 'Inter', 'Segoe UI', Arial, sans-serif; }
      .mono { font-family: 'JetBrains Mono', 'Consolas', monospace; }
      .board { rx: 8; ry: 8; }
      .pin-label { font-size: 11px; fill: ${COLORS.textDim}; }
      .pin-name { font-size: 12px; font-weight: 600; }
      .wire { stroke-width: 2.5; fill: none; stroke-linecap: round; }
      .wire-dash { stroke-dasharray: 6 3; }
      .title { font-size: 22px; font-weight: 700; fill: ${COLORS.text}; }
      .subtitle { font-size: 13px; fill: ${COLORS.textDim}; }
      .conn-label { font-size: 10px; font-weight: 600; }
      .note { font-size: 11px; fill: ${COLORS.accentOrange}; font-style: italic; }
    </style>
    <filter id="glow">
      <feGaussianBlur stdDeviation="2" result="blur"/>
      <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
    </filter>
    <marker id="arrowR" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto">
      <path d="M0,0 L8,3 L0,6 Z" fill="${COLORS.wireRx}"/>
    </marker>
    <marker id="arrowT" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto">
      <path d="M0,0 L8,3 L0,6 Z" fill="${COLORS.wireTx}"/>
    </marker>
  </defs>
  ${content}
</svg>`;
}

// ============================================================================
// Component drawing helpers
// ============================================================================
function drawBoard(x, y, w, h, label, sublabel, color, pins = []) {
  let html = `
  <rect class="board" x="${x}" y="${y}" width="${w}" height="${h}"
        fill="${COLORS.card}" stroke="${color}" stroke-width="2"/>
  <rect x="${x}" y="${y}" width="${w}" height="32" rx="8" ry="8"
        fill="${color}" opacity="0.15"/>
  <text x="${x + w/2}" y="${y + 21}" text-anchor="middle"
        style="font-size:14px;font-weight:700;fill:${color}">${label}</text>
  <text x="${x + w/2}" y="${y + 46}" text-anchor="middle"
        class="subtitle">${sublabel}</text>`;

  // Draw pin labels
  pins.forEach((pin, i) => {
    const side = pin.side || 'right';
    let px, py, anchor;
    if (side === 'right') {
      px = x + w + 4;
      py = y + 60 + i * 22;
      anchor = 'start';
    } else if (side === 'left') {
      px = x - 4;
      py = y + 60 + i * 22;
      anchor = 'end';
    } else if (side === 'bottom') {
      px = x + 30 + i * 80;
      py = y + h + 16;
      anchor = 'middle';
    }
    html += `
  <circle cx="${side === 'right' ? x + w : side === 'left' ? x : px}" cy="${py - 4}" r="3" fill="${pin.color || COLORS.wire}"/>
  <text x="${px}" y="${py}" text-anchor="${anchor}" class="mono pin-name" fill="${pin.color || COLORS.textDim}">${pin.label}</text>`;
  });

  return html;
}

function drawWire(x1, y1, x2, y2, color, label = '', dashed = false) {
  const dashAttr = dashed ? ' class="wire wire-dash"' : ' class="wire"';
  let html = `\n  <path${dashAttr} stroke="${color}" d="M${x1},${y1}`;
  // Simple routing: horizontal then vertical or vice versa
  if (Math.abs(x2 - x1) > 5 && Math.abs(y2 - y1) > 5) {
    // L-shaped routing
    const midX = (x1 + x2) / 2;
    html += ` H${midX} V${y2} H${x2}`;
  } else {
    html += ` L${x2},${y2}`;
  }
  html += `"/>`;

  if (label) {
    const mx = (x1 + x2) / 2;
    const my = (y1 + y2) / 2;
    html += `\n  <text x="${mx}" y="${my - 8}" text-anchor="middle" class="conn-label" fill="${color}">${label}</text>`;
  }
  return html;
}

function drawPowerBus(x, y, width) {
  return `
  <line x1="${x}" y1="${y}" x2="${x + width}" y2="${y}" stroke="${COLORS.wirePower}" stroke-width="3"/>
  <text x="${x}" y="${y - 6}" class="conn-label" fill="${COLORS.wirePower}">3.3V / 5V</text>
  <line x1="${x}" y1="${y + 14}" x2="${x + width}" y2="${y + 14}" stroke="${COLORS.wireGnd}" stroke-width="3"/>
  <text x="${x}" y="${y + 28}" class="conn-label" fill="${COLORS.wireGnd}">GND</text>`;
}

// ============================================================================
// LEADER WIRING DIAGRAM
// ============================================================================
function generateLeaderDiagram() {
  const W = 920, H = 700;
  let content = '';

  // Background
  content += `<rect width="${W}" height="${H}" fill="${COLORS.bg}"/>`;

  // Title
  content += `
  <text x="${W/2}" y="38" text-anchor="middle" class="title">Leader Wiring Diagram</text>
  <text x="${W/2}" y="58" text-anchor="middle" class="subtitle">Raspberry Pi + Flight Controller + Lora Module</text>`;

  // ===== Raspberry Pi =====
  const piX = 60, piY = 120, piW = 240, piH = 280;
  content += `
  <rect class="board" x="${piX}" y="${piY}" width="${piW}" height="${piH}"
        fill="${COLORS.card}" stroke="${COLORS.accentGreen}" stroke-width="2.5"/>
  <rect x="${piX}" y="${piY}" width="${piW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentGreen}" opacity="0.15"/>
  <text x="${piX + piW/2}" y="${piY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentGreen}">Raspberry Pi</text>
  <text x="${piX + piW/2}" y="${piY + 52}" text-anchor="middle" class="subtitle">Zero 2W / 4B</text>`;

  // Pi GPIO pins (right side)
  const piPins = [
    { label: 'GPIO 14 (TXD)', color: COLORS.wireTx, y: piY + 80 },
    { label: 'GPIO 15 (RXD)', color: COLORS.wireRx, y: piY + 102 },
    { label: 'GPIO 4 (TXD1)', color: COLORS.wireTx, y: piY + 140 },
    { label: 'GPIO 5 (RXD1)', color: COLORS.wireRx, y: piY + 162 },
    { label: '3.3V', color: COLORS.wirePower, y: piY + 200 },
    { label: '5V', color: COLORS.wirePower, y: piY + 222 },
    { label: 'GND', color: COLORS.wireGnd, y: piY + 244 },
  ];

  piPins.forEach(pin => {
    content += `
  <circle cx="${piX + piW}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${piX + piW + 8}" y="${pin.y + 4}" class="mono pin-name" fill="${pin.color}" font-size="12">${pin.label}</text>`;
  });

  // Pi internal labels
  content += `
  <text x="${piX + piW/2}" y="${piY + 300}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">/dev/serial0 → FC</text>
  <text x="${piX + piW/2}" y="${piY + 318}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">/dev/serial1 → Lora</text>
  <text x="${piX + piW/2}" y="${piY + 336}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">WiFi → Web Dashboard</text>`;

  // ===== Flight Controller =====
  const fcX = 580, fcY = 100, fcW = 260, fcH = 200;
  content += `
  <rect class="board" x="${fcX}" y="${fcY}" width="${fcW}" height="${fcH}"
        fill="${COLORS.card}" stroke="${COLORS.accentBlue}" stroke-width="2.5"/>
  <rect x="${fcX}" y="${fcY}" width="${fcW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentBlue}" opacity="0.15"/>
  <text x="${fcX + fcW/2}" y="${fcY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentBlue}">Flight Controller</text>
  <text x="${fcX + fcW/2}" y="${fcY + 52}" text-anchor="middle" class="subtitle">INAV oder ArduPilot</text>`;

  // FC pins (left side)
  const fcPins = [
    { label: 'RX (MAVLink)', color: COLORS.wireRx, y: fcY + 80 },
    { label: 'TX (MAVLink)', color: COLORS.wireTx, y: fcY + 102 },
    { label: '5V IN', color: COLORS.wirePower, y: fcY + 140 },
    { label: 'GND', color: COLORS.wireGnd, y: fcY + 162 },
  ];

  fcPins.forEach(pin => {
    content += `
  <circle cx="${fcX}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${fcX - 8}" y="${pin.y + 4}" text-anchor="end" class="mono pin-name" fill="${pin.color}" font-size="12">${pin.label}</text>`;
  });

  // FC internal
  content += `
  <text x="${fcX + fcW/2}" y="${fcY + 180}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">57600 baud (MAVLink)</text>`;

  // ===== Lora Module =====
  const lrX = 580, lrY = 380, lrW = 260, lrH = 200;
  content += `
  <rect class="board" x="${lrX}" y="${lrY}" width="${lrW}" height="${lrH}"
        fill="${COLORS.card}" stroke="${COLORS.accentOrange}" stroke-width="2.5"/>
  <rect x="${lrX}" y="${lrY}" width="${lrW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentOrange}" opacity="0.15"/>
  <text x="${lrX + lrW/2}" y="${lrY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentOrange}">Lora Module</text>
  <text x="${lrX + lrW/2}" y="${lrY + 52}" text-anchor="middle" class="subtitle">SX1278 / RFM95W / E22-433T30D</text>`;

  // Lora pins (left side)
  const lrPins = [
    { label: 'RX (UART)', color: COLORS.wireRx, y: lrY + 80 },
    { label: 'TX (UART)', color: COLORS.wireTx, y: lrY + 102 },
    { label: '3.3V', color: COLORS.wirePower, y: lrY + 140 },
    { label: 'GND', color: COLORS.wireGnd, y: lrY + 162 },
  ];

  lrPins.forEach(pin => {
    content += `
  <circle cx="${lrX}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${lrX - 8}" y="${pin.y + 4}" text-anchor="end" class="mono pin-name" fill="${pin.color}" font-size="12">${pin.label}</text>`;
  });

  // Lora internal
  content += `
  <text x="${lrX + lrW/2}" y="${lrY + 180}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">9600 baud (transparent)</text>
  <text x="${lrX + lrW/2}" y="${lrY + 196}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="11">433.125 MHz / SF7 / 20dBm</text>`;

  // ===== Antenna =====
  content += `
  <text x="${lrX + lrW - 40}" y="${lrY + 75}" text-anchor="middle" style="font-size:28px">📡</text>
  <text x="${lrX + lrW - 40}" y="${lrY + 95}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="9">433MHz</text>`;

  // ===== WIRES =====

  // Pi → FC (MAVLink)
  content += drawWire(piX + piW + 130, piY + 80, fcX, fcY + 80, COLORS.wireTx, 'TX → FC RX', true);
  content += drawWire(piX + piW + 130, piY + 102, fcX, fcY + 102, COLORS.wireRx, 'RX ← FC TX', true);

  // Pi → Lora
  content += drawWire(piX + piW + 130, piY + 140, lrX, lrY + 80, COLORS.wireTx, 'TX → Lora RX', true);
  content += drawWire(piX + piW + 130, piY + 162, lrX, lrY + 102, COLORS.wireRx, 'RX ← Lora TX', true);

  // Power bus
  content += `
  <line x1="${piX + piW}" y1="${piY + 200}" x2="${fcX - 8}" y2="${fcY + 140}"
        stroke="${COLORS.wirePower}" stroke-width="2" stroke-dasharray="6 3"/>
  <text x="${(piX + piW + fcX)/2}" y="${(piY + 200 + fcY + 140)/2 - 8}" text-anchor="middle"
        class="conn-label" fill="${COLORS.wirePower}">5V</text>`;

  content += `
  <line x1="${piX + piW}" y1="${piY + 222}" x2="${lrX - 8}" y2="${lrY + 140}"
        stroke="${COLORS.wirePower}" stroke-width="2" stroke-dasharray="6 3"/>
  <text x="${(piX + piW + lrX)/2 - 20}" y="${(piY + 222 + lrY + 140)/2 - 8}" text-anchor="middle"
        class="conn-label" fill="${COLORS.wirePower}">3.3V</text>`;

  // GND
  content += `
  <line x1="${piX + piW}" y1="${piY + 244}" x2="${fcX - 8}" y2="${fcY + 162}"
        stroke="${COLORS.wireGnd}" stroke-width="2" stroke-dasharray="4 3"/>
  <line x1="${piX + piW}" y1="${piY + 244}" x2="${lrX - 8}" y2="${lrY + 162}"
        stroke="${COLORS.wireGnd}" stroke-width="2" stroke-dasharray="4 3"/>`;

  // ===== Legend =====
  const legY = H - 80;
  content += `
  <rect x="40" y="${legY}" width="${W - 80}" height="55" rx="6" fill="${COLORS.card}" stroke="${COLORS.cardStroke}" stroke-width="1"/>
  <text x="60" y="${legY + 20}" class="mono" fill="${COLORS.wireTx}" font-size="12">━━ TX (Daten senden)</text>
  <text x="260" y="${legY + 20}" class="mono" fill="${COLORS.wireRx}" font-size="12">━━ RX (Daten empfangen)</text>
  <text x="480" y="${legY + 20}" class="mono" fill="${COLORS.wirePower}" font-size="12">╌╌ Stromversorgung</text>
  <text x="660" y="${legY + 20}" class="mono" fill="${COLORS.wireGnd}" font-size="12">╌╌ GND</text>
  <text x="60" y="${legY + 42}" class="note">⚠️ Pi und FC auf gemeinsame GND verbinden! FC-Versorgung unabhängig vom Pi!</text>`;

  return makeSvg(content, W, H);
}

// ============================================================================
// FOLLOWER WIRING DIAGRAM
// ============================================================================
function generateFollowerDiagram() {
  const W = 920, H = 780;
  let content = '';

  content += `<rect width="${W}" height="${H}" fill="${COLORS.bg}"/>`;

  // Title
  content += `
  <text x="${W/2}" y="38" text-anchor="middle" class="title">Follower Wiring Diagram</text>
  <text x="${W/2}" y="58" text-anchor="middle" class="subtitle">ESP32 + Flight Controller + Lora Module + Status LEDs</text>`;

  // ===== ESP32 =====
  const espX = 60, espY = 100, espW = 260, espH = 420;
  content += `
  <rect class="board" x="${espX}" y="${espY}" width="${espW}" height="${espH}"
        fill="${COLORS.card}" stroke="${COLORS.accentGreen}" stroke-width="2.5"/>
  <rect x="${espX}" y="${espY}" width="${espW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentGreen}" opacity="0.15"/>
  <text x="${espX + espW/2}" y="${espY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentGreen}">ESP32-WROOM-32</text>
  <text x="${espX + espW/2}" y="${espY + 52}" text-anchor="middle" class="subtitle">DevKit / TTGO Lora32</text>`;

  // ESP32 pins
  const espPins = [
    { label: 'GPIO 17 (TX1)', color: COLORS.wireTx, y: espY + 80 },
    { label: 'GPIO 16 (RX1)', color: COLORS.wireRx, y: espY + 102 },
    { label: 'GPIO 18 (SCK)', color: COLORS.wireSpi, y: espY + 134 },
    { label: 'GPIO 19 (MISO)', color: COLORS.wireSpi, y: espY + 156 },
    { label: 'GPIO 23 (MOSI)', color: COLORS.wireSpi, y: espY + 178 },
    { label: 'GPIO 5  (NSS)', color: COLORS.wireSpi, y: espY + 200 },
    { label: 'GPIO 14 (RST)', color: COLORS.wireSpi, y: espY + 222 },
    { label: 'GPIO 2  (DIO0)', color: COLORS.wireGpio, y: espY + 244 },
    { label: 'GPIO 25 (LINK)', color: COLORS.accentGreen, y: espY + 276 },
    { label: 'GPIO 26 (FC)', color: COLORS.accentBlue, y: espY + 298 },
    { label: 'GPIO 27 (GPS)', color: COLORS.accentPurple, y: espY + 320 },
    { label: 'GPIO 32 (ERR)', color: COLORS.accent, y: espY + 342 },
    { label: 'GPIO 33 (BTN)', color: COLORS.accentOrange, y: espY + 374 },
    { label: '3.3V', color: COLORS.wirePower, y: espY + 406 },
  ];

  espPins.forEach(pin => {
    content += `
  <circle cx="${espX + espW}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${espX + espW + 8}" y="${pin.y + 4}" class="mono pin-name" fill="${pin.color}" font-size="11.5">${pin.label}</text>`;
  });

  // ESP32 internal
  content += `
  <text x="${espX + espW/2}" y="${espY + 440}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="10">Serial0: USB Debug (115200)</text>
  <text x="${espX + espW/2}" y="${espY + 455}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="10">Serial1: FC (57600/115200)</text>`;

  // ===== Flight Controller =====
  const fcX = 580, fcY = 80, fcW = 260, fcH = 180;
  content += `
  <rect class="board" x="${fcX}" y="${fcY}" width="${fcW}" height="${fcH}"
        fill="${COLORS.card}" stroke="${COLORS.accentBlue}" stroke-width="2.5"/>
  <rect x="${fcX}" y="${fcY}" width="${fcW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentBlue}" opacity="0.15"/>
  <text x="${fcX + fcW/2}" y="${fcY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentBlue}">Flight Controller</text>
  <text x="${fcX + fcW/2}" y="${fcY + 52}" text-anchor="middle" class="subtitle">INAV oder ArduPilot</text>`;

  const fcPins = [
    { label: 'RX', color: COLORS.wireRx, y: fcY + 80 },
    { label: 'TX', color: COLORS.wireTx, y: fcY + 102 },
    { label: '5V IN', color: COLORS.wirePower, y: fcY + 136 },
    { label: 'GND', color: COLORS.wireGnd, y: fcY + 158 },
  ];

  fcPins.forEach(pin => {
    content += `
  <circle cx="${fcX}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${fcX - 8}" y="${pin.y + 4}" text-anchor="end" class="mono pin-name" fill="${pin.color}" font-size="12">${pin.label}</text>`;
  });

  // ===== Lora Module =====
  const lrX = 580, lrY = 320, lrW = 260, lrH = 200;
  content += `
  <rect class="board" x="${lrX}" y="${lrY}" width="${lrW}" height="${lrH}"
        fill="${COLORS.card}" stroke="${COLORS.accentOrange}" stroke-width="2.5"/>
  <rect x="${lrX}" y="${lrY}" width="${lrW}" height="36" rx="8" ry="8"
        fill="${COLORS.accentOrange}" opacity="0.15"/>
  <text x="${lrX + lrW/2}" y="${lrY + 24}" text-anchor="middle"
        style="font-size:15px;font-weight:700;fill:${COLORS.accentOrange}">Lora Module</text>
  <text x="${lrX + lrW/2}" y="${lrY + 52}" text-anchor="middle" class="subtitle">SX1278 / RFM95W (SPI)</text>`;

  const lrPins = [
    { label: 'SCK', color: COLORS.wireSpi, y: lrY + 80 },
    { label: 'MISO', color: COLORS.wireSpi, y: lrY + 102 },
    { label: 'MOSI', color: COLORS.wireSpi, y: lrY + 124 },
    { label: 'NSS (CS)', color: COLORS.wireSpi, y: lrY + 146 },
    { label: 'RST', color: COLORS.wireSpi, y: lrY + 168 },
  ];

  lrPins.forEach(pin => {
    content += `
  <circle cx="${lrX}" cy="${pin.y}" r="3.5" fill="${pin.color}"/>
  <text x="${lrX - 8}" y="${pin.y + 4}" text-anchor="end" class="mono pin-name" fill="${pin.color}" font-size="12">${pin.label}</text>`;
  });

  // Lora antenna
  content += `
  <text x="${lrX + lrW - 40}" y="${lrY + 75}" text-anchor="middle" style="font-size:28px">📡</text>
  <text x="${lrX + lrW - 40}" y="${lrY + 95}" text-anchor="middle" class="mono" fill="${COLORS.textDim}" font-size="9">433MHz</text>`;

  // ===== LEDs =====
  const ledX = 580, ledY = 570;
  content += `
  <rect x="${ledX}" y="${ledY}" width="260" height="110" rx="6" fill="${COLORS.card}" stroke="${COLORS.accentPurple}" stroke-width="1.5"/>
  <text x="${ledX + 130}" y="${ledY + 20}" text-anchor="middle" style="font-size:13px;font-weight:600;fill:${COLORS.accentPurple}">Status LEDs</text>`;

  const leds = [
    { label: '🟢 LINK (GPIO 25)', color: COLORS.accentGreen, y: ledY + 44 },
    { label: '🔵 FC (GPIO 26)', color: COLORS.accentBlue, y: ledY + 62 },
    { label: '⚪ GPS (GPIO 27)', color: COLORS.accentPurple, y: ledY + 80 },
    { label: '🔴 ERR (GPIO 32)', color: COLORS.accent, y: ledY + 98 },
  ];

  leds.forEach(led => {
    content += `
  <text x="${ledX + 12}" y="${led.y}" class="mono" fill="${led.color}" font-size="11">${led.label}</text>`;
  });

  // Resistor note
  content += `
  <text x="${ledX + 130}" y="${ledY + 108}" text-anchor="middle" class="note" font-size="10">je 220 Ohm Vorwiderstand!</text>`;

  // ===== Config Button =====
  const btnX = 580, btnY = 700;
  content += `
  <rect x="${btnX}" y="${btnY}" width="260" height="40" rx="6" fill="${COLORS.card}" stroke="${COLORS.accentOrange}" stroke-width="1.5"/>
  <text x="${btnX + 130}" y="${btnY + 25}" text-anchor="middle" class="mono" fill="${COLORS.accentOrange}" font-size="12">⏺ Config Button (GPIO 33 → GND)</text>`;

  // ===== WIRES =====

  // ESP32 → FC (UART)
  content += drawWire(espX + espW + 130, espY + 80, fcX, fcY + 80, COLORS.wireTx, 'TX1 → FC RX', true);
  content += drawWire(espX + espW + 130, espY + 102, fcX, fcY + 102, COLORS.wireRx, 'RX1 ← FC TX', true);

  // ESP32 → Lora (SPI)
  content += drawWire(espX + espW + 130, espY + 134, lrX, lrY + 80, COLORS.wireSpi, 'SCK', true);
  content += drawWire(espX + espW + 130, espY + 156, lrX, lrY + 102, COLORS.wireSpi, 'MISO', true);
  content += drawWire(espX + espW + 130, espY + 178, lrX, lrY + 124, COLORS.wireSpi, 'MOSI', true);
  content += drawWire(espX + espW + 130, espY + 200, lrX, lrY + 146, COLORS.wireSpi, 'NSS', true);
  content += drawWire(espX + espW + 130, espY + 222, lrX, lrY + 168, COLORS.wireSpi, 'RST', true);

  // ESP32 → DIO0 (GPIO interrupt)
  content += `
  <line x1="${espX + espW + 130}" y1="${espY + 244}" x2="${lrX}" y2="${lrY + 190}"
        stroke="${COLORS.wireGpio}" stroke-width="2.5" stroke-dasharray="6 3"/>
  <text x="${(espX + espW + 130 + lrX)/2}" y="${(espY + 244 + lrY + 190)/2 - 8}" text-anchor="middle"
        class="conn-label" fill="${COLORS.wireGpio}">DIO0 (IRQ)</text>`;

  // ESP32 → LEDs
  content += `
  <line x1="${espX + espW + 130}" y1="${espY + 276}" x2="${ledX}" y2="${ledY + 44}"
        stroke="${COLORS.accentGreen}" stroke-width="2" stroke-dasharray="6 3"/>
  <line x1="${espX + espW + 130}" y1="${espY + 298}" x2="${ledX}" y2="${ledY + 62}"
        stroke="${COLORS.accentBlue}" stroke-width="2" stroke-dasharray="6 3"/>
  <line x1="${espX + espW + 130}" y1="${espY + 320}" x2="${ledX}" y2="${ledY + 80}"
        stroke="${COLORS.accentPurple}" stroke-width="2" stroke-dasharray="6 3"/>
  <line x1="${espX + espW + 130}" y1="${espY + 342}" x2="${ledX}" y2="${ledY + 98}"
        stroke="${COLORS.accent}" stroke-width="2" stroke-dasharray="6 3"/>`;

  // ESP32 → Button
  content += `
  <line x1="${espX + espW + 130}" y1="${espY + 374}" x2="${btnX}" y2="${btnY + 20}"
        stroke="${COLORS.accentOrange}" stroke-width="2" stroke-dasharray="6 3"/>`;

  // Power
  content += `
  <line x1="${espX + espW + 130}" y1="${espY + 406}" x2="${fcX - 8}" y2="${fcY + 136}"
        stroke="${COLORS.wirePower}" stroke-width="2" stroke-dasharray="6 3"/>
  <text x="${(espX + espW + 130 + fcX)/2}" y="${(espY + 406 + fcY + 136)/2 - 8}" text-anchor="middle"
        class="conn-label" fill="${COLORS.wirePower}">5V (USB)</text>`;

  // GND
  content += `
  <line x1="${espX + espW + 130}" y1="${espY + 410}" x2="${fcX - 8}" y2="${fcY + 158}"
        stroke="${COLORS.wireGnd}" stroke-width="2" stroke-dasharray="4 3"/>`;

  // ===== Legend =====
  const legY = H - 60;
  content += `
  <rect x="40" y="${legY}" width="${W - 80}" height="45" rx="6" fill="${COLORS.card}" stroke="${COLORS.cardStroke}" stroke-width="1"/>
  <text x="60" y="${legY + 18}" class="mono" fill="${COLORS.wireTx}" font-size="11">━━ TX</text>
  <text x="140" y="${legY + 18}" class="mono" fill="${COLORS.wireRx}" font-size="11">━━ RX</text>
  <text x="220" y="${legY + 18}" class="mono" fill="${COLORS.wireSpi}" font-size="11">━━ SPI</text>
  <text x="300" y="${legY + 18}" class="mono" fill="${COLORS.wireGpio}" font-size="11">━━ GPIO</text>
  <text x="400" y="${legY + 18}" class="mono" fill="${COLORS.wirePower}" font-size="11">╌╌ Power</text>
  <text x="500" y="${legY + 18}" class="mono" fill="${COLORS.wireGnd}" font-size="11">╌╌ GND</text>
  <text x="60" y="${legY + 38}" class="note">⚠️ LEDs mit 220 Ohm Vorwiderstand! Button mit internem Pullup. Alle GND verbinden!</text>`;

  return makeSvg(content, W, H);
}

// ============================================================================
// Generate both diagrams
// ============================================================================
if (!fs.existsSync(outputDir)) {
  fs.mkdirSync(outputDir, { recursive: true });
}

fs.writeFileSync(path.join(outputDir, 'wiring_leader.svg'), generateLeaderDiagram());
console.log('✅ Leader wiring diagram saved: docs/wiring_leader.svg');

fs.writeFileSync(path.join(outputDir, 'wiring_follower.svg'), generateFollowerDiagram());
console.log('✅ Follower wiring diagram saved: docs/wiring_follower.svg');
