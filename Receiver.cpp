#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#include <stdarg.h>   // PATCH

#ifndef MAX_NODES
  #define MAX_NODES 5
#endif

static const char* AP_SSID = "LEECH_RECEIVER";
static const char* AP_PASS = "12345678";

static const char* SERVICE_UUID = "12345678-1234-1234-1234-1234567890ab";
static const char* ANGLES_UUID  = "12345678-1234-1234-1234-1234567890ac";

static const uint8_t PACKET_VERSION = 1;

#pragma pack(push, 1)
struct AnglesPacket {
  uint8_t  version;
  uint8_t  node_id;
  uint32_t t_ms;
  float    roll_deg;
  float    pitch_deg;
  uint16_t crc16;
};
#pragma pack(pop)

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static float roll_latest[MAX_NODES + 1];
static float pitch_latest[MAX_NODES + 1];
static uint32_t last_update_ms[MAX_NODES + 1];

static WebServer server(80);

// PATCH: tiny logger helpers so prints are flushed (helps on some USB CDC setups)
static void logln(const char* s) {
  Serial.println(s);
  Serial.flush();
}
static void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  Serial.flush();
}

static const char HTML_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Leech Receiver</title>
  <style>
    :root {
      --bg: #0b0f14;
      --card: #111824;
      --muted: #93a4b8;
      --text: #e6eef8;
      --border: rgba(255,255,255,0.08);
      --ok: #2dd4bf;
      --bad: #fb7185;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
      background: radial-gradient(1200px 800px at 20% 0%, #172033 0%, var(--bg) 55%);
      color: var(--text);
      padding: 18px;
    }
    h1 {
      margin: 0 0 6px 0;
      font-size: 18px;
      letter-spacing: 0.2px;
    }
    p {
      margin: 0 0 16px 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.4;
    }
    .grid {
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 14px;
      max-width: 1100px;
    }
    @media (max-width: 900px) {
      .grid { grid-template-columns: 1fr; }
    }
    .card {
      background: rgba(17,24,36,0.85);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 14px;
      box-shadow: 0 10px 30px rgba(0,0,0,0.35);
      backdrop-filter: blur(6px);
    }
    .row {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 10px;
      margin-bottom: 10px;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border: 1px solid var(--border);
      border-radius: 999px;
      padding: 6px 10px;
      font-size: 12px;
      color: var(--muted);
    }
    .dot {
      width: 9px;
      height: 9px;
      border-radius: 999px;
      background: var(--bad);
      box-shadow: 0 0 0 3px rgba(251,113,133,0.18);
    }
    .dot.ok {
      background: var(--ok);
      box-shadow: 0 0 0 3px rgba(45,212,191,0.18);
    }

    .big {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 10px;
      margin-top: 10px;
    }
    .metric {
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 10px;
      background: rgba(255,255,255,0.03);
    }
    .metric .k { font-size: 11px; color: var(--muted); margin-bottom: 6px; }
    .metric .v { font-size: 22px; font-weight: 650; letter-spacing: 0.2px; }
    .metric .u { font-size: 11px; color: var(--muted); margin-left: 6px; }

    .tiltWrap {
      display: grid;
      grid-template-columns: 1fr;
      gap: 12px;
    }
    .tiltBox {
      position: relative;
      height: 280px;
      border-radius: 14px;
      border: 1px solid var(--border);
      background:
        linear-gradient(to right, rgba(255,255,255,0.06) 1px, transparent 1px) 0 0 / 40px 40px,
        linear-gradient(to bottom, rgba(255,255,255,0.06) 1px, transparent 1px) 0 0 / 40px 40px,
        radial-gradient(600px 380px at 50% 40%, rgba(45,212,191,0.10), transparent 55%),
        rgba(255,255,255,0.02);
      overflow: hidden;
    }
    .axesLabel {
      position: absolute;
      font-size: 11px;
      color: var(--muted);
      opacity: 0.85;
      user-select: none;
      pointer-events: none;
    }
    .axesLabel.top { top: 10px; left: 12px; }
    .axesLabel.bottom { bottom: 10px; left: 12px; }
    .axesLabel.right { top: 10px; right: 12px; text-align: right; }

    .crossH, .crossV {
      position: absolute;
      background: rgba(255,255,255,0.10);
    }
    .crossH { left: 0; right: 0; top: 50%; height: 1px; }
    .crossV { top: 0; bottom: 0; left: 50%; width: 1px; }

    .point {
      position: absolute;
      width: 14px;
      height: 14px;
      border-radius: 999px;
      transform: translate(-50%, -50%);
      background: var(--ok);
      box-shadow: 0 10px 22px rgba(0,0,0,0.45), 0 0 0 4px rgba(45,212,191,0.20);
      transition: left 120ms linear, top 120ms linear;
    }

    canvas {
      width: 100%;
      height: 190px;
      border-radius: 14px;
      border: 1px solid var(--border);
      background: rgba(255,255,255,0.02);
      display: block;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      border-bottom: 1px solid var(--border);
      padding: 10px 8px;
      text-align: left;
    }
    th { color: var(--muted); font-weight: 600; }
    .statusOk { color: var(--ok); font-weight: 700; }
    .statusBad { color: var(--bad); font-weight: 700; }
  </style>
</head>
<body>
  <h1>Leech Receiver</h1>
  <p>Connect to the receiver WiFi and open this page. Live data comes from <code>/api</code>.</p>

  <div class="grid">
    <div class="card">
      <div class="row">
        <div class="pill">
          <span id="connDot" class="dot"></span>
          <span id="connText">NO DATA</span>
        </div>
        <div class="pill">
          Node <span id="nodeNum">1</span>
          <span style="opacity:0.5">·</span>
          age <span id="ageMs">0</span> ms
        </div>
      </div>

      <div class="big">
        <div class="metric">
          <div class="k">Roll</div>
          <div class="v"><span id="rollVal">0.00</span><span class="u">deg</span></div>
        </div>
        <div class="metric">
          <div class="k">Pitch</div>
          <div class="v"><span id="pitchVal">0.00</span><span class="u">deg</span></div>
        </div>
        <div class="metric">
          <div class="k">Updates</div>
          <div class="v"><span id="hzVal">0</span><span class="u">Hz</span></div>
        </div>
      </div>

      <div class="tiltWrap" style="margin-top: 12px;">
        <div class="tiltBox" id="tiltBox">
          <div class="crossH"></div>
          <div class="crossV"></div>
          <div class="axesLabel top">Pitch +</div>
          <div class="axesLabel bottom">Pitch -</div>
          <div class="axesLabel right">Roll + →</div>
          <div class="point" id="point" style="left:50%; top:50%;"></div>
        </div>
        <canvas id="chart"></canvas>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <div class="pill">All nodes</div>
        <div class="pill">poll 250 ms</div>
      </div>
      <table>
        <thead>
          <tr><th>Node</th><th>Status</th><th>Age (ms)</th><th>Roll</th><th>Pitch</th></tr>
        </thead>
        <tbody id="rows"></tbody>
      </table>
    </div>
  </div>

<script>
const MAX_NODES = 5;

const rowsEl = document.getElementById('rows');
const rollEl = document.getElementById('rollVal');
const pitchEl = document.getElementById('pitchVal');
const ageEl = document.getElementById('ageMs');
const connDot = document.getElementById('connDot');
const connText = document.getElementById('connText');
const hzEl = document.getElementById('hzVal');

const tiltBox = document.getElementById('tiltBox');
const point = document.getElementById('point');

const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.floor(rect.width * dpr);
  canvas.height = Math.floor(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

const CAP = 48;
const rollBuf = [];
const pitchBuf = [];
const tBuf = [];

let updateCount = 0;
let lastHzTick = performance.now();

function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }

const RANGE_DEG = 45;

function setPoint(roll, pitch) {
  const rect = tiltBox.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height;

  const xNorm = clamp(roll / RANGE_DEG, -1, 1);
  const yNorm = clamp(pitch / RANGE_DEG, -1, 1);

  const x = (w * 0.5) + (xNorm * w * 0.45);
  const y = (h * 0.5) - (yNorm * h * 0.45);

  point.style.left = x + "px";
  point.style.top = y + "px";
}

function drawChart() {
  const w = canvas.getBoundingClientRect().width;
  const h = canvas.getBoundingClientRect().height;

  ctx.clearRect(0, 0, w, h);

  ctx.globalAlpha = 0.35;
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = (h * i) / 4;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  for (let i = 0; i <= 6; i++) {
    const x = (w * i) / 6;
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }

  const ymin = -RANGE_DEG;
  const ymax = RANGE_DEG;

  function xAt(i, n) {
    if (n <= 1) return 0;
    return (w * i) / (n - 1);
  }
  function yAt(v) {
    const t = (v - ymin) / (ymax - ymin);
    return h - (t * h);
  }

  ctx.globalAlpha = 1.0;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < rollBuf.length; i++) {
    const x = xAt(i, rollBuf.length);
    const y = yAt(rollBuf[i]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  for (let i = 0; i < pitchBuf.length; i++) {
    const x = xAt(i, pitchBuf.length);
    const y = yAt(pitchBuf[i]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.globalAlpha = 0.9;
  ctx.font = "12px -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Arial";
  ctx.fillText("Roll (solid) / Pitch (dashed)", 10, 18);
  ctx.globalAlpha = 0.75;
  ctx.fillText("Scale: +- " + RANGE_DEG + " deg", 10, 36);
}

function renderTable(nodes) {
  rowsEl.innerHTML = '';
  for (const n of nodes) {
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td>${n.id}</td>` +
      `<td class="${n.connected ? 'statusOk' : 'statusBad'}">${n.connected ? 'CONNECTED' : 'NO DATA'}</td>` +
      `<td>${n.age_ms}</td>` +
      `<td>${Number(n.roll).toFixed(2)}</td>` +
      `<td>${Number(n.pitch).toFixed(2)}</td>`;
    rowsEl.appendChild(tr);
  }
}

async function tick() {
  try {
    const r = await fetch('/api', { cache: 'no-store' });
    const j = await r.json();

    renderTable(j.nodes);

    const n1 = j.nodes.find(x => x.id === 1) || j.nodes[0];
    const roll = Number(n1.roll) || 0;
    const pitch = Number(n1.pitch) || 0;
    const age = Number(n1.age_ms) || 0;
    const connected = !!n1.connected;

    rollEl.textContent = roll.toFixed(2);
    pitchEl.textContent = pitch.toFixed(2);
    ageEl.textContent = age.toFixed(0);

    if (connected) {
      connDot.classList.add('ok');
      connText.textContent = "CONNECTED";
    } else {
      connDot.classList.remove('ok');
      connText.textContent = "NO DATA";
    }

    if (connected) {
      setPoint(roll, pitch);

      updateCount++;
      const now = performance.now();
      if (now - lastHzTick > 1000) {
        hzEl.textContent = Math.round(updateCount * 1000 / (now - lastHzTick));
        updateCount = 0;
        lastHzTick = now;
      }

      rollBuf.push(roll);
      pitchBuf.push(pitch);
      tBuf.push(j.ms);

      if (rollBuf.length > CAP) {
        rollBuf.shift();
        pitchBuf.shift();
        tBuf.shift();
      }

      drawChart();
    }
  } catch (e) {
    connDot.classList.remove('ok');
    connText.textContent = "NO DATA";
  }
}

setInterval(tick, 250);
tick();
</script>
</body>
</html>
)HTML";

static void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

static void handleApi() {
  uint32_t now = millis();
  String out;
  out.reserve(900);

  out += "{\"ms\":";
  out += String(now);
  out += ",\"nodes\":[";

  portENTER_CRITICAL(&mux);
  for (int id = 1; id <= MAX_NODES; id++) {
    uint32_t age = (last_update_ms[id] == 0) ? 0 : (now - last_update_ms[id]);
    bool connected = (last_update_ms[id] != 0) && (age < 5000);

    out += "{\"id\":";
    out += String(id);
    out += ",\"connected\":";
    out += (connected ? "true" : "false");
    out += ",\"age_ms\":";
    out += String(age);
    out += ",\"roll\":";
    out += String(roll_latest[id], 4);
    out += ",\"pitch\":";
    out += String(pitch_latest[id], 4);
    out += "}";

    if (id != MAX_NODES) out += ",";
  }
  portEXIT_CRITICAL(&mux);

  out += "]}";
  server.send(200, "application/json", out);
}

static NimBLEAdvertisedDevice* foundDev = nullptr;
static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic* remoteChar = nullptr;

static void notifyCB(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;

  if (len != sizeof(AnglesPacket)) return;

  AnglesPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.version != PACKET_VERSION) return;
  if (pkt.node_id < 1 || pkt.node_id > MAX_NODES) return;

  uint16_t want = crc16_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 2);
  if (want != pkt.crc16) return;

  portENTER_CRITICAL(&mux);
  roll_latest[pkt.node_id] = pkt.roll_deg;
  pitch_latest[pkt.node_id] = pkt.pitch_deg;
  last_update_ms[pkt.node_id] = millis();
  portEXIT_CRITICAL(&mux);
}

// PATCH: make this static and reused (no repeated new allocations)
class AdvCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    if (foundDev != nullptr) return;

    bool match = false;
    if (dev->haveServiceUUID() && dev->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) match = true;
    if (dev->haveName()) {
      std::string name = dev->getName();
      if (name.rfind("LEECH_NODE_", 0) == 0) match = true;
    }

    if (!match) return;

    foundDev = new NimBLEAdvertisedDevice(*dev);
    logf("Found node: %s\n", dev->haveName() ? dev->getName().c_str() : dev->getAddress().toString().c_str());
  }
};

static AdvCallbacks advCB;  // PATCH

static bool connectAndSubscribe() {
  if (foundDev == nullptr) return false;

  if (client != nullptr) {
    if (client->isConnected()) client->disconnect();
    NimBLEDevice::deleteClient(client);
    client = nullptr;
  }

  client = NimBLEDevice::createClient();

  logf("Connecting to %s\n", foundDev->getAddress().toString().c_str());

  bool ok = client->connect(foundDev);

  // PATCH: free foundDev no matter what (avoid leak)
  delete foundDev;
  foundDev = nullptr;

  if (!ok) {
    logln("Connect failed");
    return false;
  }

  NimBLERemoteService* svc = client->getService(SERVICE_UUID);
  if (!svc) {
    logln("Service not found");
    client->disconnect();
    return false;
  }

  remoteChar = svc->getCharacteristic(ANGLES_UUID);
  if (!remoteChar) {
    logln("Characteristic not found");
    client->disconnect();
    return false;
  }

  if (!remoteChar->canNotify()) {
    logln("Characteristic cannot notify");
    client->disconnect();
    return false;
  }

  if (!remoteChar->subscribe(true, notifyCB)) {
    logln("Subscribe failed");
    client->disconnect();
    return false;
  }

  logln("Subscribed OK");
  return true;
}

static void bleLoopOnce() {
  if (client && client->isConnected()) return;

  // PATCH: ensure no leftover allocation
  if (foundDev) {
    delete foundDev;
    foundDev = nullptr;
  }

  NimBLEScan* scan = NimBLEDevice::getScan();

  logln("Scanning BLE");
  scan->start(2, false);
  scan->clearResults();

  if (foundDev == nullptr) {
    logln("No node found");
    return;
  }

  bool ok = connectAndSubscribe();
  if (!ok) {
    logln("Connect and subscribe failed");
  }
}

void setup() {
  Serial.begin(115200);

  // PATCH: give USB CDC a moment to attach, but do not block forever
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 1500)) { delay(10); }

  logln("");
  logln("BOOT OK");

  for (int i = 0; i <= MAX_NODES; i++) {
    roll_latest[i] = 0.0f;
    pitch_latest[i] = 0.0f;
    last_update_ms[i] = 0;
  }

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  logf("WiFi AP started: %s\n", ok ? "OK" : "FAIL");
  logf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api", HTTP_GET, handleApi);
  server.begin();
  logln("Web server started");

  NimBLEDevice::init("LEECH_RECEIVER");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  // PATCH: configure scan once, reuse callbacks
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&advCB, true);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);

  logln("BLE init done");
}

void loop() {
  server.handleClient();

  static uint32_t lastBleTry = 0;
  if (millis() - lastBleTry > 1500) {
    bleLoopOnce();
    lastBleTry = millis();
  }

  // PATCH: heartbeat so you always know Serial is alive (even if you missed BOOT OK)
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 1000) {
    logf("alive ms=%lu free=%lu\n", (unsigned long)millis(), (unsigned long)ESP.getFreeHeap());
    lastBeat = millis();
  }

  delay(2);
}
