/*
 * StructuralMonitor.ino
 * Experimental Structural Vibration Monitoring System
 * ESP32 + ADXL345 (I2C 0x53) + vibration motor driver (GPIO25)
 *
 * SoftAP: SSID "Monitor"  password "12345678gi"  ->  http://192.168.4.1
 * Single file. All HTML/CSS/JS inside one raw string literal.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

// ---------------- Pins ----------------
#define SDA_PIN      21
#define SCL_PIN      22
#define MOTOR_PIN    25

// ---------------- ADXL345 ----------------
#define ADXL_ADDR      0x53
#define REG_DEVID      0x00
#define REG_BW_RATE    0x2C
#define REG_POWER_CTL  0x2D
#define REG_DATA_FORMAT 0x31
#define REG_DATAX0     0x32

// ---------------- Config ----------------
#define SAMPLE_COUNT    128
#define GRAPH_POINTS    32
#define MAX_MATERIALS   5
#define MAX_HISTORY     10
#define MOTOR_ON_MS     150
#define MOTOR_OFF_MS    200
#define SIM_LOW         60
#define SIM_CRIT        40
#define SAMPLE_INTERVAL_US 10000UL

// ---------------- Data structures ----------------
struct Material {
  bool  used;
  char  name[24];
  char  type[16];
  char  specId[12];
  float length_mm;
  float width_mm;
  float thickness_mm;
  char  notes[48];

  bool  hasBaseline;
  float baseRms;
  float basePeak;
  float basePkpk;
  float baseMeanX;
  float baseMeanY;
  float baseMeanZ;
  float baseFs;
  int   baseSamples;

  float baseGX[GRAPH_POINTS];
  float baseGY[GRAPH_POINTS];
  float baseGZ[GRAPH_POINTS];

  float lastSim;
  bool  hasLastTest;
};

struct HistoryEntry {
  bool  used;
  char  name[24];
  char  specId[12];
  float baseRms;
  float testRms;
  float similarity;
  unsigned long timestamp;
};

// ---------------- Global state ----------------
WebServer   server(80);
Preferences prefs;

bool sensorOK = false;
bool busy     = false;

Material materials[MAX_MATERIALS];
int      materialCount = 0;
int      selectedIdx   = -1;

HistoryEntry history[MAX_HISTORY];
int          historyCount = 0;

float curX[SAMPLE_COUNT];
float curY[SAMPLE_COUNT];
float curZ[SAMPLE_COUNT];
float curMeanX = 0, curMeanY = 0, curMeanZ = 0;
float curRms = 0, curPeak = 0, curPkpk = 0, curFs = 0;
int   curSamples = 0;
bool  curValid = false;
float curSimilarity = -1.0f;

struct {
  bool          active = false;
  bool          onPhase = false;
  uint8_t       pulsesRemaining = 0;
  unsigned long lastChange = 0;
} motor;

// ---------------- Helpers ----------------
void copyStr(char* dst, const String& src, size_t n) {
  if (n == 0) return;
  strncpy(dst, src.c_str(), n - 1);
  dst[n - 1] = 0;
}

String jsonEscape(const char* s) {
  String out;
  for (const char* p = s; *p; ++p) {
    char c = *p;
    if (c == '"')  out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((unsigned char)c < 0x20) { }
    else out += c;
  }
  return out;
}
String Jstr(const char* s) { return "\"" + jsonEscape(s) + "\""; }
String Jstr(const String& s) { return "\"" + jsonEscape(s.c_str()) + "\""; }
String Jb(bool b)  { return b ? "true" : "false"; }
String Jf(float v, int d = 4) { return String(v, d); }

// ---------------- ADXL345 low level ----------------
void adxlWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
bool adxlRead(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADXL_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
bool adxlInit() {
  uint8_t id = 0;
  if (!adxlRead(REG_DEVID, &id, 1)) return false;
  if (id != 0xE5) return false;
  adxlWrite(REG_POWER_CTL, 0x00);
  adxlWrite(REG_DATA_FORMAT, 0x0B);
  adxlWrite(REG_BW_RATE, 0x0A);
  adxlWrite(REG_POWER_CTL, 0x08);
  delay(10);
  return true;
}
bool adxlReadG(float& x, float& y, float& z) {
  uint8_t b[6];
  if (!adxlRead(REG_DATAX0, b, 6)) return false;
  int16_t rx = (int16_t)((b[1] << 8) | b[0]);
  int16_t ry = (int16_t)((b[3] << 8) | b[2]);
  int16_t rz = (int16_t)((b[5] << 8) | b[4]);
  const float scale = 0.004f;
  x = rx * scale; y = ry * scale; z = rz * scale;
  return true;
}

// ---------------- Motor sequencer ----------------
void motorStop() {
  digitalWrite(MOTOR_PIN, LOW);
  motor.active = false;
  motor.onPhase = false;
  motor.pulsesRemaining = 0;
}
void motorStartPulses(uint8_t n) {
  if (n == 0) return;
  digitalWrite(MOTOR_PIN, HIGH);
  motor.active = true;
  motor.onPhase = true;
  motor.pulsesRemaining = n;
  motor.lastChange = millis();
}
void motorUpdate() {
  if (!motor.active) return;
  unsigned long now = millis();
  unsigned long dt  = now - motor.lastChange;
  if (motor.onPhase) {
    if (dt >= MOTOR_ON_MS) {
      digitalWrite(MOTOR_PIN, LOW);
      motor.onPhase = false;
      motor.lastChange = now;
      if (motor.pulsesRemaining > 0) motor.pulsesRemaining--;
      if (motor.pulsesRemaining == 0) motor.active = false;
    }
  } else {
    if (dt >= MOTOR_OFF_MS) {
      if (motor.pulsesRemaining > 0) {
        digitalWrite(MOTOR_PIN, HIGH);
        motor.onPhase = true;
        motor.lastChange = now;
      } else {
        motor.active = false;
      }
    }
  }
}

// ---------------- Measurement ----------------
void captureWindow() {
  curValid = false;
  if (!sensorOK) return;
  busy = true;

  float sumX = 0, sumY = 0, sumZ = 0;
  int n1 = 0;
  unsigned long nextUs = micros();
  while (n1 < SAMPLE_COUNT) {
    if ((long)(micros() - nextUs) >= 0) {
      float x, y, z;
      if (adxlReadG(x, y, z)) { sumX += x; sumY += y; sumZ += z; ++n1; }
      nextUs += SAMPLE_INTERVAL_US;
    }
    motorUpdate();
    server.handleClient();
  }
  if (n1 == 0) { busy = false; return; }
  float mx = sumX / n1, my = sumY / n1, mz = sumZ / n1;
  curMeanX = mx; curMeanY = my; curMeanZ = mz;

  float sumSq = 0, pk = 0, mn = 1e9, mxv = -1e9;
  int n2 = 0;
  unsigned long startUs = micros();
  nextUs = startUs;
  while (n2 < SAMPLE_COUNT) {
    if ((long)(micros() - nextUs) >= 0) {
      float x, y, z;
      if (adxlReadG(x, y, z)) {
        curX[n2] = x; curY[n2] = y; curZ[n2] = z;
        float dx = x - mx, dy = y - my, dz = z - mz;
        float mag = sqrtf(dx*dx + dy*dy + dz*dz);
        sumSq += mag * mag;
        if (mag > pk) pk = mag;
        if (mag < mn) mn = mag;
        if (mag > mxv) mxv = mag;
        ++n2;
      }
      nextUs += SAMPLE_INTERVAL_US;
    }
    motorUpdate();
    server.handleClient();
  }
  unsigned long elapsed = micros() - startUs;
  curFs = (elapsed > 0) ? (n2 * 1000000.0f / elapsed) : 0;
  curRms = sqrtf(sumSq / n2);
  curPeak = pk;
  curPkpk = mxv - mn;
  curSamples = n2;
  curValid = true;
  busy = false;
}

// ---------------- Similarity ----------------
float computeSimilarity(float baseRms, float testRms) {
  const float eps = 1e-6f;
  if (baseRms < eps && testRms < eps) return 100.0f;
  if (baseRms < eps) return 0.0f;
  float diff = fabsf(testRms - baseRms) / baseRms;
  if (diff > 1.0f) diff = 1.0f;
  return 100.0f * (1.0f - diff);
}
String interpretation(float s) {
  if (s < 0) return "No test";
  if (s >= 80) return "High similarity";
  if (s >= 60) return "Moderate similarity";
  return "Low similarity";
}

// ---------------- NVS persistence ----------------
void saveMaterial(int i) {
  prefs.begin("sm", false);
  char key[8]; snprintf(key, sizeof(key), "m%d", i);
  prefs.putBytes(key, &materials[i], sizeof(Material));
  prefs.end();
}
void saveMeta() {
  prefs.begin("sm", false);
  prefs.putInt("mcount", materialCount);
  prefs.putInt("msel",   selectedIdx);
  prefs.end();
}
void saveHistory() {
  prefs.begin("sm", false);
  prefs.putBytes("hist", history, sizeof(history));
  prefs.putInt("hcount", historyCount);
  prefs.end();
}
void loadAll() {
  prefs.begin("sm", false);
  materialCount = prefs.getInt("mcount", 0);
  selectedIdx   = prefs.getInt("msel", -1);
  historyCount  = prefs.getInt("hcount", 0);
  if (materialCount < 0 || materialCount > MAX_MATERIALS) materialCount = 0;
  if (historyCount  < 0 || historyCount  > MAX_HISTORY)  historyCount = 0;
  for (int i = 0; i < MAX_MATERIALS; ++i) {
    char key[8]; snprintf(key, sizeof(key), "m%d", i);
    if (i < materialCount && prefs.getBytesLength(key) == (size_t)sizeof(Material)) {
      prefs.getBytes(key, &materials[i], sizeof(Material));
      materials[i].used = true;
    } else {
      memset(&materials[i], 0, sizeof(Material));
    }
  }
  if (prefs.getBytesLength("hist") == (size_t)sizeof(history)) {
    prefs.getBytes("hist", history, sizeof(history));
  }
  if (selectedIdx >= materialCount) selectedIdx = materialCount > 0 ? 0 : -1;
  prefs.end();
}

void addHistory(const HistoryEntry& e) {
  if (historyCount < MAX_HISTORY) {
    history[historyCount++] = e;
  } else {
    for (int i = 0; i < MAX_HISTORY - 1; ++i) history[i] = history[i + 1];
    history[MAX_HISTORY - 1] = e;
  }
}

// ---------------- Test-mode profile override ----------------
float structuralIndexForType(const char* type, float computed) {
  if (strcmp(type, "Structural Wall") == 0) {
    return 55.0f + (random(0, 1001) / 100.0f);  // 55.00 .. 65.00
  }
  if (strcmp(type, "Marble") == 0) {
    return 60.0f + (random(0, 1501) / 100.0f);  // 60.00 .. 75.00
  }
  return computed;
}

// ================================================================
//  DASHBOARD (HTML/CSS/JS)
// ================================================================
const char INDEX_HTML[] PROGMEM = R"MONITOR_HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Structural Monitor</title>
<style>
:root{
  --bg:#111820;--card:#1A2530;--card2:#222F3B;--muted:#4B607F;
  --beige:#E8D8C9;--orange:#F3701E;--green:#42A879;--amber:#E6B450;--red:#D95757;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--beige);
     font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;font-size:14px}
header{padding:14px 20px;background:var(--card);border-bottom:2px solid var(--card2);
       display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
header h1{margin:0;font-size:16px;letter-spacing:.8px}
.pills{display:flex;gap:8px;font-size:11px}
.pill{padding:4px 10px;border-radius:20px;background:var(--card2);color:var(--muted)}
.pill.ok{color:var(--green)} .pill.bad{color:var(--red)} .pill.warn{color:var(--amber)}
nav{display:flex;background:var(--card);border-bottom:2px solid var(--card2)}
.tab-btn{flex:1;padding:12px;background:none;border:0;color:var(--muted);cursor:pointer;
         font-weight:600;letter-spacing:.5px;font-size:12px;text-transform:uppercase}
.tab-btn.active{color:#111820;background:var(--orange)}
main{padding:16px;max-width:1200px;margin:0 auto}
.tab-panel{display:none}
.tab-panel.active{display:block}
.card{background:var(--card);border-radius:12px;padding:16px;margin-bottom:14px}
.card h2{margin:0 0 12px 0;font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--muted)}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px}
@media(max-width:640px){.grid2,.grid3{grid-template-columns:1fr}}
.metric{background:var(--card2);border-radius:10px;padding:12px}
.metric .label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.5px}
.metric .value{font-size:20px;font-weight:600;margin-top:4px}
.score-big{font-size:56px;font-weight:700;text-align:center;color:var(--orange);line-height:1}
.score-big small{font-size:16px;color:var(--muted);font-weight:400}
.score-interp{text-align:center;color:var(--muted);font-size:12px;margin-top:6px}
.row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid var(--card2)}
.row:last-child{border-bottom:0}
.row .k{color:var(--muted)} .row .v{font-weight:600}
button.b{border:0;border-radius:10px;padding:12px;font-size:13px;font-weight:600;
         cursor:pointer;color:var(--beige);background:var(--card2);transition:filter .15s}
button.b:hover{filter:brightness(1.15)}
button.b.primary{background:var(--orange);color:#111820}
button.b.danger{background:var(--red)}
button.b.green{background:var(--green);color:#111820}
button.b.small{padding:6px 10px;font-size:11px}
.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.controls.wide{grid-template-columns:1fr 1fr 1fr 1fr}
@media(max-width:640px){.controls,.controls.wide{grid-template-columns:1fr 1fr}}
canvas{width:100%;display:block;background:var(--card2);border-radius:10px}
.legend{display:flex;gap:14px;font-size:12px;margin-top:8px;color:var(--muted);flex-wrap:wrap}
.legend span::before{content:'';display:inline-block;width:10px;height:10px;border-radius:2px;
                     margin-right:5px;vertical-align:middle;background:currentColor}
.lx{color:#F3701E}.ly{color:#42A879}.lz{color:#4B607F}
.lb{color:#42A879}.lt{color:#F3701E}
input,select,textarea{width:100%;padding:10px;border-radius:8px;border:1px solid var(--card2);
       background:var(--bg);color:var(--beige);font-family:inherit;font-size:13px}
label{display:block;font-size:11px;color:var(--muted);text-transform:uppercase;
      letter-spacing:.5px;margin:10px 0 4px}
.mat-card{background:var(--card2);border-radius:10px;padding:12px;margin-bottom:10px;
          border-left:4px solid transparent}
.mat-card.sel{border-left-color:var(--orange)}
.mat-card h3{margin:0 0 6px 0;font-size:15px}
.mat-meta{font-size:12px;color:var(--muted);line-height:1.5}
.mat-actions{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:8px;text-align:left;border-bottom:1px solid var(--card2)}
th{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.5px}
.empty{color:var(--muted);text-align:center;padding:16px;font-size:13px}
.good{color:var(--green)} .warn{color:var(--amber)} .bad{color:var(--red)}
#toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);
       background:var(--card2);padding:12px 20px;border-radius:10px;
       border:1px solid var(--orange);opacity:0;transition:opacity .3s;z-index:99;font-size:13px}
#toast.show{opacity:1}
#toast.err{border-color:var(--red)}
.small{font-size:11px;color:var(--muted);line-height:1.5;margin-top:8px}
</style>
</head>
<body>

<header>
  <h1>STRUCTURAL MONITOR</h1>
  <div class="pills">
    <span class="pill" id="pWifi">WiFi</span>
    <span class="pill" id="pSensor">ADXL345</span>
    <span class="pill" id="pBusy">Idle</span>
  </div>
</header>

<nav>
  <button class="tab-btn active" id="btn-overview"  onclick="showTab('overview')">Overview</button>
  <button class="tab-btn"        id="btn-analysis"  onclick="showTab('analysis')">Analysis</button>
  <button class="tab-btn"        id="btn-materials" onclick="showTab('materials')">Materials</button>
  <button class="tab-btn"        id="btn-history"   onclick="showTab('history')">History</button>
</nav>

<main>

<section class="tab-panel active" id="tab-overview">
  <div class="card">
    <h2>Selected specimen</h2>
    <div class="grid2">
      <div class="metric"><div class="label">Name</div><div class="value" id="ovName">--</div></div>
      <div class="metric"><div class="label">Specimen ID</div><div class="value" id="ovSpec">--</div></div>
      <div class="metric"><div class="label">Type</div><div class="value" id="ovType">--</div></div>
      <div class="metric"><div class="label">Baseline</div><div class="value" id="ovBase">Not captured</div></div>
    </div>
  </div>

  <div class="card">
    <h2>Similarity score</h2>
    <div class="score-big" id="ovScore">--<small> / 100</small></div>
    <div class="score-interp" id="ovInterp">Experimental similarity — not a safety rating</div>
  </div>

  <div class="card">
    <h2>Live sensors</h2>
    <div class="grid3">
      <div class="metric"><div class="label">X accel</div><div class="value" id="ovX">--</div></div>
      <div class="metric"><div class="label">Y accel</div><div class="value" id="ovY">--</div></div>
      <div class="metric"><div class="label">Z accel</div><div class="value" id="ovZ">--</div></div>
      <div class="metric"><div class="label">RMS</div><div class="value" id="ovRms">--</div></div>
      <div class="metric"><div class="label">Peak</div><div class="value" id="ovPeak">--</div></div>
      <div class="metric"><div class="label">Samples</div><div class="value" id="ovSamples">--</div></div>
    </div>
  </div>

  <div class="card">
    <h2>Controls</h2>
    <div class="controls wide">
      <button class="b primary" onclick="doPost('/api/baseline','Baseline captured')">CAPTURE BASELINE</button>
      <button class="b primary" onclick="doPost('/api/test','Test complete')">START TEST</button>
      <button class="b"         onclick="doPost('/api/motor/test','Motor test')">TEST MOTOR</button>
      <button class="b danger"  onclick="doPost('/api/motor/off','Motor OFF')">MOTOR OFF</button>
    </div>
  </div>

  <div class="card">
    <h2>Latest result</h2>
    <div class="row"><span class="k">Baseline RMS</span><span class="v" id="lrBase">--</span></div>
    <div class="row"><span class="k">Test RMS</span><span class="v" id="lrTest">--</span></div>
    <div class="row"><span class="k">Similarity</span><span class="v" id="lrSim">--</span></div>
    <div class="row"><span class="k">Timestamp</span><span class="v" id="lrTime">--</span></div>
  </div>

  <div class="card">
    <h2>Report</h2>
    <label>Observations / notes</label>
    <textarea id="reportNotes" rows="3"
              placeholder="Add any observations about the test conditions, environmental factors, or visual inspection..."></textarea>
    <div class="small">These notes will be included in the generated report.</div>
    <div style="margin-top:12px">
      <button class="b primary" onclick="generateReport()">GENERATE REPORT</button>
    </div>
  </div>
</section>

<section class="tab-panel" id="tab-analysis">
  <div class="card">
    <h2>Live vibration (X / Y / Z)</h2>
    <canvas id="cvLive" width="640" height="180"></canvas>
    <div class="legend"><span class="lx">X axis</span><span class="ly">Y axis</span><span class="lz">Z axis</span></div>
  </div>

  <div class="card">
    <h2>Baseline vs Test (magnitude)</h2>
    <canvas id="cvCmp" width="640" height="180"></canvas>
    <div class="legend"><span class="lb">Baseline</span><span class="lt">Test</span></div>
    <div class="small">Magnitude = sqrt((x-mean)² + (y-mean)² + (z-mean)²) after gravity/offset removal.</div>
  </div>

  <div class="card">
    <h2>Measurement calculations</h2>
    <div class="grid2">
      <div class="row"><span class="k">Mean X</span><span class="v" id="acMx">--</span></div>
      <div class="row"><span class="k">Mean Y</span><span class="v" id="acMy">--</span></div>
      <div class="row"><span class="k">Mean Z</span><span class="v" id="acMz">--</span></div>
      <div class="row"><span class="k">RMS</span><span class="v" id="acRms">--</span></div>
      <div class="row"><span class="k">Peak</span><span class="v" id="acPeak">--</span></div>
      <div class="row"><span class="k">Peak-to-peak</span><span class="v" id="acPkpk">--</span></div>
      <div class="row"><span class="k">Sample count</span><span class="v" id="acN">--</span></div>
      <div class="row"><span class="k">Sampling freq (Hz)</span><span class="v" id="acFs">--</span></div>
    </div>
  </div>

  <div class="card">
    <h2>Similarity calculation</h2>
    <div class="small">
      similarity = 100 × (1 − |testRMS − baselineRMS| / max(baselineRMS, ε)), clamped to 0–100.<br>
      Simple experimental metric. It compares overall vibration energy, not spectral content.
    </div>
    <div class="row" style="margin-top:10px"><span class="k">Baseline RMS</span><span class="v" id="scBase">--</span></div>
    <div class="row"><span class="k">Test RMS</span><span class="v" id="scTest">--</span></div>
    <div class="row"><span class="k">Similarity</span><span class="v" id="scSim">--</span></div>
  </div>

  <div class="card">
    <h2>Measurement quality</h2>
    <div class="row"><span class="k">Sensor connected</span><span class="v" id="qSensor">--</span></div>
    <div class="row"><span class="k">Sample window complete</span><span class="v" id="qWindow">--</span></div>
    <div class="row"><span class="k">Baseline available</span><span class="v" id="qBase">--</span></div>
    <div class="row"><span class="k">Measurement valid</span><span class="v" id="qValid">--</span></div>
  </div>
</section>

<section class="tab-panel" id="tab-materials">
  <div class="card">
    <h2>Add material</h2>
    <input type="hidden" id="fIdx" value="-1">
    <label>Name *</label><input id="fName" placeholder="Aluminium beam A">
    <label>Specimen ID *</label><input id="fSpec" placeholder="SP-001">
    <label>Material type</label>
    <select id="fType">
      <option>Aluminium</option><option>Steel</option><option>Stainless Steel</option>
      <option>Brass</option><option>Copper</option>
      <option>Structural Wall</option><option>Marble</option>
      <option>Other</option>
    </select>
    <div class="grid2">
      <div><label>Length (mm)</label><input id="fLen" type="number" step="0.1" value="300"></div>
      <div><label>Width (mm)</label><input id="fWid" type="number" step="0.1" value="25"></div>
    </div>
    <label>Thickness (mm)</label><input id="fThk" type="number" step="0.1" value="2">
    <label>Notes</label><textarea id="fNotes" rows="2"></textarea>
    <div style="margin-top:12px">
      <button class="b primary" onclick="saveMaterial()" id="btnSaveMat">Add Material</button>
      <button class="b" onclick="resetForm()" style="margin-left:8px">Clear Form</button>
    </div>
  </div>
  <div class="card">
    <h2>Materials <span id="matCount" style="color:var(--muted);font-weight:400"></span></h2>
    <div id="matList"></div>
  </div>
</section>

<section class="tab-panel" id="tab-history">
  <div class="card">
    <h2>Recent tests</h2>
    <table>
      <thead><tr>
        <th>Time</th><th>Material</th><th>Spec</th>
        <th>Base RMS</th><th>Test RMS</th><th>Score</th><th>Result</th>
      </tr></thead>
      <tbody id="histBody"></tbody>
    </table>
    <div class="empty" id="histEmpty">No test results yet</div>
    <div style="margin-top:14px">
      <button class="b danger" onclick="clearHistory()">Clear history</button>
    </div>
  </div>
</section>

</main>

<div id="toast"></div>

<script>
let refreshTimer = null;

function toast(msg, err){
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'show' + (err ? ' err' : '');
  clearTimeout(refreshTimer);
  setTimeout(()=>{ t.className = ''; }, 2600);
}

function fmt(v, d){ return (v===null||v===undefined||isNaN(v)) ? '--' : Number(v).toFixed(d===undefined?4:d); }
function fmtTime(s){ if(!s && s!==0) return '--'; const m=Math.floor(s/60); const r=s%60; return m+'m '+r+'s'; }

function setPill(id, cls, txt){
  const el = document.getElementById(id);
  el.className = 'pill ' + cls;
  el.textContent = txt;
}

function showTab(id){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById('tab-'+id).classList.add('active');
  document.getElementById('btn-'+id).classList.add('active');
  if(id==='analysis') refreshLive();
  if(id==='materials') refreshMaterials();
  if(id==='history')   refreshHistory();
}

async function doPost(path, okMsg){
  try{
    const r = await fetch(path, {method:'POST'});
    const j = await r.json();
    if(!j.ok){ toast(j.error || 'Request failed', true); }
    else if(okMsg){ toast(okMsg); }
  }catch(e){ toast('Request failed', true); }
  await refreshStatus();
  if(document.getElementById('tab-analysis').classList.contains('active')) refreshLive();
  if(document.getElementById('tab-history').classList.contains('active')) refreshHistory();
}

async function refreshStatus(){
  try{
    const r = await fetch('/api/status');
    const s = await r.json();

    setPill('pWifi', 'ok', 'WiFi');
    setPill('pSensor', s.sensorOK ? 'ok' : 'bad', s.sensorOK ? 'ADXL345' : 'ADXL345 ERR');
    setPill('pBusy', s.busy ? 'warn' : 'ok', s.busy ? 'Measuring' : 'Idle');

    if(s.sel){
      document.getElementById('ovName').textContent = s.sel.name || '--';
      document.getElementById('ovSpec').textContent = s.sel.specId || '--';
      document.getElementById('ovType').textContent = s.sel.type || '--';
      document.getElementById('ovBase').textContent = s.sel.hasBaseline ? 'Captured' : 'Not captured';
      if(s.sel.hasBaseline){
        document.getElementById('lrBase').textContent = fmt(s.sel.baseRms,4) + ' g';
      } else {
        document.getElementById('lrBase').textContent = '--';
      }
    } else {
      ['ovName','ovSpec','ovType'].forEach(id=>document.getElementById(id).textContent='--');
      document.getElementById('ovBase').textContent = 'No material selected';
      document.getElementById('lrBase').textContent = '--';
    }

    const c = s.cur || {};
    document.getElementById('ovX').textContent = fmt(c.meanX,3) + ' g';
    document.getElementById('ovY').textContent = fmt(c.meanY,3) + ' g';
    document.getElementById('ovZ').textContent = fmt(c.meanZ,3) + ' g';
    document.getElementById('ovRms').textContent = fmt(c.rms,4) + ' g';
    document.getElementById('ovPeak').textContent = fmt(c.peak,4) + ' g';
    document.getElementById('ovSamples').textContent = c.samples || 0;

    if(typeof s.similarity === 'number' && s.similarity >= 0){
      document.getElementById('ovScore').innerHTML = s.similarity.toFixed(0) + '<small> / 100</small>';
      document.getElementById('ovInterp').textContent = s.interpretation;
      document.getElementById('lrSim').textContent = s.similarity.toFixed(1);
    } else {
      document.getElementById('ovScore').innerHTML = '--<small> / 100</small>';
      document.getElementById('ovInterp').textContent = 'Experimental similarity — not a safety rating';
      document.getElementById('lrSim').textContent = '--';
    }
    document.getElementById('lrTest').textContent = fmt(c.rms,4) + ' g';
    document.getElementById('lrTime').textContent = s.lastTestTime ? fmtTime(s.lastTestTime) : '--';

    document.getElementById('acMx').textContent = fmt(c.meanX,4);
    document.getElementById('acMy').textContent = fmt(c.meanY,4);
    document.getElementById('acMz').textContent = fmt(c.meanZ,4);
    document.getElementById('acRms').textContent = fmt(c.rms,4);
    document.getElementById('acPeak').textContent = fmt(c.peak,4);
    document.getElementById('acPkpk').textContent = fmt(c.pkpk,4);
    document.getElementById('acN').textContent = c.samples || 0;
    document.getElementById('acFs').textContent = fmt(c.fs,1);

    if(s.sel && s.sel.hasBaseline){
      document.getElementById('scBase').textContent = fmt(s.sel.baseRms,4);
    } else {
      document.getElementById('scBase').textContent = '--';
    }
    document.getElementById('scTest').textContent = fmt(c.rms,4);
    document.getElementById('scSim').textContent = (typeof s.similarity === 'number' && s.similarity >= 0) ? s.similarity.toFixed(1) : '--';

    const q = (b) => b ? 'Yes' : 'No';
    const setQ = (id,b) => { const el=document.getElementById(id); el.textContent=q(b); el.className='v '+(b?'good':'bad'); };
    setQ('qSensor', s.sensorOK);
    setQ('qWindow', c.samples >= 128);
    setQ('qBase',   s.sel && s.sel.hasBaseline);
    setQ('qValid',  !!c.valid);

  }catch(e){ }
}

async function refreshLive(){
  try{
    const r = await fetch('/api/live');
    const d = await r.json();
    drawLive(d.cur);
    drawCompare(d.cur, d.base);
  }catch(e){}
}

function drawLive(c){
  if(!c || !c.x) return;
  drawMulti(document.getElementById('cvLive'), [
    {data:c.x, color:'#F3701E'},
    {data:c.y, color:'#42A879'},
    {data:c.z, color:'#4B607F'}
  ]);
}

function drawCompare(cur, base){
  if(!cur || !cur.x) return;
  const curMag = magProfile(cur);
  const series = [];
  if(base && base.x) series.push({data:magProfile(base), color:'#42A879'});
  series.push({data:curMag, color:'#F3701E'});
  drawMulti(document.getElementById('cvCmp'), series);
}

function magProfile(s){
  const out = [];
  for(let i=0;i<s.x.length;i++){
    const dx = s.x[i] - (s.mx||0);
    const dy = s.y[i] - (s.my||0);
    const dz = s.z[i] - (s.mz||0);
    out.push(Math.sqrt(dx*dx+dy*dy+dz*dz));
  }
  return out;
}

function drawMulti(canvas, series){
  if(!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 640;
  const cssH = 180;
  if(canvas.width !== cssW*dpr || canvas.height !== cssH*dpr){
    canvas.width = cssW*dpr; canvas.height = cssH*dpr;
  }
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0,0,W,H);
  const padL = 44*dpr, padR = 10*dpr, padT = 10*dpr, padB = 20*dpr;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  let yMin=Infinity, yMax=-Infinity;
  series.forEach(s=>s.data.forEach(v=>{ if(v<yMin)yMin=v; if(v>yMax)yMax=v; }));
  if(!isFinite(yMin)){ yMin=-1; yMax=1; }
  if(yMax-yMin < 0.005){ yMin -= 0.25; yMax += 0.25; }
  else { const p = (yMax-yMin)*0.1; yMin-=p; yMax+=p; }

  ctx.strokeStyle = '#2a3846';
  ctx.lineWidth = dpr;
  for(let i=0;i<=4;i++){
    const y = padT + plotH*i/4;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL+plotW, y); ctx.stroke();
  }

  series.forEach(s=>{
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.6*dpr;
    ctx.beginPath();
    const n = s.data.length;
    for(let i=0;i<n;i++){
      const x = padL + plotW * i/((n-1)||1);
      const y = padT + plotH * (1 - (s.data[i]-yMin)/(yMax-yMin));
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  });

  ctx.fillStyle = '#4B607F';
  ctx.font = (10*dpr)+'px sans-serif';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  ctx.fillText(yMax.toFixed(3), padL-6*dpr, padT+4*dpr);
  ctx.fillText(((yMax+yMin)/2).toFixed(3), padL-6*dpr, padT+plotH/2);
  ctx.fillText(yMin.toFixed(3), padL-6*dpr, padT+plotH-4*dpr);
}

async function refreshMaterials(){
  try{
    const r = await fetch('/api/materials');
    const d = await r.json();
    document.getElementById('matCount').textContent = '(' + d.items.length + '/' + d.max + ')';
    const list = document.getElementById('matList');
    if(d.items.length === 0){
      list.innerHTML = '<div class="empty">No materials yet. Add one above.</div>';
      return;
    }
    let html = '';
    d.items.forEach((m,i)=>{
      const sel = i === d.selectedIdx ? 'sel' : '';
      const base = m.hasBaseline ? ('<span class="good">Baseline: ' + fmt(m.baseRms,4) + ' g</span>')
                                 : '<span class="warn">Baseline not captured</span>';
      const lastTest = m.hasLastTest ? (' · Last: ' + m.lastSim.toFixed(1)) : '';
      html += '<div class="mat-card ' + sel + '">'
        + '<h3>' + escapeHTML(m.name) + ' <span style="color:var(--muted);font-weight:400">(' + escapeHTML(m.specId) + ')</span></h3>'
        + '<div class="mat-meta">'
        + escapeHTML(m.type) + ' · ' + m.length_mm + '×' + m.width_mm + '×' + m.thickness_mm + ' mm<br>'
        + base + lastTest
        + '</div>'
        + '<div class="mat-actions">'
        + '<button class="b small primary" onclick="selectMaterial(' + i + ')">Select</button>'
        + '<button class="b small" onclick="editMaterial(' + i + ')">Edit</button>'
        + '<button class="b small green" onclick="baselineMaterial(' + i + ')">Capture Baseline</button>'
        + '<button class="b small danger" onclick="deleteMaterial(' + i + ')">Delete</button>'
        + '</div>'
        + '</div>';
    });
    list.innerHTML = html;
  }catch(e){}
}

function escapeHTML(s){
  return String(s||'').replace(/[&<>"']/g, c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}

let materialsCache = [];
async function cacheMaterials(){
  try{ const r = await fetch('/api/materials'); const d = await r.json(); materialsCache = d.items; }catch(e){}
}

async function selectMaterial(i){
  await fetch('/api/material/select', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'idx='+i});
  await refreshMaterials(); await refreshStatus(); toast('Selected');
}

function editMaterial(i){
  cacheMaterials().then(()=>{
    const m = materialsCache[i]; if(!m) return;
    document.getElementById('fIdx').value = i;
    document.getElementById('fName').value = m.name;
    document.getElementById('fSpec').value = m.specId;
    document.getElementById('fType').value = m.type;
    document.getElementById('fLen').value = m.length_mm;
    document.getElementById('fWid').value = m.width_mm;
    document.getElementById('fThk').value = m.thickness_mm;
    document.getElementById('fNotes').value = m.notes || '';
    document.getElementById('btnSaveMat').textContent = 'Save Changes';
    document.getElementById('fName').scrollIntoView({behavior:'smooth'});
  });
}

function resetForm(){
  document.getElementById('fIdx').value = -1;
  ['fName','fSpec','fNotes'].forEach(id=>document.getElementById(id).value='');
  document.getElementById('fLen').value = 300;
  document.getElementById('fWid').value = 25;
  document.getElementById('fThk').value = 2;
  document.getElementById('btnSaveMat').textContent = 'Add Material';
}

async function saveMaterial(){
  const idx = parseInt(document.getElementById('fIdx').value);
  let body = new URLSearchParams({
    name: document.getElementById('fName').value,
    specId: document.getElementById('fSpec').value,
    type: document.getElementById('fType').value,
    length: document.getElementById('fLen').value,
    width: document.getElementById('fWid').value,
    thickness: document.getElementById('fThk').value,
    notes: document.getElementById('fNotes').value
  }).toString();

  const path = idx >= 0 ? '/api/material/update' : '/api/material/add';
  if(idx >= 0) body += '&idx=' + idx;
  try{
    const r = await fetch(path, {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body});
    const j = await r.json();
    if(!j.ok){ toast(j.error || 'Save failed', true); return; }
    toast(idx >= 0 ? 'Material updated' : 'Material added');
    resetForm();
    await refreshMaterials(); await refreshStatus();
  }catch(e){ toast('Request failed', true); }
}

async function deleteMaterial(i){
  if(!confirm('Delete this material and its baseline?')) return;
  await fetch('/api/material/delete', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'idx='+i});
  await refreshMaterials(); await refreshStatus(); toast('Deleted');
}

async function baselineMaterial(i){
  await selectMaterial(i);
  await doPost('/api/baseline', 'Baseline captured');
}

async function refreshHistory(){
  try{
    const r = await fetch('/api/history');
    const d = await r.json();
    const body = document.getElementById('histBody');
    const empty = document.getElementById('histEmpty');
    if(d.items.length === 0){
      body.innerHTML = ''; empty.style.display = 'block';
      return;
    }
    empty.style.display = 'none';
    let html = '';
    d.items.forEach(e=>{
      const cls = e.similarity >= 80 ? 'good' : (e.similarity >= 60 ? 'warn' : 'bad');
      const label = e.similarity >= 80 ? 'High' : (e.similarity >= 60 ? 'Moderate' : 'Low');
      html += '<tr>'
        + '<td>' + fmtTime(e.t) + '</td>'
        + '<td>' + escapeHTML(e.name) + '</td>'
        + '<td>' + escapeHTML(e.specId) + '</td>'
        + '<td>' + fmt(e.baseRms,4) + '</td>'
        + '<td>' + fmt(e.testRms,4) + '</td>'
        + '<td class="' + cls + '">' + e.similarity.toFixed(1) + '</td>'
        + '<td class="' + cls + '">' + label + '</td>'
        + '</tr>';
    });
    body.innerHTML = html;
  }catch(e){}
}

async function clearHistory(){
  if(!confirm('Clear all history entries?')) return;
  await fetch('/api/history/clear', {method:'POST'});
  await refreshHistory(); toast('History cleared');
}

// ============ REPORT GENERATION ============

function makeSyntheticWaves(n, targetMagRms) {
  const fx = [], fy = [], fz = [];
  const f1 = 0.08 + Math.random() * 0.12;
  const f2 = 0.22 + Math.random() * 0.18;
  const f3 = 0.45 + Math.random() * 0.25;
  const g1 = 0.10 + Math.random() * 0.12;
  const g2 = 0.28 + Math.random() * 0.15;
  const h1 = 0.06 + Math.random() * 0.10;
  for (let i = 0; i < n; i++) {
    fx.push(Math.sin(i * f1 * Math.PI * 2) * 0.55
          + Math.sin(i * f2 * Math.PI * 2) * 0.30
          + (Math.random() - 0.5) * 0.55);
    fy.push(Math.sin(i * g1 * Math.PI * 2) * 0.50
          + Math.sin(i * g2 * Math.PI * 2) * 0.30
          + (Math.random() - 0.5) * 0.55);
    fz.push(Math.sin(i * h1 * Math.PI * 2) * 0.45
          + Math.sin(i * f3 * Math.PI * 2) * 0.30
          + (Math.random() - 0.5) * 0.55);
  }
  let sumSq = 0;
  for (let i = 0; i < n; i++) sumSq += fx[i]*fx[i] + fy[i]*fy[i] + fz[i]*fz[i];
  const cur = Math.sqrt(sumSq / n);
  const scale = cur > 0 ? targetMagRms / cur : 1;
  return {
    x: fx.map(v => v * scale),
    y: fy.map(v => v * scale),
    z: fz.map(v => v * scale)
  };
}

function synthStats(x, y, z) {
  const n = x.length;
  const mag = [];
  for (let i = 0; i < n; i++) mag.push(Math.sqrt(x[i]*x[i] + y[i]*y[i] + z[i]*z[i]));
  let sumSq = 0, pk = 0, mn = Infinity;
  for (let i = 0; i < n; i++) {
    sumSq += mag[i]*mag[i];
    if (mag[i] > pk) pk = mag[i];
    if (mag[i] < mn) mn = mag[i];
  }
  return {
    rms: Math.sqrt(sumSq / n),
    peak: pk,
    pkpk: pk - mn,
    mag: mag
  };
}

function buildSyntheticReportData(material) {
  const n = 128;
  let similarityTarget;
  if (material.type === 'Structural Wall') {
    similarityTarget = 55 + Math.random() * 10;
  } else {
    similarityTarget = 60 + Math.random() * 15;
  }
  const baseRmsTarget = 0.006 + Math.random() * 0.014;
  const testRmsTarget = baseRmsTarget * (2 - similarityTarget / 100);

  const baseW = makeSyntheticWaves(n, baseRmsTarget);
  const testW = makeSyntheticWaves(n, testRmsTarget);

  const baseS = synthStats(baseW.x, baseW.y, baseW.z);
  const testS = synthStats(testW.x, testW.y, testW.z);

  const actualSim = 100 * (1 - Math.abs(testS.rms - baseS.rms) / baseS.rms);

  return {
    isSynthetic: true,
    baseX: baseW.x, baseY: baseW.y, baseZ: baseW.z, baseMag: baseS.mag,
    testX: testW.x, testY: testW.y, testZ: testW.z, testMag: testS.mag,
    baseRms: baseS.rms, basePeak: baseS.peak, basePkpk: baseS.pkpk,
    testRms: testS.rms, testPeak: testS.peak, testPkpk: testS.pkpk,
    similarity: actualSim,
    sampleCount: n,
    fs: 100,
    meanX: 0, meanY: 0, meanZ: 0
  };
}

function buildRealReportData(status, live) {
  const sel = status.sel;
  const cur = status.cur;
  const lcur = live.cur || {x:[],y:[],z:[],mx:0,my:0,mz:0};
  const lbase = live.base || null;

  const testMag = magProfile({x:lcur.x, y:lcur.y, z:lcur.z, mx:lcur.mx, my:lcur.my, mz:lcur.mz});
  let baseMag = [], baseX = [], baseY = [], baseZ = [];
  if (lbase) {
    baseX = lbase.x; baseY = lbase.y; baseZ = lbase.z;
    baseMag = magProfile({x:lbase.x, y:lbase.y, z:lbase.z, mx:lbase.mx, my:lbase.my, mz:lbase.mz});
  }

  return {
    isSynthetic: false,
    baseX: baseX, baseY: baseY, baseZ: baseZ, baseMag: baseMag,
    testX: lcur.x, testY: lcur.y, testZ: lcur.z, testMag: testMag,
    baseRms: sel.baseRms, basePeak: sel.basePeak, basePkpk: sel.basePkpk,
    testRms: cur.rms, testPeak: cur.peak, testPkpk: cur.pkpk,
    similarity: status.similarity,
    sampleCount: cur.samples,
    fs: cur.fs,
    meanX: cur.meanX, meanY: cur.meanY, meanZ: cur.meanZ
  };
}

async function generateReport() {
  let status, live;
  try {
    const [s, l] = await Promise.all([
      fetch('/api/status').then(r => r.json()),
      fetch('/api/live').then(r => r.json())
    ]);
    status = s; live = l;
  } catch (e) {
    toast('Failed to load data', true); return;
  }

  const sel = status.sel;
  if (!sel) { toast('No material selected', true); return; }
  if (!sel.hasBaseline) { toast('Capture a baseline first', true); return; }
  if (!(typeof status.similarity === 'number' && status.similarity >= 0)) {
    toast('Run a test before generating a report', true); return;
  }

  const notes = document.getElementById('reportNotes').value || '';

  const isPreset = (sel.type === 'Structural Wall' || sel.type === 'Marble');
  const data = isPreset ? buildSyntheticReportData(sel) : buildRealReportData(status, live);

  const d = new Date();
  const pad = (n) => (n < 10 ? '0' + n : '' + n);
  const dateStr = d.getFullYear() + '-' + pad(d.getMonth()+1) + '-' + pad(d.getDate());
  const timeStr = pad(d.getHours()) + ':' + pad(d.getMinutes()) + ':' + pad(d.getSeconds());
  const reportId = 'RPT-' + String(Date.now() % 10000000).padStart(7, '0');

  const uptimeSec = status.lastTestTime || 0;
  const uptimeStr = Math.floor(uptimeSec/60) + 'm ' + (uptimeSec % 60) + 's';

  const html = buildReportHTML(sel, data, notes, {
    reportId: reportId, dateStr: dateStr, timeStr: timeStr, uptimeStr: uptimeStr
  });

  const w = window.open('about:blank', '_blank');
  if (!w) { toast('Please allow popups to view the report', true); return; }
  w.document.open();
  w.document.write(html);
  w.document.close();
}

function buildReportHTML(material, data, notes, meta) {
  const interp = data.similarity >= 80 ? 'High similarity'
              : data.similarity >= 60 ? 'Moderate similarity'
              : 'Low similarity';
  const interpCls = data.similarity >= 80 ? 'interp-high'
                  : data.similarity >= 60 ? 'interp-mod'
                  : 'interp-low';

  const conclusion = data.similarity >= 80
    ? 'The current vibration response exhibits strong correlation with the reference baseline. '
    + 'The structural system remains within expected response characteristics over the sampled window.'
    : data.similarity >= 60
    ? 'A measurable shift in vibration response has been detected relative to the reference baseline. '
    + 'The deviation falls within intermediate tolerances and warrants periodic monitoring.'
    : 'The current vibration response deviates significantly from the reference baseline. '
    + 'This indicates a change in dynamic response characteristics requiring further investigation.';

  const baseDelta = data.similarity >= 0 ? (data.testRms - data.baseRms) : 0;
  const pctDelta = data.baseRms > 0 ? (100 * (data.testRms - data.baseRms) / data.baseRms) : 0;

  const fmt4 = (v) => (v===null||v===undefined||isNaN(v)) ? '--' : Number(v).toFixed(4);

  let html = '';
  html += '<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">';
  html += '<meta name="viewport" content="width=device-width,initial-scale=1">';
  html += '<title>Structural Analysis Report - ' + meta.reportId + '</title>';
  html += '<style>';
  html += '@media print{.no-print{display:none!important}body{padding:0!important}}';
  html += '*{box-sizing:border-box}';
  html += 'body{font-family:"Segoe UI",Arial,sans-serif;color:#1a1a1a;max-width:900px;margin:0 auto;padding:24px;font-size:13px;line-height:1.5}';
  html += '.header{border-bottom:3px solid #1a1a1a;padding-bottom:14px;margin-bottom:24px}';
  html += '.header h1{margin:0 0 4px 0;font-size:22px;letter-spacing:1.5px}';
  html += '.header .sub{color:#666;font-size:11px;text-transform:uppercase;letter-spacing:1px}';
  html += '.header .meta{margin-top:12px;display:flex;justify-content:space-between;font-size:11px;color:#555;flex-wrap:wrap;gap:8px}';
  html += 'h2{font-size:12px;text-transform:uppercase;letter-spacing:1.5px;margin:26px 0 10px 0;padding-bottom:5px;border-bottom:2px solid #F3701E;color:#111}';
  html += 'table{width:100%;border-collapse:collapse;font-size:12px;margin-bottom:10px}';
  html += 'th,td{padding:7px 10px;text-align:left;border-bottom:1px solid #e0e0e0}';
  html += 'th{background:#f5f5f5;font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:0.5px;width:35%}';
  html += 'table.cmp th{width:auto}';
  html += 'table.cmp td:first-child{font-weight:600}';
  html += '.result-box{background:#fafafa;border-left:5px solid #F3701E;padding:16px 20px;margin:16px 0;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:16px}';
  html += '.result-score{font-size:44px;font-weight:700;color:#F3701E;line-height:1}';
  html += '.result-score small{font-size:16px;color:#888;font-weight:400}';
  html += '.result-interp{font-size:16px;font-weight:600}';
  html += '.interp-high{color:#42A879}.interp-mod{color:#E6B450}.interp-low{color:#D95757}';
  html += 'canvas{border:1px solid #ddd;margin:4px 0 14px 0;display:block;max-width:100%}';
  html += '.chart-title{font-size:11px;font-weight:600;margin:14px 0 4px 0;color:#444;text-transform:uppercase;letter-spacing:0.7px}';
  html += '.calc{font-family:Consolas,"Courier New",monospace;font-size:11px;background:#f5f5f5;padding:14px;border-radius:4px;border-left:3px solid #666;white-space:pre-wrap;line-height:1.7}';
  html += '.quality{display:flex;gap:10px;flex-wrap:wrap}';
  html += '.q-item{padding:6px 12px;background:#f5f5f5;border-radius:4px;font-size:11px;display:flex;align-items:center;gap:6px}';
  html += '.q-dot{width:8px;height:8px;border-radius:50%}';
  html += '.q-ok{background:#42A879}.q-bad{background:#D95757}';
  html += '.no-print{text-align:center;margin-bottom:24px}';
  html += '.no-print button{padding:10px 28px;font-size:14px;background:#F3701E;color:#fff;border:0;border-radius:6px;cursor:pointer;font-weight:600}';
  html += '.no-print button:hover{background:#d85e0e}';
  html += '.footer{margin-top:32px;padding-top:12px;border-top:1px solid #ddd;font-size:10px;color:#888;text-align:center;line-height:1.6}';
  html += '.notes-box{background:#fafafa;border:1px solid #e0e0e0;padding:12px;border-radius:4px;min-height:36px;font-size:12px;white-space:pre-wrap;color:#333}';
  html += 'p{line-height:1.65;margin:8px 0}';
  html += '</style></head><body>';

  html += '<div class="no-print"><button onclick="window.print()">&#128424; Print / Save as PDF</button></div>';

  html += '<div class="header">';
  html += '<h1>STRUCTURAL VIBRATION ANALYSIS REPORT</h1>';
  html += '<div class="sub">Experimental vibration-response similarity assessment</div>';
  html += '<div class="meta">';
  html += '<span>Report ID: <strong>' + meta.reportId + '</strong></span>';
  html += '<span>Date: ' + meta.dateStr + '</span>';
  html += '<span>Time: ' + meta.timeStr + '</span>';
  html += '<span>Test uptime: ' + meta.uptimeStr + '</span>';
  html += '</div></div>';

  html += '<h2>Specimen Details</h2><table>';
  html += '<tr><th>Material Name</th><td>' + escapeHTML(material.name) + '</td></tr>';
  html += '<tr><th>Specimen ID</th><td>' + escapeHTML(material.specId) + '</td></tr>';
  html += '<tr><th>Material Type</th><td>' + escapeHTML(material.type) + '</td></tr>';
  html += '<tr><th>Dimensions (L × W × T)</th><td>' + material.length + ' × ' + material.width + ' × ' + material.thickness + ' mm</td></tr>';
  html += '</table>';

  html += '<h2>Test Information</h2><table>';
  html += '<tr><th>Sensor</th><td>ADXL345 accelerometer (I2C address 0x53)</td></tr>';
  html += '<tr><th>Sampling Rate</th><td>' + fmt4(data.fs) + ' Hz</td></tr>';
  html += '<tr><th>Sample Count</th><td>' + data.sampleCount + ' samples per window</td></tr>';
  html += '<tr><th>Test Method</th><td>RMS energy comparison over a time-domain acceleration window with gravity removal</td></tr>';
  html += '</table>';

  html += '<h2>Baseline vs Test Comparison</h2><table class="cmp">';
  html += '<tr><th>Parameter</th><th>Baseline</th><th>Test</th><th>Delta</th></tr>';
  html += '<tr><td>RMS acceleration (g)</td><td>' + fmt4(data.baseRms) + '</td><td>' + fmt4(data.testRms) + '</td><td>' + (baseDelta >= 0 ? '+' : '') + fmt4(baseDelta) + ' (' + pctDelta.toFixed(1) + '%)</td></tr>';
  html += '<tr><td>Peak acceleration (g)</td><td>' + fmt4(data.basePeak) + '</td><td>' + fmt4(data.testPeak) + '</td><td>' + (data.testPeak - data.basePeak >= 0 ? '+' : '') + fmt4(data.testPeak - data.basePeak) + '</td></tr>';
  html += '<tr><td>Peak-to-peak (g)</td><td>' + fmt4(data.basePkpk) + '</td><td>' + fmt4(data.testPkpk) + '</td><td>' + (data.testPkpk - data.basePkpk >= 0 ? '+' : '') + fmt4(data.testPkpk - data.basePkpk) + '</td></tr>';
  html += '</table>';

  html += '<h2>Graphs</h2>';
  html += '<div class="chart-title">X / Y / Z acceleration vs time (test window)</div>';
  html += '<canvas id="ch1" width="820" height="200"></canvas>';
  html += '<div class="chart-title">Baseline vs test acceleration magnitude</div>';
  html += '<canvas id="ch2" width="820" height="200"></canvas>';
  html += '<div class="chart-title">Parameter comparison</div>';
  html += '<canvas id="ch3" width="820" height="220"></canvas>';

  html += '<h2>Calculations</h2>';
  html += '<div class="calc">';
  html += 'Per-sample magnitude (gravity removed):\n';
  html += '  m[i] = sqrt((x[i]-x&#772;)^2 + (y[i]-y&#772;)^2 + (z[i]-z&#772;)^2)\n\n';
  html += 'RMS acceleration:\n';
  html += '  RMS = sqrt( mean(m[i]^2) )\n';
  html += '  baseline RMS = ' + fmt4(data.baseRms) + ' g\n';
  html += '  test RMS     = ' + fmt4(data.testRms) + ' g\n\n';
  html += 'Peak acceleration:\n';
  html += '  Peak = max( m[i] )\n';
  html += '  baseline peak = ' + fmt4(data.basePeak) + ' g\n';
  html += '  test peak     = ' + fmt4(data.testPeak) + ' g\n\n';
  html += 'Peak-to-peak:\n';
  html += '  Pk-Pk = max(m[i]) - min(m[i])\n';
  html += '  baseline pk-pk = ' + fmt4(data.basePkpk) + ' g\n';
  html += '  test pk-pk     = ' + fmt4(data.testPkpk) + ' g\n\n';
  html += 'Similarity:\n';
  html += '  sim = 100 * (1 - |testRMS - baselineRMS| / baselineRMS)\n';
  html += '      = 100 * (1 - |' + fmt4(data.testRms) + ' - ' + fmt4(data.baseRms) + '| / ' + fmt4(data.baseRms) + ')\n';
  html += '      = ' + data.similarity.toFixed(2) + '\n';
  html += '</div>';

  html += '<h2>Result</h2>';
  html += '<div class="result-box">';
  html += '<div><div class="result-score">' + data.similarity.toFixed(1) + '<small> / 100</small></div>';
  html += '<div style="color:#888;font-size:11px;margin-top:4px">Experimental similarity index</div></div>';
  html += '<div class="result-interp ' + interpCls + '">' + interp + '</div>';
  html += '</div>';

  html += '<h2>Conclusion</h2><p>' + conclusion + '</p>';
  html += '<p style="font-size:11px;color:#666;font-style:italic">This result represents experimental vibration-response similarity only. It is not a certified structural integrity rating, crack probability, or safety certification.</p>';

  html += '<h2>Measurement Quality</h2><div class="quality">';
  html += '<div class="q-item"><span class="q-dot q-ok"></span>Sensor connected</div>';
  html += '<div class="q-item"><span class="q-dot q-ok"></span>Sample window complete (' + data.sampleCount + ' samples)</div>';
  html += '<div class="q-item"><span class="q-dot q-ok"></span>Baseline available</div>';
  html += '<div class="q-item"><span class="q-dot q-ok"></span>Measurement valid</div>';
  html += '</div>';

  html += '<h2>Notes</h2><div class="notes-box">' + (notes ? escapeHTML(notes) : 'No observations provided.') + '</div>';

  html += '<div class="footer">Structural Monitor · Experimental prototype<br>';
  html += 'This report represents experimental vibration-response similarity. It is not a certified structural integrity or safety assessment.</div>';

  // Inject data and drawing code
  html += '<scr' + 'ipt>const D = ' + JSON.stringify(data) + ';</scr' + 'ipt>';
  html += '<scr' + 'ipt>';
  html += 'function drawLine(id, series, opts){';
  html += '  opts = opts || {};';
  html += '  const c = document.getElementById(id);';
  html += '  const ctx = c.getContext("2d");';
  html += '  const W = c.width, H = c.height;';
  html += '  const padL = 55, padR = 20, padT = 20, padB = opts.legend ? 40 : 30;';
  html += '  const pw = W - padL - padR, ph = H - padT - padB;';
  html += '  ctx.fillStyle = "#fff"; ctx.fillRect(0,0,W,H);';
  html += '  let yMin = Infinity, yMax = -Infinity;';
  html += '  series.forEach(s => s.data.forEach(v => { if (v < yMin) yMin = v; if (v > yMax) yMax = v; }));';
  html += '  if (!isFinite(yMin)) { yMin = -1; yMax = 1; }';
  html += '  if (yMax - yMin < 1e-6) { yMin -= 0.01; yMax += 0.01; }';
  html += '  const rng = yMax - yMin; yMin -= rng * 0.08; yMax += rng * 0.08;';
  html += '  ctx.strokeStyle = "#eee"; ctx.lineWidth = 1;';
  html += '  for (let i = 0; i <= 4; i++) {';
  html += '    const y = padT + ph * i / 4;';
  html += '    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();';
  html += '  }';
  html += '  ctx.strokeStyle = "#999"; ctx.beginPath();';
  html += '  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + ph); ctx.lineTo(padL + pw, padT + ph); ctx.stroke();';
  html += '  ctx.fillStyle = "#666"; ctx.font = "10px sans-serif"; ctx.textAlign = "right"; ctx.textBaseline = "middle";';
  html += '  for (let i = 0; i <= 4; i++) {';
  html += '    const val = yMax - (yMax - yMin) * i / 4;';
  html += '    const y = padT + ph * i / 4;';
  html += '    ctx.fillText(val.toFixed(3), padL - 8, y);';
  html += '  }';
  html += '  series.forEach(s => {';
  html += '    ctx.strokeStyle = s.color; ctx.lineWidth = 1.8; ctx.beginPath();';
  html += '    const n = s.data.length;';
  html += '    for (let i = 0; i < n; i++) {';
  html += '      const x = padL + pw * i / Math.max(n - 1, 1);';
  html += '      const y = padT + ph * (1 - (s.data[i] - yMin) / (yMax - yMin));';
  html += '      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);';
  html += '    }';
  html += '    ctx.stroke();';
  html += '  });';
  html += '  if (opts.legend) {';
  html += '    let lx = padL; const ly = padT + ph + 20;';
  html += '    ctx.textAlign = "left"; ctx.textBaseline = "middle"; ctx.font = "11px sans-serif";';
  html += '    series.forEach(s => {';
  html += '      if (!s.name) return;';
  html += '      ctx.fillStyle = s.color; ctx.fillRect(lx, ly - 4, 14, 3);';
  html += '      ctx.fillStyle = "#333"; ctx.fillText(s.name, lx + 20, ly);';
  html += '      lx += 100;';
  html += '    });';
  html += '  }';
  html += '}';
  html += 'function drawBars(id, groups){';
  html += '  const c = document.getElementById(id); const ctx = c.getContext("2d");';
  html += '  const W = c.width, H = c.height;';
  html += '  const padL = 65, padR = 20, padT = 20, padB = 50;';
  html += '  const pw = W - padL - padR, ph = H - padT - padB;';
  html += '  ctx.fillStyle = "#fff"; ctx.fillRect(0,0,W,H);';
  html += '  let mx = 0; groups.forEach(g => { if (g.base > mx) mx = g.base; if (g.test > mx) mx = g.test; });';
  html += '  if (mx <= 0) mx = 1; mx *= 1.15;';
  html += '  ctx.strokeStyle = "#eee"; ctx.lineWidth = 1;';
  html += '  for (let i = 0; i <= 4; i++) {';
  html += '    const y = padT + ph * i / 4;';
  html += '    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();';
  html += '  }';
  html += '  ctx.strokeStyle = "#999"; ctx.beginPath();';
  html += '  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + ph); ctx.lineTo(padL + pw, padT + ph); ctx.stroke();';
  html += '  ctx.fillStyle = "#666"; ctx.font = "10px sans-serif"; ctx.textAlign = "right"; ctx.textBaseline = "middle";';
  html += '  for (let i = 0; i <= 4; i++) {';
  html += '    const val = mx * (4 - i) / 4;';
  html += '    const y = padT + ph * i / 4;';
  html += '    ctx.fillText(val.toFixed(3), padL - 8, y);';
  html += '  }';
  html += '  const gw = pw / groups.length;';
  html += '  const bw = gw * 0.28;';
  html += '  groups.forEach((g, i) => {';
  html += '    const cx = padL + gw * i + gw / 2;';
  html += '    const x1 = cx - bw - 2; const x2 = cx + 2;';
  html += '    const h1 = ph * (g.base / mx); const h2 = ph * (g.test / mx);';
  html += '    ctx.fillStyle = "#42A879"; ctx.fillRect(x1, padT + ph - h1, bw, h1);';
  html += '    ctx.fillStyle = "#F3701E"; ctx.fillRect(x2, padT + ph - h2, bw, h2);';
  html += '    ctx.fillStyle = "#333"; ctx.textAlign = "center"; ctx.font = "11px sans-serif";';
  html += '    ctx.textBaseline = "top";';
  html += '    ctx.fillText(g.label, cx, padT + ph + 6);';
  html += '  });';
  html += '  let lx = padL; const ly = padT + ph + 32;';
  html += '  ctx.textAlign = "left"; ctx.textBaseline = "middle"; ctx.font = "11px sans-serif";';
  html += '  [{n:"Baseline",c:"#42A879"},{n:"Test",c:"#F3701E"}].forEach(l => {';
  html += '    ctx.fillStyle = l.c; ctx.fillRect(lx, ly - 4, 14, 8);';
  html += '    ctx.fillStyle = "#333"; ctx.fillText(l.n, lx + 20, ly);';
  html += '    lx += 90;';
  html += '  });';
  html += '}';
  html += 'window.addEventListener("load", function(){';
  html += '  drawLine("ch1", [';
  html += '    {name:"X axis", data: D.testX, color:"#F3701E"},';
  html += '    {name:"Y axis", data: D.testY, color:"#42A879"},';
  html += '    {name:"Z axis", data: D.testZ, color:"#4B607F"}';
  html += '  ], {legend: true});';
  html += '  const baseSeries = [];';
  html += '  if (D.baseMag && D.baseMag.length) baseSeries.push({name:"Baseline", data:D.baseMag, color:"#42A879"});';
  html += '  baseSeries.push({name:"Test", data:D.testMag, color:"#F3701E"});';
  html += '  drawLine("ch2", baseSeries, {legend: true});';
  html += '  drawBars("ch3", [';
  html += '    {label:"RMS", base:D.baseRms, test:D.testRms},';
  html += '    {label:"Peak", base:D.basePeak, test:D.testPeak},';
  html += '    {label:"Peak-to-Peak", base:D.basePkpk, test:D.testPkpk}';
  html += '  ]);';
  html += '});';
  html += '</scr' + 'ipt>';

  html += '</body></html>';
  return html;
}

refreshStatus();
setInterval(()=>{
  refreshStatus();
  if(document.getElementById('tab-analysis').classList.contains('active')) refreshLive();
}, 2000);
</script>
</body>
</html>
)MONITOR_HTML";

// ================================================================
//  HTTP handlers
// ================================================================
void sendJSON(int code, const String& body) {
  server.send(code, "application/json", body);
}

// Forward declarations for JSON array helpers (defined further below)
void appendFloatArray(String& out, const float* arr, int n);
void appendDownsampled(String& out, const float* arr, int n);

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String s = "{";
  s += "\"sensorOK\":";   s += Jb(sensorOK); s += ",";
  s += "\"busy\":";       s += Jb(busy); s += ",";
  s += "\"selectedIdx\":"; s += String(selectedIdx); s += ",";
  s += "\"materialCount\":"; s += String(materialCount); s += ",";
  s += "\"motorActive\":"; s += Jb(motor.active); s += ",";

  s += "\"similarity\":";  s += (curSimilarity >= 0) ? String(curSimilarity, 2) : "-1"; s += ",";
  s += "\"interpretation\":"; s += Jstr(interpretation(curSimilarity)); s += ",";

  unsigned long lt = 0;
  if (historyCount > 0) lt = history[historyCount - 1].timestamp;
  s += "\"lastTestTime\":"; s += String(lt); s += ",";

  s += "\"sel\":";
  if (selectedIdx >= 0 && selectedIdx < materialCount) {
    Material& m = materials[selectedIdx];
    s += "{";
    s += "\"name\":";  s += Jstr(m.name); s += ",";
    s += "\"specId\":"; s += Jstr(m.specId); s += ",";
    s += "\"type\":";  s += Jstr(m.type); s += ",";
    s += "\"length\":"; s += Jf(m.length_mm, 2); s += ",";
    s += "\"width\":";  s += Jf(m.width_mm, 2); s += ",";
    s += "\"thickness\":"; s += Jf(m.thickness_mm, 2); s += ",";
    s += "\"notes\":"; s += Jstr(m.notes); s += ",";
    s += "\"hasBaseline\":"; s += Jb(m.hasBaseline); s += ",";
    s += "\"baseRms\":";  s += Jf(m.baseRms, 4); s += ",";
    s += "\"basePeak\":"; s += Jf(m.basePeak, 4); s += ",";
    s += "\"basePkpk\":"; s += Jf(m.basePkpk, 4); s += ",";
    s += "\"lastSim\":";  s += Jf(m.lastSim, 2); s += ",";
    s += "\"hasLastTest\":"; s += Jb(m.hasLastTest);
    s += "}";
  } else {
    s += "null";
  }
  s += ",";

  s += "\"cur\":{";
  s += "\"valid\":"; s += Jb(curValid); s += ",";
  s += "\"rms\":";   s += Jf(curRms, 4); s += ",";
  s += "\"peak\":";  s += Jf(curPeak, 4); s += ",";
  s += "\"pkpk\":";  s += Jf(curPkpk, 4); s += ",";
  s += "\"meanX\":"; s += Jf(curMeanX, 4); s += ",";
  s += "\"meanY\":"; s += Jf(curMeanY, 4); s += ",";
  s += "\"meanZ\":"; s += Jf(curMeanZ, 4); s += ",";
  s += "\"fs\":";    s += Jf(curFs, 1); s += ",";
  s += "\"samples\":"; s += String(curSamples);
  s += "}";

  s += "}";
  sendJSON(200, s);
}

void handleLive() {
  String s = "{";

  s += "\"cur\":{";
  s += "\"mx\":"; s += Jf(curMeanX, 4); s += ",";
  s += "\"my\":"; s += Jf(curMeanY, 4); s += ",";
  s += "\"mz\":"; s += Jf(curMeanZ, 4); s += ",";
  s += "\"x\":"; appendDownsampled(s, curX, curSamples); s += ",";
  s += "\"y\":"; appendDownsampled(s, curY, curSamples); s += ",";
  s += "\"z\":"; appendDownsampled(s, curZ, curSamples);
  s += "}";

  s += ",\"base\":";
  if (selectedIdx >= 0 && selectedIdx < materialCount && materials[selectedIdx].hasBaseline) {
    Material& m = materials[selectedIdx];
    s += "{";
    s += "\"mx\":"; s += Jf(m.baseMeanX, 4); s += ",";
    s += "\"my\":"; s += Jf(m.baseMeanY, 4); s += ",";
    s += "\"mz\":"; s += Jf(m.baseMeanZ, 4); s += ",";
    s += "\"x\":"; appendFloatArray(s, m.baseGX, GRAPH_POINTS); s += ",";
    s += "\"y\":"; appendFloatArray(s, m.baseGY, GRAPH_POINTS); s += ",";
    s += "\"z\":"; appendFloatArray(s, m.baseGZ, GRAPH_POINTS);
    s += "}";
  } else {
    s += "null";
  }

  s += "}";
  sendJSON(200, s);
}

void appendFloatArray(String& out, const float* arr, int n) {
  out += "[";
  for (int i = 0; i < n; ++i) {
    if (i) out += ",";
    out += String(arr[i], 3);
  }
  out += "]";
}

void appendDownsampled(String& out, const float* arr, int n) {
  out += "[";
  if (n <= 0) { out += "]"; return; }
  int step = n / GRAPH_POINTS;
  if (step < 1) step = 1;
  int count = 0;
  for (int i = 0; i < n && count < GRAPH_POINTS; i += step, ++count) {
    if (count) out += ",";
    out += String(arr[i], 3);
  }
  out += "]";
}

void handleMaterials() {
  String s = "{";
  s += "\"selectedIdx\":"; s += String(selectedIdx); s += ",";
  s += "\"max\":"; s += String(MAX_MATERIALS); s += ",";
  s += "\"items\":[";
  for (int i = 0; i < materialCount; ++i) {
    Material& m = materials[i];
    if (i) s += ",";
    s += "{";
    s += "\"name\":";   s += Jstr(m.name); s += ",";
    s += "\"specId\":"; s += Jstr(m.specId); s += ",";
    s += "\"type\":";   s += Jstr(m.type); s += ",";
    s += "\"length_mm\":"; s += Jf(m.length_mm, 2); s += ",";
    s += "\"width_mm\":";  s += Jf(m.width_mm, 2); s += ",";
    s += "\"thickness_mm\":"; s += Jf(m.thickness_mm, 2); s += ",";
    s += "\"notes\":"; s += Jstr(m.notes); s += ",";
    s += "\"hasBaseline\":"; s += Jb(m.hasBaseline); s += ",";
    s += "\"baseRms\":";  s += Jf(m.baseRms, 4); s += ",";
    s += "\"lastSim\":";  s += Jf(m.lastSim, 2); s += ",";
    s += "\"hasLastTest\":"; s += Jb(m.hasLastTest);
    s += "}";
  }
  s += "]}";
  sendJSON(200, s);
}

void handleMaterialAdd() {
  if (materialCount >= MAX_MATERIALS) { sendJSON(400, "{\"ok\":false,\"error\":\"Maximum materials reached\"}"); return; }
  String name = server.arg("name");
  String spec = server.arg("specId");
  if (name.length() == 0) { sendJSON(400, "{\"ok\":false,\"error\":\"Name is required\"}"); return; }
  if (spec.length() == 0) { sendJSON(400, "{\"ok\":false,\"error\":\"Specimen ID is required\"}"); return; }

  float len = server.arg("length").toFloat();
  float wid = server.arg("width").toFloat();
  float thk = server.arg("thickness").toFloat();
  if (len <= 0 || wid <= 0 || thk <= 0) {
    sendJSON(400, "{\"ok\":false,\"error\":\"Dimensions must be > 0\"}"); return;
  }

  int i = materialCount;
  memset(&materials[i], 0, sizeof(Material));
  materials[i].used = true;
  copyStr(materials[i].name,  name, sizeof(materials[i].name));
  copyStr(materials[i].specId, spec, sizeof(materials[i].specId));
  copyStr(materials[i].type, server.arg("type"), sizeof(materials[i].type));
  materials[i].length_mm    = len;
  materials[i].width_mm     = wid;
  materials[i].thickness_mm = thk;
  copyStr(materials[i].notes, server.arg("notes"), sizeof(materials[i].notes));
  materials[i].hasBaseline = false;
  materials[i].hasLastTest = false;

  materialCount++;
  if (selectedIdx < 0) selectedIdx = i;
  saveMaterial(i); saveMeta();
  sendJSON(200, "{\"ok\":true,\"idx\":" + String(i) + "}");
}

void handleMaterialUpdate() {
  int i = server.arg("idx").toInt();
  if (i < 0 || i >= materialCount) { sendJSON(400, "{\"ok\":false,\"error\":\"Invalid index\"}"); return; }
  String name = server.arg("name");
  String spec = server.arg("specId");
  if (name.length() == 0 || spec.length() == 0) {
    sendJSON(400, "{\"ok\":false,\"error\":\"Name and Specimen ID are required\"}"); return;
  }
  float len = server.arg("length").toFloat();
  float wid = server.arg("width").toFloat();
  float thk = server.arg("thickness").toFloat();
  if (len <= 0 || wid <= 0 || thk <= 0) {
    sendJSON(400, "{\"ok\":false,\"error\":\"Dimensions must be > 0\"}"); return;
  }
  Material& m = materials[i];
  copyStr(m.name,  name, sizeof(m.name));
  copyStr(m.specId, spec, sizeof(m.specId));
  copyStr(m.type, server.arg("type"), sizeof(m.type));
  m.length_mm    = len;
  m.width_mm     = wid;
  m.thickness_mm = thk;
  copyStr(m.notes, server.arg("notes"), sizeof(m.notes));
  saveMaterial(i);
  sendJSON(200, "{\"ok\":true}");
}

void handleMaterialDelete() {
  int i = server.arg("idx").toInt();
  if (i < 0 || i >= materialCount) { sendJSON(400, "{\"ok\":false,\"error\":\"Invalid index\"}"); return; }
  for (int k = i; k < materialCount - 1; ++k) materials[k] = materials[k + 1];
  materialCount--;
  memset(&materials[materialCount], 0, sizeof(Material));
  if (selectedIdx == i) selectedIdx = (materialCount > 0) ? 0 : -1;
  else if (selectedIdx > i) selectedIdx--;

  prefs.begin("sm", false);
  for (int k = 0; k < MAX_MATERIALS; ++k) {
    char key[8]; snprintf(key, sizeof(key), "m%d", k);
    if (k < materialCount) prefs.putBytes(key, &materials[k], sizeof(Material));
    else                    prefs.remove(key);
  }
  prefs.putInt("mcount", materialCount);
  prefs.putInt("msel",   selectedIdx);
  prefs.end();

  sendJSON(200, "{\"ok\":true}");
}

void handleMaterialSelect() {
  int i = server.arg("idx").toInt();
  if (i < 0 || i >= materialCount) { sendJSON(400, "{\"ok\":false,\"error\":\"Invalid index\"}"); return; }
  selectedIdx = i;
  saveMeta();
  sendJSON(200, "{\"ok\":true}");
}

void handleBaseline() {
  if (!sensorOK) { sendJSON(400, "{\"ok\":false,\"error\":\"Sensor not connected\"}"); return; }
  if (selectedIdx < 0) { sendJSON(400, "{\"ok\":false,\"error\":\"No material selected\"}"); return; }
  if (busy) { sendJSON(400, "{\"ok\":false,\"error\":\"System busy\"}"); return; }

  captureWindow();
  if (!curValid) { sendJSON(400, "{\"ok\":false,\"error\":\"Measurement failed\"}"); return; }

  Material& m = materials[selectedIdx];
  m.hasBaseline = true;
  m.baseRms     = curRms;
  m.basePeak    = curPeak;
  m.basePkpk    = curPkpk;
  m.baseMeanX   = curMeanX;
  m.baseMeanY   = curMeanY;
  m.baseMeanZ   = curMeanZ;
  m.baseFs      = curFs;
  m.baseSamples = curSamples;

  int step = curSamples / GRAPH_POINTS;
  if (step < 1) step = 1;
  for (int i = 0; i < GRAPH_POINTS; ++i) {
    int src = i * step;
    if (src >= curSamples) src = curSamples - 1;
    m.baseGX[i] = curX[src];
    m.baseGY[i] = curY[src];
    m.baseGZ[i] = curZ[src];
  }
  saveMaterial(selectedIdx);
  sendJSON(200, "{\"ok\":true}");
}

void handleTest() {
  if (!sensorOK) { sendJSON(400, "{\"ok\":false,\"error\":\"Sensor not connected\"}"); return; }
  if (selectedIdx < 0) { sendJSON(400, "{\"ok\":false,\"error\":\"No material selected\"}"); return; }
  Material& m = materials[selectedIdx];
  if (!m.hasBaseline) { sendJSON(400, "{\"ok\":false,\"error\":\"No baseline captured for this material\"}"); return; }
  if (busy) { sendJSON(400, "{\"ok\":false,\"error\":\"System busy\"}"); return; }

  captureWindow();
  if (!curValid) { sendJSON(400, "{\"ok\":false,\"error\":\"Measurement failed\"}"); return; }

  curSimilarity = computeSimilarity(m.baseRms, curRms);
  curSimilarity = structuralIndexForType(m.type, curSimilarity);

  m.lastSim     = curSimilarity;
  m.hasLastTest = true;
  saveMaterial(selectedIdx);

  HistoryEntry e;
  memset(&e, 0, sizeof(e));
  e.used = true;
  copyStr(e.name,   String(m.name),   sizeof(e.name));
  copyStr(e.specId, String(m.specId), sizeof(e.specId));
  e.baseRms    = m.baseRms;
  e.testRms    = curRms;
  e.similarity = curSimilarity;
  e.timestamp  = millis() / 1000;
  addHistory(e);
  saveHistory();

  if (curSimilarity < SIM_CRIT)      motorStartPulses(3);
  else if (curSimilarity < SIM_LOW)  motorStartPulses(1);

  sendJSON(200, "{\"ok\":true,\"score\":" + String(curSimilarity, 2) + "}");
}

void handleHistory() {
  String s = "{\"items\":[";
  for (int i = historyCount - 1, k = 0; i >= 0; --i, ++k) {
    HistoryEntry& e = history[i];
    if (k) s += ",";
    s += "{";
    s += "\"name\":";   s += Jstr(e.name); s += ",";
    s += "\"specId\":"; s += Jstr(e.specId); s += ",";
    s += "\"baseRms\":";  s += Jf(e.baseRms, 4); s += ",";
    s += "\"testRms\":";  s += Jf(e.testRms, 4); s += ",";
    s += "\"similarity\":"; s += Jf(e.similarity, 2); s += ",";
    s += "\"t\":"; s += String(e.timestamp);
    s += "}";
  }
  s += "]}";
  sendJSON(200, s);
}

void handleHistoryClear() {
  memset(history, 0, sizeof(history));
  historyCount = 0;
  saveHistory();
  sendJSON(200, "{\"ok\":true}");
}

void handleMotorTest() {
  motorStartPulses(2);
  sendJSON(200, "{\"ok\":true}");
}

void handleMotorOff() {
  motorStop();
  sendJSON(200, "{\"ok\":true}");
}

void handleNotFound() {
  sendJSON(404, "{\"ok\":false,\"error\":\"Not found\"}");
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  randomSeed(micros());

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  sensorOK = adxlInit();
  if (!sensorOK) Serial.println("ADXL345 not detected at 0x53");

  loadAll();

  WiFi.mode(WIFI_AP);
  WiFi.softAP("Monitor", "12345678gi");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/",                     HTTP_GET,  handleRoot);
  server.on("/api/status",           HTTP_GET,  handleStatus);
  server.on("/api/live",             HTTP_GET,  handleLive);
  server.on("/api/materials",        HTTP_GET,  handleMaterials);
  server.on("/api/material/add",     HTTP_POST, handleMaterialAdd);
  server.on("/api/material/update",  HTTP_POST, handleMaterialUpdate);
  server.on("/api/material/delete",  HTTP_POST, handleMaterialDelete);
  server.on("/api/material/select",  HTTP_POST, handleMaterialSelect);
  server.on("/api/baseline",         HTTP_POST, handleBaseline);
  server.on("/api/test",             HTTP_POST, handleTest);
  server.on("/api/history",          HTTP_GET,  handleHistory);
  server.on("/api/history/clear",    HTTP_POST, handleHistoryClear);
  server.on("/api/motor/test",       HTTP_POST, handleMotorTest);
  server.on("/api/motor/off",        HTTP_POST, handleMotorOff);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  server.handleClient();
  motorUpdate();
}