/*
 Taupunktlüfter (ESP8266, Arduino IDE) - SPA + TZ string + DST handling

 Features:
 - DHT22 (indoor) + SHT40 (outside) + I2C LCD
 - Relay logic with dew point and hysteresis
 - LittleFS logging (taupunkt.csv)
 - Config stored in EEPROM (JSON) including TZ string (with DST rules)
 - STA/AP logic (AP for initial setup when wifi_pass empty)
 - Captive redirect in AP mode
 - SPA web UI: live AJAX status + live charts + config form + CSV download + OTA
 - MQTT with optional TLS (insecure by default)
 - Factory reset by holding USR (D3) low for 10s at boot
*/

#include <Arduino.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_SHT4x.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <WiFiClientSecure.h>

// ---------- Pin and hardware config ----------
#define RELAY_PIN        D6
#define RELAY_ACTIVE_LOW true
#define DHT_PIN          D7
#define DHT_TYPE         DHT22
#define LED_PIN          LED_BUILTIN
#define USR_PIN          D3

#define LCD_I2C_ADDR     0x27
#define LCD_COLS         16
#define LCD_ROWS         2

// EEPROM config area size (bytes)
#define EEPROM_SIZE      4096
#define CFG_EEPROM_ADDR  0

// ---------- Timing ----------
const unsigned long DISPLAY_INTERVAL_MS = 2000UL;
const unsigned long MEASURE_INTERVAL_MS = 3000UL;
const unsigned long LOG_INTERVAL_MS     = 10UL * 60UL * 1000UL; // 10 min

// sensor corrections
const float KORREKTUR_T_1 = -2.0;
const float KORREKTUR_T_2 = -1.0;
const float KORREKTUR_H_1 = 0.0;
const float KORREKTUR_H_2 = -1.0;

// control parameters
const float SCHALTmin = 0.2;
const float HYSTERESE = 3.0;
const float TEMP1_min = 10.0;
const float TEMP2_min = -10.0;

// storage filenames
const char *LOG_FILENAME = "/taupunkt.csv";

// ---------- Globals ----------
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

volatile float h1 = 0.0, t1 = 0.0, h2 = 0.0, t2 = 0.0;
bool rel = false;
bool firstRun = true;

String logBuffer = "";
const int MAXLINES = 144;

unsigned long lastMeasure = 0, lastDisplay = 0, lastLog = 0;

// Web & networking
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient wifiClient;
WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient; // will set client later

// small circular MQTT log buffer
#define MQTT_LOG_LINES 12
String mqttLogs[MQTT_LOG_LINES];
int mqttLogIdx = 0;

// ---------- Config stored in EEPROM ----------
struct Config {
  String wifi_ssid;
  String wifi_pass;
  String ntp_host;
  String tz; // TZ string, e.g. "CET-1CEST,M3.5.0/2,M10.5.0/3"
  String ota_user;
  String ota_pass;
  bool mqtt_enabled;
  bool mqtt_use_ssl;
  String mqtt_host;
  uint16_t mqtt_port;
  String mqtt_user;
  String mqtt_pass;
  String mqtt_topic;
} cfg;

void defaultConfig() {
  cfg.wifi_ssid = "";
  cfg.wifi_pass = "";
  cfg.ntp_host = "192.168.178.1";
  cfg.tz = "CET-1CEST,M3.5.0/2,M10.5.0/3";
  cfg.ota_user = "admin";
  cfg.ota_pass = "admin";
  cfg.mqtt_enabled = false;
  cfg.mqtt_use_ssl = false;
  cfg.mqtt_host = "";
  cfg.mqtt_port = 1883;
  cfg.mqtt_user = "";
  cfg.mqtt_pass = "";
  cfg.mqtt_topic = "taupunkt";
}

// ---------- Forward declarations ----------
void startAP();
void startSTA();
void startWebServices();
void handleRoot();
void handleSaveConfig();
void handleDownload();
void handleNotFound();
void handleStatusJSON();
void loadConfig();
void saveConfig();
void eraseConfigEEPROM();
void ensureLogHeader();
void flushLogBufferToFS();
float taupunkt(float t, float r);
void doMeasure();
void doDisplay();
void doLog(bool force=false);
void setupMQTT();
void mqttReconnect();
void publishMeasurementMQTT();
void addMqttLog(const String &l);
void applyTZ(); // apply cfg.tz to environment

// ---------- Helpers ----------
float taupunkt(float t, float r) {
  float a,b;
  if (t >= 0.0) { a = 7.5; b = 237.3; }
  else { a = 7.6; b = 240.7; }
  double sdd = 6.1078 * pow(10.0, (a * t) / (b + t));
  double dd = sdd * (r / 100.0);
  double v = log10(dd / 6.1078);
  double tt = (b * v) / (a - v);
  return (float)tt;
}

String pt() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  if (localtime_r(&now, &timeinfo) == nullptr) {
    unsigned long s = (millis()/1000)%60;
    unsigned long m = (millis()/60000)%60;
    unsigned long h = (millis()/3600000)%24;
    char buf[32];
    snprintf(buf, sizeof(buf), "00.00.1970 %02lu:%02lu:%02lu", h, m, s);
    return String(buf);
  } else {
    char buf[32];
    snprintf(buf, sizeof(buf), "%02d.%02d.%04d %02d:%02d:%02d",
             timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return String(buf);
  }
}

// EEPROM JSON save/load
void loadConfig() {
  defaultConfig();
  if (EEPROM.read(CFG_EEPROM_ADDR) != 0xA5) {
    Serial.println("No config in EEPROM");
    return;
  }
  uint16_t len = (uint16_t)EEPROM.read(CFG_EEPROM_ADDR+1) << 8 | (uint16_t)EEPROM.read(CFG_EEPROM_ADDR+2);
  if (len == 0 || len > (EEPROM_SIZE - 16)) {
    Serial.println("Invalid config length in EEPROM");
    return;
  }
  std::unique_ptr<char[]> buf(new char[len+1]);
  for (uint16_t i=0; i<len; ++i) buf[i] = (char)EEPROM.read(CFG_EEPROM_ADDR + 3 + i);
  buf[len]=0;
  StaticJsonDocument<1536> doc;
  auto err = deserializeJson(doc, buf.get());
  if (err) {
    Serial.println("Failed to parse JSON from EEPROM");
    return;
  }
  if (doc.containsKey("wifi_ssid")) cfg.wifi_ssid = String((const char*)doc["wifi_ssid"]);
  if (doc.containsKey("wifi_pass")) cfg.wifi_pass = String((const char*)doc["wifi_pass"]);
  if (doc.containsKey("ntp_host")) cfg.ntp_host = String((const char*)doc["ntp_host"]);
  if (doc.containsKey("tz")) cfg.tz = String((const char*)doc["tz"]);
  if (doc.containsKey("ota_user")) cfg.ota_user = String((const char*)doc["ota_user"]);
  if (doc.containsKey("ota_pass")) cfg.ota_pass = String((const char*)doc["ota_pass"]);
  if (doc.containsKey("mqtt_enabled")) cfg.mqtt_enabled = doc["mqtt_enabled"];
  if (doc.containsKey("mqtt_use_ssl")) cfg.mqtt_use_ssl = doc["mqtt_use_ssl"];
  if (doc.containsKey("mqtt_host")) cfg.mqtt_host = String((const char*)doc["mqtt_host"]);
  if (doc.containsKey("mqtt_port")) cfg.mqtt_port = doc["mqtt_port"];
  if (doc.containsKey("mqtt_user")) cfg.mqtt_user = String((const char*)doc["mqtt_user"]);
  if (doc.containsKey("mqtt_pass")) cfg.mqtt_pass = String((const char*)doc["mqtt_pass"]);
  if (doc.containsKey("mqtt_topic")) cfg.mqtt_topic = String((const char*)doc["mqtt_topic"]);
  Serial.println("Config loaded from EEPROM");
}

void saveConfig() {
  StaticJsonDocument<1536> doc;
  doc["wifi_ssid"] = cfg.wifi_ssid;
  doc["wifi_pass"] = cfg.wifi_pass;
  doc["ntp_host"] = cfg.ntp_host;
  doc["tz"] = cfg.tz;
  doc["ota_user"] = cfg.ota_user;
  doc["ota_pass"] = cfg.ota_pass;
  doc["mqtt_enabled"] = cfg.mqtt_enabled;
  doc["mqtt_use_ssl"] = cfg.mqtt_use_ssl;
  doc["mqtt_host"] = cfg.mqtt_host;
  doc["mqtt_port"] = cfg.mqtt_port;
  doc["mqtt_user"] = cfg.mqtt_user;
  doc["mqtt_pass"] = cfg.mqtt_pass;
  doc["mqtt_topic"] = cfg.mqtt_topic;
  String out;
  serializeJson(doc, out);
  uint16_t len = out.length();
  if (len + 16 > EEPROM_SIZE) {
    Serial.println("Config too large for EEPROM");
    return;
  }
  EEPROM.write(CFG_EEPROM_ADDR, 0xA5);
  EEPROM.write(CFG_EEPROM_ADDR+1, (uint8_t)(len >> 8));
  EEPROM.write(CFG_EEPROM_ADDR+2, (uint8_t)(len & 0xFF));
  for (uint16_t i=0;i<len;++i) EEPROM.write(CFG_EEPROM_ADDR+3+i, out[i]);
  EEPROM.commit();
  Serial.println("Config saved to EEPROM");
}

void eraseConfigEEPROM() {
  EEPROM.write(CFG_EEPROM_ADDR, 0xFF);
  for (int i=1;i<128;i++) EEPROM.write(CFG_EEPROM_ADDR+i, 0);
  EEPROM.commit();
  Serial.println("EEPROM config erased");
}

// ---------- Logging ----------
void ensureLogHeader() {
  if (!LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "w");
    if (f) { f.println("Date,t1,h1,tp1,t2,h2,tp2,fan"); f.close(); }
  }
}

void flushLogBufferToFS() {
  if (logBuffer.length() == 0) return;
  File f = LittleFS.open(LOG_FILENAME, "a");
  if (!f) { Serial.println("Failed to open log file for append"); return; }
  f.print(logBuffer);
  f.close();
  logBuffer = "";
  Serial.println("Logbuffer flushed");
}

// ---------- small MQTT log buffer ----------
void addMqttLog(const String &l) {
  mqttLogIdx = (mqttLogIdx + MQTT_LOG_LINES - 1) % MQTT_LOG_LINES;
  mqttLogs[mqttLogIdx] = String(millis()/1000) + "s: " + l;
}

// ---------- Web handlers (SPA + endpoints) ----------
const char SPA_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Taupunktlüfter</title>
<style>
body{font-family:Inter,Arial,Helvetica,sans-serif;margin:12px;background:#f7f9fc;color:#111}
.header{display:flex;align-items:center;justify-content:space-between}
.box{background:#fff;padding:12px;border-radius:8px;box-shadow:0 1px 4px rgba(0,0,0,0.08);margin-bottom:12px}
.row{display:flex;gap:12px}
.col{flex:1}
h1{margin:0 0 8px 0;font-size:18px}
label{display:block;font-size:13px;margin-top:8px}
input,select,textarea{width:100%;padding:8px;margin-top:4px;border:1px solid #ddd;border-radius:6px}
button{padding:8px 12px;border:none;background:#007bff;color:white;border-radius:6px;cursor:pointer}
small{color:#666}
canvas{width:100%;height:140px}
.status-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.status-item{background:#f3f6fb;padding:8px;border-radius:6px;text-align:center}
</style>
</head>
<body>
<div class="header">
  <div><h1>Taupunktlüfter</h1><small id="ipinfo"></small></div>
  <div><button onclick="downloadCsv()">Download CSV</button> <button onclick="openUpdate()">OTA</button></div>
</div>

<div class="box">
  <div class="status-grid">
    <div class="status-item"><div>Indoor T</div><div id="t1" style="font-size:20px">--</div></div>
    <div class="status-item"><div>Outdoor T</div><div id="t2" style="font-size:20px">--</div></div>
    <div class="status-item"><div>Fan</div><div id="fan" style="font-size:20px">--</div></div>
    <div class="status-item"><div>Indoor H</div><div id="h1" style="font-size:20px">--</div></div>
    <div class="status-item"><div>Outdoor H</div><div id="h2" style="font-size:20px">--</div></div>
    <div class="status-item"><div>Time</div><div id="time" style="font-size:12px">--</div></div>
  </div>
</div>

<div class="box row">
  <div class="col">
    <h3>Temperature & Dewpoint</h3>
    <canvas id="chartTemp"></canvas>
  </div>
  <div class="col">
    <h3>Recent</h3>
    <pre id="recent" style="height:180px;overflow:auto;background:#f8fbff;padding:8px;border-radius:6px">--</pre>
  </div>
</div>

<div class="box">
  <h3>Config</h3>
  <form id="cfgForm" onsubmit="saveCfg(event)">
    <label>Wi-Fi SSID<input id="ssid" name="ssid"></label>
    <label>Wi-Fi Password (leave empty to keep or force AP on next boot)<input id="pass" name="pass" type="password"></label>
    <label>NTP Host<input id="ntp" name="ntp"></label>
    <label>Timezone (TZ string, e.g. CET-1CEST,M3.5.0/2,M10.5.0/3)<input id="tz" name="tz"></label>
    <label>OTA User<input id="ota_user" name="ota_user"></label>
    <label>OTA Password (leave empty to keep)<input id="ota_pass" name="ota_pass" type="password"></label>
    <label><input id="mqtt_enabled" type="checkbox" name="mqtt_enabled"> Enable MQTT</label>
    <label>MQTT TLS <input id="mqtt_use_ssl" type="checkbox" name="mqtt_use_ssl"></label>
    <label>MQTT Host<input id="mqtt_host" name="mqtt_host"></label>
    <label>MQTT Port<input id="mqtt_port" name="mqtt_port" type="number"></label>
    <label>MQTT User<input id="mqtt_user" name="mqtt_user"></label>
    <label>MQTT Pass (leave empty to keep)<input id="mqtt_pass" name="mqtt_pass" type="password"></label>
    <label>MQTT Topic<input id="mqtt_topic" name="mqtt_topic"></label>
    <button type="submit">Save and Reboot</button>
  </form>
</div>

<script>
let samples = { t1:[], t2:[], tp1:[], tp2:[], time:[] };
const maxSamples = 60;
function fetchStatus(){
  fetch('/status.json').then(r=>r.json()).then(j=>{
    document.getElementById('t1').innerText = j.t1.toFixed(2)+' °C';
    document.getElementById('t2').innerText = j.t2.toFixed(2)+' °C';
    document.getElementById('h1').innerText = j.h1.toFixed(1)+' %';
    document.getElementById('h2').innerText = j.h2.toFixed(1)+' %';
    document.getElementById('fan').innerText = j.fan ? 'ON' : 'OFF';
    document.getElementById('time').innerText = j.time;
    document.getElementById('ipinfo').innerText = j.ip;
    // add samples
    samples.t1.push(j.t1); samples.t2.push(j.t2); samples.tp1.push(j.tp1); samples.tp2.push(j.tp2);
    samples.time.push(j.time);
    if (samples.t1.length > maxSamples) {
      for (let k in samples) samples[k].shift();
    }
    drawCharts();
    // recent area: show last few CSV lines if available via status.log_preview
    if (j.log_preview) {
      document.getElementById('recent').innerText = j.log_preview;
    }
  }).catch(e=>{
    console.log('status fetch err', e);
  });
}
function drawCharts(){
  let c = document.getElementById('chartTemp');
  let ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  // size handling
  c.width = c.clientWidth; c.height = 140;
  function drawLine(arr, color){
    if (arr.length < 2) return;
    // find min/max for scale
    let vmin = Math.min(...arr), vmax = Math.max(...arr);
    if (vmin==vmax){ vmin -= 0.5; vmax += 0.5; }
    let range = vmax - vmin;
    ctx.beginPath();
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    for (let i=0;i<arr.length;i++){
      let x = (i/(arr.length-1))*c.width;
      let y = c.height - ((arr[i]-vmin)/range)*c.height;
      if (i==0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  // draw grid
  ctx.strokeStyle = '#eee'; ctx.lineWidth = 1;
  for (let i=0;i<4;i++){ ctx.beginPath(); ctx.moveTo(0,i*(c.height/4)); ctx.lineTo(c.width,i*(c.height/4)); ctx.stroke(); }
  drawLine(samples.t1, '#d9534f'); // indoor temp red
  drawLine(samples.t2, '#337ab7'); // outdoor temp blue
  drawLine(samples.tp1, '#f0ad4e'); // dewpoint orange
}
function loadConfig(){
  fetch('/status.json').then(r=>r.json()).then(j=>{
    document.getElementById('ssid').value = j.cfg_wifi_ssid || '';
    document.getElementById('ntp').value = j.cfg_ntp || '';
    document.getElementById('tz').value = j.cfg_tz || '';
    document.getElementById('ota_user').value = j.cfg_ota_user || '';
    document.getElementById('mqtt_enabled').checked = j.cfg_mqtt_enabled||false;
    document.getElementById('mqtt_use_ssl').checked = j.cfg_mqtt_use_ssl||false;
    document.getElementById('mqtt_host').value = j.cfg_mqtt_host || '';
    document.getElementById('mqtt_port').value = j.cfg_mqtt_port || '';
    document.getElementById('mqtt_user').value = j.cfg_mqtt_user || '';
    document.getElementById('mqtt_topic').value = j.cfg_mqtt_topic || '';
  });
}
function saveCfg(ev){
  ev.preventDefault();
  let form = document.getElementById('cfgForm');
  let data = new URLSearchParams(new FormData(form));
  fetch('/saveconfig', { method:'POST', body: data }).then(r=>r.text()).then(t=>{
    alert('Saved. Device will reboot.');
  }).catch(e=>{ alert('Save error'); });
}
function downloadCsv(){ window.location = '/download'; }
function openUpdate(){ window.location = '/update'; }
setInterval(fetchStatus,2000);
fetchStatus(); loadConfig();
</script>
</body>
</html>
)rawliteral";

// ---------- Web handlers ----------
void handleRoot() {
  server.sendHeader("Cache-Control","no-cache, no-store, must-revalidate");
  server.send_P(200, "text/html", SPA_HTML);
}

void handleSaveConfig() {
  if (server.hasArg("ssid")) cfg.wifi_ssid = server.arg("ssid");
  if (server.hasArg("pass") && server.arg("pass").length() > 0) cfg.wifi_pass = server.arg("pass");
  if (server.hasArg("ntp")) cfg.ntp_host = server.arg("ntp");
  if (server.hasArg("tz")) cfg.tz = server.arg("tz");
  if (server.hasArg("ota_user")) cfg.ota_user = server.arg("ota_user");
  if (server.hasArg("ota_pass") && server.arg("ota_pass").length() > 0) cfg.ota_pass = server.arg("ota_pass");
  cfg.mqtt_enabled = server.hasArg("mqtt_enabled");
  cfg.mqtt_use_ssl = server.hasArg("mqtt_use_ssl");
  if (server.hasArg("mqtt_host")) cfg.mqtt_host = server.arg("mqtt_host");
  if (server.hasArg("mqtt_port")) cfg.mqtt_port = server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user")) cfg.mqtt_user = server.arg("mqtt_user");
  if (server.hasArg("mqtt_pass") && server.arg("mqtt_pass").length() > 0) cfg.mqtt_pass = server.arg("mqtt_pass");
  if (server.hasArg("mqtt_topic")) cfg.mqtt_topic = server.arg("mqtt_topic");

  saveConfig();
  server.send(200, "text/plain", "OK");
  delay(800);
  ESP.restart();
}

void handleDownload() {
  if (!LittleFS.exists(LOG_FILENAME)) {
    server.send(404, "text/plain", "No log file");
    return;
  }
  File f = LittleFS.open(LOG_FILENAME, "r");
  server.sendHeader("Content-Disposition", "attachment; filename=taupunkt.csv");
  server.streamFile(f, "text/csv");
  f.close();
}

void handleNotFound() {
  if (WiFi.getMode() == WIFI_AP) {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "Not Found");
}

void handleStatusJSON() {
  StaticJsonDocument<512> doc;
  doc["t1"] = t1;
  doc["h1"] = h1;
  doc["tp1"] = taupunkt(t1,h1);
  doc["t2"] = t2;
  doc["h2"] = h2;
  doc["tp2"] = taupunkt(t2,h2);
  doc["fan"] = rel ? 1 : 0;
  doc["wifi_connected"] = (WiFi.status()==WL_CONNECTED);
  doc["ip"] = (WiFi.status()==WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  // log preview: last 10 lines from file
  String preview;
  if (LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "r");
    if (f) {
      int lines = 0;
      int maxlines = 10;
      // read backwards
      std::vector<String> tail;
      while (f.available()) {
        String line = f.readStringUntil('\n');
        if (line.length()) {
          tail.push_back(line);
          if ((int)tail.size() > maxlines) tail.erase(tail.begin());
        }
      }
      for (auto &ln : tail) { preview += ln + "\n"; }
      f.close();
    }
  }
  doc["log_preview"] = preview;
  // include current config (for SPA form population)
  doc["cfg_wifi_ssid"] = cfg.wifi_ssid;
  doc["cfg_ntp"] = cfg.ntp_host;
  doc["cfg_tz"] = cfg.tz;
  doc["cfg_ota_user"] = cfg.ota_user;
  doc["cfg_mqtt_enabled"] = cfg.mqtt_enabled;
  doc["cfg_mqtt_use_ssl"] = cfg.mqtt_use_ssl;
  doc["cfg_mqtt_host"] = cfg.mqtt_host;
  doc["cfg_mqtt_port"] = cfg.mqtt_port;
  doc["cfg_mqtt_user"] = cfg.mqtt_user;
  doc["cfg_mqtt_topic"] = cfg.mqtt_topic;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ---------- WiFi/AP / web service ----------
void startWebServices() {
  server.on("/", handleRoot);
  server.on("/saveconfig", HTTP_POST, handleSaveConfig);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/status.json", HTTP_GET, handleStatusJSON);
  server.onNotFound(handleNotFound);
  httpUpdater.setup(&server, "/update", cfg.ota_user.c_str(), cfg.ota_pass.c_str());
  server.begin();
  Serial.println("HTTP server started");
}

void startAP() {
  WiFi.disconnect(true); delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Taupunkt_AP");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  startWebServices();
}

bool connectSTAOnce(unsigned long timeoutMs = 12000) {
  if (cfg.wifi_ssid.length() == 0) return false;
  Serial.printf("Connecting to SSID '%s'\n", cfg.wifi_ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(200); Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void applyTZ() {
  if (cfg.tz.length()) {
    setenv("TZ", cfg.tz.c_str(), 1);
    tzset();
    Serial.print("Applied TZ: "); Serial.println(cfg.tz);
  }
}

void startSTA() {
  if (!connectSTAOnce(10000)) {
    Serial.println("Initial STA connect failed. Staying in STA mode and will retry with backoff (no AP).");
  } else {
    Serial.print("Connected, IP="); Serial.println(WiFi.localIP());
    configTime(0, 0, cfg.ntp_host.c_str());
    applyTZ();
  }
  startWebServices();
}

// ---------- MQTT ----------
void setupMQTT() {
  if (!cfg.mqtt_enabled || cfg.mqtt_host.length()==0) return;
  if (cfg.mqtt_use_ssl) {
    mqttClient.setClient(wifiClientSecure);
    wifiClientSecure.setInsecure(); // convenience; replace with proper verification for production
  } else {
    mqttClient.setClient(wifiClient);
  }
  mqttClient.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);
}

void mqttReconnect() {
  if (!cfg.mqtt_enabled || cfg.mqtt_host.length()==0) return;
  if (mqttClient.connected()) return;
  Serial.printf("MQTT connect to %s:%u\n", cfg.mqtt_host.c_str(), cfg.mqtt_port);
  String clientId = "taupunkt-" + String(ESP.getChipId());
  bool ok;
  if (cfg.mqtt_user.length()) ok = mqttClient.connect(clientId.c_str(), cfg.mqtt_user.c_str(), cfg.mqtt_pass.c_str());
  else ok = mqttClient.connect(clientId.c_str());
  if (ok) { Serial.println("MQTT connected"); addMqttLog("MQTT connected"); }
  else { Serial.printf("MQTT failed rc=%d\n", mqttClient.state()); addMqttLog("MQTT failed rc="+String(mqttClient.state())); }
}

void publishMeasurementMQTT() {
  if (!cfg.mqtt_enabled || cfg.mqtt_host.length()==0) return;
  if (!mqttClient.connected()) mqttReconnect();
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["t1"] = t1; doc["h1"] = h1; doc["tp1"] = taupunkt(t1,h1);
  doc["t2"] = t2; doc["h2"] = h2; doc["tp2"] = taupunkt(t2,h2);
  doc["fan"] = rel ? 1 : 0;
  char buf[256];
  size_t n = serializeJson(doc, buf);
  if (mqttClient.publish(cfg.mqtt_topic.c_str(), buf, n)) addMqttLog("MQTT pub ok"); else addMqttLog("MQTT pub fail");
}

// ---------- Measurement / display / logging ----------
void doMeasure() {
  digitalWrite(LED_PIN, LOW); delay(80); digitalWrite(LED_PIN, HIGH);
  bool err = false;
  float rh1 = dht.readHumidity(); float tt1 = dht.readTemperature();
  if (isnan(rh1) || isnan(tt1)) { Serial.println("DHT read error"); err = true; } else { h1 = rh1 + KORREKTUR_H_1; t1 = tt1 + KORREKTUR_T_1; }
  sensors_event_t humidityEvent, tempEvent;
  if (!sht4.getEvent(&humidityEvent, &tempEvent)) { Serial.println("SHT4x read error"); err = true; } else { h2 = humidityEvent.relative_humidity + KORREKTUR_H_2; t2 = tempEvent.temperature + KORREKTUR_T_2; }
}

void doDisplay() {
  float tp1 = taupunkt(t1,h1), tp2 = taupunkt(t2,h2), deltaTP = tp1 - tp2;
  if (deltaTP > (SCHALTmin + HYSTERESE)) rel = true;
  if (deltaTP < (SCHALTmin)) rel = false;
  if (t1 < TEMP1_min) rel = false;
  if (t2 < TEMP2_min) rel = false;
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, rel ? LOW : HIGH); else digitalWrite(RELAY_PIN, rel ? HIGH : LOW);
  digitalWrite(LED_PIN, rel ? LOW : HIGH);
  lcd.clear();
  String out1 = String((int)round(t1)) + String((char)248) + "C|" + String((int)round(h1)) + "%|" + String(tp1,1) + String((char)248) + "C";
  String out2 = String((int)round(t2)) + String((char)248) + "C|" + String((int)round(h2)) + "%|" + String(tp2,1) + String((char)248) + "C";
  lcd.setCursor(0,0); lcd.print(out1);
  lcd.setCursor(0,1); lcd.print(out2);
  Serial.printf("S1: %.2f C | %.2f %% | tp1 %.2f C | S2: %.2f C | %.2f %% | tp2 %.2f C | fan %d\n", t1,h1,tp1,t2,h2,tp2, rel?1:0);
}

void doLog(bool force=false) {
  float tp1 = taupunkt(t1,h1), tp2 = taupunkt(t2,h2);
  String row = pt() + "," + String(t1,2) + "," + String(h1,2) + "," + String(tp1,2) + "," + String(t2,2) + "," + String(h2,2) + "," + String(tp2,2) + "," + String(rel?1:0) + "\n";
  Serial.print(row);
  logBuffer += row;
  int lines = 0; for (unsigned int i=0;i<logBuffer.length();++i) if (logBuffer.charAt(i)=='\n') ++lines;
  if (force || (lines > MAXLINES)) flushLogBufferToFS();
  publishMeasurementMQTT();
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200); delay(50);
  pinMode(RELAY_PIN, OUTPUT); if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, HIGH); else digitalWrite(RELAY_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(USR_PIN, INPUT_PULLUP);
  EEPROM.begin(EEPROM_SIZE);
  if (!LittleFS.begin()) Serial.println("LittleFS mount failed"); else Serial.println("LittleFS mounted");
  ensureLogHeader();

  // factory reset: hold USR low for 10s at boot
  if (digitalRead(USR_PIN) == LOW) {
    Serial.println("USR pressed at boot - waiting 10s for factory reset...");
    unsigned long start = millis();
    while (digitalRead(USR_PIN) == LOW && (millis()-start) < 10000) delay(200);
    if (digitalRead(USR_PIN) == LOW) { Serial.println("Factory reset: erasing config"); eraseConfigEEPROM(); delay(1000); ESP.restart(); }
    else Serial.println("USR released - continue");
  }

  loadConfig();

  dht.begin();
  Wire.begin();
  if (!sht4.begin()) Serial.println("SHT4x not found");
  lcd.init(); lcd.backlight(); lcd.clear(); lcd.setCursor(0,0); lcd.print("Teste Sensoren.."); delay(1200); lcd.clear();

  // start mode: if wifi_pass empty -> AP else STA (no AP fallback)
  if (cfg.wifi_pass.length()==0) { Serial.println("Empty wifi_pass => start AP"); startAP(); }
  else { Serial.println("Starting STA (no AP fallback)"); startSTA(); }

  // MQTT client set
  if (cfg.mqtt_use_ssl) {
    mqttClient.setClient(wifiClientSecure);
    wifiClientSecure.setInsecure(); // convenience
  } else {
    mqttClient.setClient(wifiClient);
  }
  mqttClient.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);

  doMeasure(); doDisplay(); firstRun = false;
  lastMeasure = millis(); lastDisplay = millis(); lastLog = millis();
}

void loop(){
  unsigned long now = millis();
  server.handleClient();

  // wifi reconnect/backoff only in STA mode, no AP fallback
  static unsigned long wifiReconnectDelay = 1000;
  static unsigned long lastWifiAttempt = 0;
  if (WiFi.getMode() == WIFI_STA) {
    if (WiFi.status() != WL_CONNECTED) {
      if (millis() - lastWifiAttempt > wifiReconnectDelay) {
        lastWifiAttempt = millis();
        Serial.println("Attempting WiFi reconnect...");
        WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 8000) delay(200);
        if (WiFi.status() == WL_CONNECTED) { Serial.println("WiFi reconnected"); wifiReconnectDelay = 1000; configTime(0,0,cfg.ntp_host.c_str()); applyTZ(); }
        else { Serial.println("WiFi reconnect failed"); wifiReconnectDelay = min((unsigned long)60000, wifiReconnectDelay*2); }
      }
    }
  }

  if (now - lastMeasure >= MEASURE_INTERVAL_MS) { lastMeasure = now; doMeasure(); }
  if (now - lastDisplay >= DISPLAY_INTERVAL_MS) { lastDisplay = now; doDisplay(); }
  if (now - lastLog >= LOG_INTERVAL_MS) { lastLog = now; doLog(true); }

  static unsigned long lastFlush = 0;
  if (millis() - lastFlush > 60000 && logBuffer.length() > 0) { flushLogBufferToFS(); lastFlush = millis(); }

  // MQTT reconnect/backoff
  if (cfg.mqtt_enabled && cfg.mqtt_host.length()) {
    static unsigned long lastMqttAttempt = 0;
    static unsigned long currentMqttBackoff = 1000;
    if (!mqttClient.connected() && millis() - lastMqttAttempt > currentMqttBackoff) {
      lastMqttAttempt = millis();
      String clientId = "taupunkt-" + String(ESP.getChipId());
      bool ok;
      if (cfg.mqtt_user.length()) ok = mqttClient.connect(clientId.c_str(), cfg.mqtt_user.c_str(), cfg.mqtt_pass.c_str());
      else ok = mqttClient.connect(clientId.c_str());
      if (ok) { addMqttLog("MQTT connected"); currentMqttBackoff = 1000; }
      else { addMqttLog("MQTT failed rc="+String(mqttClient.state())); currentMqttBackoff = min((unsigned long)60000, currentMqttBackoff*2); }
    } else if (mqttClient.connected()) mqttClient.loop();
  }

  delay(10);
}
