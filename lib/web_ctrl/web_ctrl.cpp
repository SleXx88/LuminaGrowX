#include "web_ctrl.h"
#include "rtc_ctrl.h"
#include "plant_ctrl.h"
#include "net_ctrl.h"
#include "../../include/health.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include "../../include/version.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace web_ctrl;
using net_ctrl::NetCtrl;

static const char* APP_CFG_PATH   = "/cfg/app.json";
static const char* GROW_CFG_PATH  = "/cfg/grow.json";
static const char* NOTIFY_CFG_PATH= "/cfg/notify.json";
static const char* UPDATE_CFG_PATH= "/cfg/update.json";
// Version kommt aus include/version.h (FW_VERSION)
// GitHub Releases (latest) defaults â€“ fest im Code hinterlegt
static const char* GH_OWNER = "SleXx88";
static const char* GH_REPO  = "LuminaGrowX";
static const char* GH_ASSET = "LuminaGrowX-package.tar";
static const char* UPDATE_MANIFEST_URL = ""; // leer -> aus /cfg/update.json lesen

static bool readTextFile(const char* path, String& out) {
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return false;
  out = f.readString();
  f.close();
  return true;
}
static bool writeTextFile(const char* path, const String& data) {
  LittleFS.mkdir("/cfg");
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) return false;
  f.print(data);
  f.close();
  return true;
}

WebCtrl::WebCtrl() {}

bool WebCtrl::loadAppCfg(AppCfg& out) {
  String s; if (!readTextFile(APP_CFG_PATH, s)) return false;
  JsonDocument doc; if (deserializeJson(doc, s)) return false;
  out.seed = String((const char*)doc["seed"]);
  app_ = out; return true;
}
bool WebCtrl::saveAppCfg(const AppCfg& c) {
  JsonDocument doc; doc["seed"] = c.seed; String out; serializeJson(doc, out);
  bool ok = writeTextFile(APP_CFG_PATH, out); if (ok) app_ = c; return ok;
}
bool WebCtrl::loadGrow(GrowState& out) {
  String s; if (!readTextFile(GROW_CFG_PATH, s)) return false;
  JsonDocument doc; if (deserializeJson(doc, s)) return false;
  out.started = doc["started"].as<bool>();
  out.start_epoch = doc["start_epoch"].as<uint32_t>();
  out.total_days = doc["total_days"].as<uint16_t>();
  grow_ = out; return true;
}
bool WebCtrl::saveGrow(const GrowState& g) {
  JsonDocument doc; doc["started"] = g.started; doc["start_epoch"] = g.start_epoch; doc["total_days"] = g.total_days; String out; serializeJson(doc, out);
  bool ok = writeTextFile(GROW_CFG_PATH, out); if (ok) grow_ = g; return ok;
}
bool WebCtrl::loadNotify(NotifyCfg& out) {
  String s; if (!readTextFile(NOTIFY_CFG_PATH, s)) return false;
  JsonDocument doc; if (deserializeJson(doc, s)) return false;
  out.enabled = doc["enabled"].as<bool>();
  out.phone = String((const char*)doc["phone"]);
  out.apikey = String((const char*)doc["apikey"]);
  notify_ = out; return true;
}
bool WebCtrl::saveNotify(const NotifyCfg& n) {
  JsonDocument doc; doc["enabled"] = n.enabled; doc["phone"] = n.phone; doc["apikey"] = n.apikey; String out; serializeJson(doc, out);
  bool ok = writeTextFile(NOTIFY_CFG_PATH, out); if (ok) notify_ = n; return ok;
}

static uint64_t uptime_s() { return millis() / 1000ULL; }

void WebCtrl::begin(plant_ctrl::PlantCtrl* ctrl, RTC_Ctrl* rtc, NetCtrl* net) {
  ctrl_ = ctrl; rtc_ = rtc; net_ = net;
  AppCfg tmpA; loadAppCfg(tmpA);
  GrowState tmpG; loadGrow(tmpG);
  NotifyCfg tmpN; loadNotify(tmpN);

  ws_.onEvent([this](AsyncWebSocket* wss, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
    if (t == WS_EVT_CONNECT) { c->text(makeStatusJson_()); }
  });
  http_.addHandler(&ws_);

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");

  setupRoutes_();
  http_.begin();
}

void WebCtrl::loop() {
  uint32_t now = millis();
  if (now >= nextProbeAt_) {
    if (net_) net_->probeInternet();
    nextProbeAt_ = now + 60000;
  }
  if (now >= nextPushAt_) {
    if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
    nextPushAt_ = now + 2000;
  }
  if (rebootAt_ && now >= rebootAt_) {
    delay(100);
    ESP.restart();
  }
}

void WebCtrl::setupRoutes_() {
  http_.on("/", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/index.html", "text/html"); });
  http_.on("/index.html", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/index.html", "text/html"); });
  http_.on("/pico.min.css", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/pico.min.css", "text/css"); });
  http_.on("/custom.css", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/custom.css", "text/css"); });
  http_.on("/bg.jpg", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/bg.jpg", "image/jpeg"); });

  // FS diagnostics
  http_.on("/api/fs/info", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", fsInfoJson_()); });
  http_.on("/api/fs/list", HTTP_GET, [this](AsyncWebServerRequest* req){
    String path = "/";
    if (req->hasParam("p")) path = req->getParam("p")->value();
    if (!path.length()) path = "/";
    req->send(200, "application/json", fsListJson_(path.c_str()));
  });

  http_.on("/api/info", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", makeInfoJson_()); });
  http_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", makeStatusJson_()); });

  // Register updater routes
  registerUpdateRoutes_();

  http_.on("/api/seed", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             if (!doc["seed"].isNull()) { app_.seed = String((const char*)doc["seed"]); saveAppCfg(app_); }
             req->send(200, "application/json", "{\"ok\":true}");
             if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
           });

  // /api/network: accepts { ssid?, pass?, use_static?, static:{ip,mask,gw,dns}? , notify?{}, reboot? }
  http_.on("/api/network", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             // network
             net_ctrl::NetworkConfig nc = net_ ? net_->cfg() : net_ctrl::NetworkConfig{};
             bool anyNetFieldTouched = false;
             if (!doc["ssid"].isNull()) { nc.ssid = String((const char*)doc["ssid"]); anyNetFieldTouched = true; }
             if (!doc["pass"].isNull()) { nc.pass = String((const char*)doc["pass"]); anyNetFieldTouched = true; }
             if (!doc["use_static"].isNull()) { nc.useStatic = doc["use_static"].as<bool>(); anyNetFieldTouched = true; }
             if (!doc["static"].isNull()) {
               JsonObject st = doc["static"].as<JsonObject>();
               if (!st["ip"].isNull())   nc.ip   = String((const char*)st["ip"]);
               if (!st["mask"].isNull()) nc.mask = String((const char*)st["mask"]);
               if (!st["gw"].isNull())   nc.gw   = String((const char*)st["gw"]);
               if (!st["dns"].isNull())  nc.dns  = String((const char*)st["dns"]);
               anyNetFieldTouched = true;
             }
             // notify (optional pass-through)
             if (!doc["notify"].isNull()) {
               JsonObject ntf = doc["notify"].as<JsonObject>();
               NotifyCfg n = notify_;
             if (!ntf["enabled"].isNull()) n.enabled = ntf["enabled"].as<bool>();
             if (!ntf["phone"].isNull())   n.phone   = String((const char*)ntf["phone"]);
             if (!ntf["apikey"].isNull())  { String k = String((const char*)ntf["apikey"]); if (k.length() && k != "***") n.apikey = k; }
               saveNotify(n);
             }
             bool requestedReboot = doc["reboot"].as<bool>();
             if (net_) net_->saveConfig(nc);
             bool connectedNow = false;
             if (anyNetFieldTouched && net_) {
               // Sofortiger Verbindungsversuch ohne Reboot
               connectedNow = net_->reconnectSTA(true /*close AP on success*/);
             }
             // Wenn erfolgreich direkt verbunden: Reboot unterdrÃ¼cken, sonst optional rebooten
             bool doReboot = (!connectedNow) && requestedReboot;
             req->send(200, "application/json", "{\"ok\":true}");
             if (doReboot) rebootAt_ = millis() + 800;
           });

  // standalone notify save
  http_.on("/api/notify", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             Serial.printf("[DEBUG] /api/notify hit, body len=%u\n", (unsigned)len);
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             NotifyCfg n = notify_;
             if (!doc["enabled"].isNull()) n.enabled = doc["enabled"].as<bool>();
             if (!doc["phone"].isNull())   n.phone   = String((const char*)doc["phone"]);
             if (!doc["apikey"].isNull())  { String k = String((const char*)doc["apikey"]); if (k.length() && k != "***") n.apikey = k; }
             saveNotify(n);
             bool sent = false;
             if (n.enabled && n.phone.length() && n.apikey.length()) {
               String timeStr = rtc_ ? rtc_->readTimeString() : String("?");
               String ipStr = (net_ && net_->isConnected()) ? net_->staIP().toString() : String("0.0.0.0");
               int rssi = net_ ? net_->rssi() : -127;
               float t = 0.f, rh = 0.f; float vpd = 0.f; float led = 0.f, fan = 0.f;
               if (ctrl_) { t = (float)ctrl_->currentTemp(); rh = (float)ctrl_->currentRh(); vpd = (float)ctrl_->currentVpd();
                 led = ctrl_->currentLedPercentEffective(); fan = ctrl_->currentFanPercent(); }
               String msg = String("LuminaGrowX: Benachrichtigung aktiviert ") + timeStr +
                            String(" | IP ") + ipStr +
                            String(" | RSSI ") + String(rssi) + String(" dBm") +
                            String(" | T ") + String(t,1) + String(" C, RH ") + String(rh,1) + String("%, VPD ") + String(vpd,2) + String(" kPa") +
                            String(" | LED ") + String(led,0) + String("%, Fan ") + String(fan,0) + String("%");
               Serial.printf("[API] /api/notify sending activation msg to %s\n", n.phone.c_str());
               sent = whatsappSend_(n.phone, n.apikey, msg);
               Serial.println(sent ? "[WA] Activation message sent" : "[WA] Activation message FAILED");
             }
             JsonDocument resp; resp["ok"] = true; resp["sent"] = sent; String out; serializeJson(resp, out);
             req->send(200, "application/json", out);
           });
  http_.on("/api/notify", HTTP_OPTIONS, [this](AsyncWebServerRequest* req){
    Serial.println("[DEBUG] OPTIONS /api/notify");
    req->send(200);
  });

  http_.on("/api/notify/test", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             Serial.println("[DEBUG] UI: Testnachricht-Button gedrueckt");
             Serial.printf("[API] /api/notify/test body len=%u\n", (unsigned)len);
             JsonDocument doc; 
             DeserializationError err = deserializeJson(doc, data, len);
             if (err) { 
               Serial.printf("[API] /api/notify/test JSON error: %s\n", err.c_str());
               req->send(400, "application/json", "{\"error\":\"bad json\"}"); 
               return; 
             }
             // If notifications are disabled but request provides phone+apikey, allow a one-off test send
             bool _hasReqPhone = !doc["phone"].isNull() && String((const char*)doc["phone"]).length() > 0;
             bool _hasReqKey   = !doc["apikey"].isNull() && String((const char*)doc["apikey"]).length() > 0;
             if (!notify_.enabled && _hasReqPhone && _hasReqKey) {
               String _phone = String((const char*)doc["phone"]);
               String _key   = String((const char*)doc["apikey"]);
               String timeStr = rtc_ ? rtc_->readTimeString() : String("?");
               String ipStr = (net_ && net_->isConnected()) ? net_->staIP().toString() : String("0.0.0.0");
               int rssi = net_ ? net_->rssi() : -127;
               float t = 0.f, rh = 0.f; float vpd = 0.f; float led = 0.f, fan = 0.f;
               if (ctrl_) { t = (float)ctrl_->currentTemp(); rh = (float)ctrl_->currentRh(); vpd = (float)ctrl_->currentVpd();
                 led = ctrl_->currentLedPercentEffective(); fan = ctrl_->currentFanPercent(); }
               String _msg = String("LuminaGrowX Test ") + timeStr +
                             String(" | IP ") + ipStr +
                             String(" | RSSI ") + String(rssi) + String(" dBm") +
                             String(" | T ") + String(t,1) + String(" C, RH ") + String(rh,1) + String("%, VPD ") + String(vpd,2) + String(" kPa") +
                             String(" | LED ") + String(led,0) + String("%, Fan ") + String(fan,0) + String("%");
               Serial.printf("[API] /api/notify/test (one-off) phone=%s\n", _phone.c_str());
               bool ok = whatsappSend_(_phone, _key, _msg);
               if (ok) { Serial.println("[WA] Testnachricht OK (one-off)"); req->send(200, "application/json", "{\"ok\":true}"); }
               else { Serial.println("[WA] Testnachricht FEHLER (one-off)"); req->send(500, "application/json", "{\"error\":\"send failed\"}"); }
               return;
             }
             if (!notify_.enabled) {
               Serial.println("[WA] Notifications disabled (switch off) â€” aborting send");
               req->send(403, "application/json", "{\"error\":\"notifications disabled\"}");
               return;
             }
             String phone = doc["phone"].isNull() ? notify_.phone : String((const char*)doc["phone"]);
             String key   = doc["apikey"].isNull() ? notify_.apikey : String((const char*)doc["apikey"]);
             // Sinnvolle Testnachricht mit Zeit, IP, RSSI, Temp/RH/VPD, LED/Fan
             String timeStr = rtc_ ? rtc_->readTimeString() : String("?");
             String ipStr = (net_ && net_->isConnected()) ? net_->staIP().toString() : String("0.0.0.0");
             int rssi = net_ ? net_->rssi() : -127;
             float t = 0.f, rh = 0.f; float vpd = 0.f; float led = 0.f, fan = 0.f;
             if (ctrl_) {
               t = (float)ctrl_->currentTemp(); rh = (float)ctrl_->currentRh(); vpd = (float)ctrl_->currentVpd();
               led = ctrl_->currentLedPercentEffective(); fan = ctrl_->currentFanPercent();
             }
             String msg = String("LuminaGrowX Test ") + timeStr +
                          String(" | IP ") + ipStr +
                          String(" | RSSI ") + String(rssi) + String(" dBm") +
                          String(" | T ") + String(t,1) + String(" C, RH ") + String(rh,1) + String("%, VPD ") + String(vpd,2) + String(" kPa") +
                          String(" | LED ") + String(led,0) + String("%, Fan ") + String(fan,0) + String("%");
             Serial.printf("[API] /api/notify/test phone=%s\n", phone.c_str());
             bool ok = whatsappSend_(phone, key, msg);
             if (ok) { Serial.println("[WA] Testnachricht OK"); req->send(200, "application/json", "{\"ok\":true}"); }
             else { Serial.println("[WA] Testnachricht FEHLER"); req->send(500, "application/json", "{\"error\":\"send failed\"}"); }
            });
  http_.on("/api/notify/test", HTTP_OPTIONS, [this](AsyncWebServerRequest* req){
    Serial.println("[DEBUG] OPTIONS /api/notify/test");
    req->send(200);
  });

  // Simple debug touch endpoint to verify frontend click reaches backend
  http_.on("/api/debug/touch", HTTP_GET, [this](AsyncWebServerRequest* req){
    String stage = req->hasParam("stage") ? req->getParam("stage")->value() : String("");
    String en = req->hasParam("en") ? req->getParam("en")->value() : String("");
    String phoneLen = req->hasParam("phoneLen") ? req->getParam("phoneLen")->value() : String("");
    String hasKey = req->hasParam("key") ? req->getParam("key")->value() : String("");
    if (stage.length()) {
      Serial.printf("[DEBUG] /api/debug/touch stage=%s en=%s phoneLen=%s key=%s\n",
                    stage.c_str(), en.c_str(), phoneLen.c_str(), hasKey.c_str());
    } else {
      Serial.println("[DEBUG] /api/debug/touch hit");
    }
    req->send(200, "text/plain", "ok");
  });

  http_.on("/api/grow", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             String action = String((const char*)doc["action"]);
             time_t now = time(nullptr);
             if (action == "start") {
               grow_.started = true; grow_.start_epoch = (uint32_t)now;
               if (!doc["total_days"].isNull()) grow_.total_days = (uint16_t)doc["total_days"].as<uint16_t>();
               saveGrow(grow_);
               req->send(200, "application/json", "{\"ok\":true}");
               if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
             } else if (action == "stop") {
               grow_.started = false; grow_.start_epoch = 0; saveGrow(grow_);
               req->send(200, "application/json", "{\"ok\":true}");
               if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
             } else {
               req->send(400, "application/json", "{\"error\":\"unknown action\"}");
             }
           });
}

void WebCtrl::sendFile_(AsyncWebServerRequest* req, const char* path, const char* mime) {
  // Ensure FS is mounted and report useful diagnostics
  if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
    Serial.println(F("[FS] not mounted when serving file"));
    req->send(500, "text/plain", "FS not mounted");
    return;
  }
  if (!LittleFS.exists(path)) {
    Serial.printf("[FS] 404 missing: %s\n", path);
    Serial.printf("[FS] total=%u used=%u free=%u\n",
                  (unsigned)LittleFS.totalBytes(), (unsigned)LittleFS.usedBytes(),
                  (unsigned)((LittleFS.totalBytes()>LittleFS.usedBytes())?(LittleFS.totalBytes()-LittleFS.usedBytes()):0));
    // Optionally list root to aid debugging
    File dir = LittleFS.open("/");
    if (dir && dir.isDirectory()) {
      Serial.println(F("[FS] root entries:"));
      File f; int cnt=0;
      while ((f = dir.openNextFile())) { Serial.printf(" - %s (%u)\n", f.name(), (unsigned)f.size()); f.close(); if (++cnt>20) break; }
    }
    req->send(404, "text/plain", "Not found");
    return;
  }
  req->send(LittleFS, path, mime);
}

static String fmtDateTime_local(time_t t) {
  if (t == 0) return String("");
  struct tm tmL; localtime_r(&t, &tmL);
  char buf[32]; strftime(buf, sizeof(buf), "%d.%m.%Y %H:%M:%S", &tmL);
  return String(buf);
}

String WebCtrl::makeStatusJson_() {
  JsonDocument doc;
  doc["ip"] = net_ && net_->isConnected() ? net_->staIP().toString() : String("0.0.0.0");
  doc["use_static"] = net_ ? net_->cfg().useStatic : false;
  doc["wifi_connected"] = net_ ? net_->isConnected() : false;
  doc["internet_ok"] = net_ ? net_->internetOK() : false;
  doc["rssi"] = net_ ? net_->rssi() : -127;
  doc["uptime_s"] = (uint64_t)uptime_s();
  doc["ntp_time"] = rtc_ ? rtc_->readTimeString() : String("");
  // Env
  float tIn = NAN, rhIn = NAN; double vpd = NAN; float tOut = NAN;
  if (ctrl_) {
    tIn = (float)ctrl_->currentTemp();
    rhIn = (float)ctrl_->currentRh();
    vpd = ctrl_->currentVpd();
    tOut = (float)ctrl_->currentTempOut();
  }
  auto round1 = [](float x){ return isnan(x)? NAN : (float)roundf(x*10.f)/10.f; };
  doc["temp_c"] = round1(tIn);
  doc["humi_rh"] = round1(rhIn);
  doc["vpd_kpa"] = round1((float)vpd);
  doc["temp_out_c"] = round1(tOut);
  // Actuators
  float ledPct = ctrl_ ? ctrl_->currentLedPercentEffective() : 0.0f;
  float fanPct = ctrl_ ? ctrl_->currentFanPercent() : 0.0f;
  doc["light_on"] = ledPct > 0.5f;
  doc["fan_on"]   = fanPct > 0.5f;
  doc["pump_on"]  = false; // kein Pumpen-State integriert

  JsonObject healthObj = doc["health"].to<JsonObject>();
  const auto& hs = health::state();
  bool anyErr = health::any_error();
  healthObj["state"] = anyErr ? "error" : "ok";
  healthObj["message"] = hs.message;
  healthObj["control_paused"] = hs.control_paused;
  JsonObject mods = healthObj["modules"].to<JsonObject>();
  mods["i2c0"] = hs.mod.i2c0_ok;
  mods["i2c1"] = hs.mod.i2c1_ok;
  mods["sht_in"] = hs.mod.sht_in_ok;
  mods["sht_out"] = hs.mod.sht_out_ok;
  mods["dac"] = hs.mod.dac_ok;
  mods["fan"] = hs.mod.fan_ok;
  mods["rtc"] = hs.mod.rtc_ok;
  mods["tof"] = hs.mod.tof_ok;
  mods["fs"] = hs.mod.fs_ok;

  // Grow block
  JsonObject g = doc["grow"].to<JsonObject>();
  g["started"] = grow_.started;
  g["start_epoch"] = grow_.start_epoch;
  g["total_days"] = grow_.total_days;
  uint16_t day = 0;
  if (grow_.started && grow_.start_epoch) {
    time_t now = time(nullptr);
    if (now >= grow_.start_epoch) {
      uint32_t days = (uint32_t)((now - grow_.start_epoch)/86400UL) + 1;
      if (days > grow_.total_days) days = grow_.total_days;
      day = (uint16_t)days;
    }
  }
  g["day"] = day;
  const char* phase = "";
  if (day > 0) { if (day <= 14) phase = "Keimung/Seedling"; else if (day <= 35) phase = "Vegetationsphase"; else phase = "BlÃ¼tephase"; }
  g["phase"] = phase;

  // Notify (mask apikey)
  JsonObject ntf = doc["notify"].to<JsonObject>();
  ntf["enabled"] = notify_.enabled;
  ntf["phone"] = notify_.phone;
  ntf["apikey"] = notify_.apikey.length() ? "***" : "";

  // AP info (nur wenn AP aktiv)
  JsonObject ap = doc["ap"].to<JsonObject>();
  if (net_ && net_->apActive()) {
    ap["ssid"] = net_->apSSID();
    ap["ip"] = net_->apIP().toString();
  } else {
    ap["ssid"] = "";
    ap["ip"] = "";
  }

  doc["seed"] = app_.seed;
  doc["fw"] = FW_VERSION;
  doc["build"] = String(__DATE__) + String(" ") + String(__TIME__);
  doc["ts"] = (uint64_t)millis();

  String out; serializeJson(doc, out); return out;
}

String WebCtrl::makeInfoJson_() {
  JsonDocument doc;
  doc["seed"] = app_.seed;
  JsonObject netw = doc["network"].to<JsonObject>();
  if (net_) {
    netw["ssid"] = net_->cfg().ssid;
    netw["use_static"] = net_->cfg().useStatic;
    JsonObject st = netw["static"].to<JsonObject>();
    st["ip"]   = net_->cfg().ip;
    st["mask"] = net_->cfg().mask;
    st["gw"]   = net_->cfg().gw;
    st["dns"]  = net_->cfg().dns;
  }
  JsonObject ntf = doc["notify"].to<JsonObject>();
  ntf["enabled"] = notify_.enabled;
  ntf["phone"] = notify_.phone;
  ntf["apikey"] = notify_.apikey; // Klartext, nur bei /api/info
  String out; serializeJson(doc, out); return out;
}

String WebCtrl::urlEncode_(const String& s) {
  String out; char hex[4];
  for (size_t i=0;i<s.length();++i){ char c=s[i];
    if ((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.'||c=='~') out+=c; else { snprintf(hex,sizeof(hex),"%%%02X",(unsigned char)c); out+=hex; }
  } return out;
}

bool WebCtrl::whatsappSend_(const String& phone, const String& apikey, const String& message) {
  if (!net_ || !net_->isConnected()) { Serial.println("[WA] not connected to WiFi"); return false; }
  String qs = String("phone=") + urlEncode_(phone) + "&text=" + urlEncode_(message) + "&apikey=" + urlEncode_(apikey);

  // Try HTTPS with insecure client (no CA)
  {
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http;
    String url = String("https://api.callmebot.com/whatsapp.php?") + qs;
    Serial.println(String("[WA] HTTPS ") + url);
    if (http.begin(client, url)) {
      int code = http.GET();
      String body = http.getString();
      Serial.printf("[WA] HTTPS code=%d\n", code);
      if (body.length()) { Serial.print("[WA] HTTPS body: "); Serial.println(body); }
      http.end();
      if (code == 200) return true;
    } else {
      Serial.println("[WA] HTTPS begin() failed");
    }
  }

  // Fallback to HTTP (not secure)
  {
    WiFiClient client;
    HTTPClient http;
    String url = String("http://api.callmebot.com/whatsapp.php?") + qs;
    Serial.println(String("[WA] HTTP ") + url);
    if (http.begin(client, url)) {
      int code = http.GET();
      String body = http.getString();
      Serial.printf("[WA] HTTP code=%d\n", code);
      if (body.length()) { Serial.print("[WA] HTTP body: "); Serial.println(body); }
      http.end();
      if (code == 200) return true;
    } else {
      Serial.println("[WA] HTTP begin() failed");
    }
  }
  return false;
}

// ========== Updater routes and helpers ==========

void WebCtrl::registerUpdateRoutes_() {
  // Minimal Update UI (dynamic)
  http_.on("/update", HTTP_GET, [this](AsyncWebServerRequest* req){
    sendFile_(req, "/update.html", "text/html");
  });

  // Check manifest or GitHub latest
  http_.on("/api/update/check", HTTP_GET, [this](AsyncWebServerRequest* req){
    JsonDocument resp;
    resp["current"] = FW_VERSION;
    resp["hasUpdate"] = false;
    resp["latest"] = "";
    resp["tar_url"] = "";
    if (!net_ || !net_->isConnected()) { String out; serializeJson(resp, out); req->send(200, "application/json", out); return; }
    // Prefer GitHub latest if configured
    String ghUrl, latestTag;
    if (ghLatestTarUrl_(ghUrl, latestTag)) {
      resp["latest"] = latestTag;
      resp["tar_url"] = ghUrl;
      resp["hasUpdate"] = (latestTag.length() && latestTag != String(FW_VERSION));
      String out; serializeJson(resp, out); req->send(200, "application/json", out); return;
    }
    // Fallback: Manifest URL mode
    String manUrl = manifestUrl_();
    if (!manUrl.length()) { String out; serializeJson(resp, out); req->send(200, "application/json", out); return; }
    WiFiClientSecure client; client.setInsecure();
    HTTPClient http; if (!http.begin(client, manUrl)) { String out; serializeJson(resp, out); req->send(200, "application/json", out); return; }
    int code = http.GET();
    if (code == 200) {
      JsonDocument man; DeserializationError err = deserializeJson(man, http.getStream());
      if (!err) {
        String latest = String((const char*)man["version"]);
        String tar = String((const char*)man["tar_url"]);
        resp["latest"] = latest;
        resp["tar_url"] = tar;
        resp["hasUpdate"] = (latest.length() && latest != String(FW_VERSION));
      }
    }
    http.end(); String out; serializeJson(resp, out); req->send(200, "application/json", out);
  });

  // Remote run (GitHub latest preferred, manifest fallback)
  http_.on("/api/update/remote", HTTP_POST, [this](AsyncWebServerRequest* req){
    if (updateJobRunning_) { req->send(409, "application/json", "{\"error\":\"update running\"}"); return; }
    if (!net_ || !net_->isConnected()) { req->send(400, "application/json", "{\"error\":\"wifi not connected\"}"); return; }
    startRemoteUpdateJob_();
    req->send(202, "application/json", "{\"started\":true}");
  });

  // Manual upload (.tar)
  http_.on("/api/update/upload", HTTP_POST,
    [this](AsyncWebServerRequest* req){
      // Finalize upload to temp file, then start async apply job
      bool fwUpd=false; int filesUpd=0; String err;
      bool saved = handlePackageUploadEnd_(fwUpd, filesUpd, err);
      if (!saved) { JsonDocument resp; resp["ok"]=false; resp["error"]=err; String out; serializeJson(resp,out); req->send(500,"application/json",out); return; }
      if (updateJobRunning_) { req->send(409, "application/json", "{\"error\":\"update running\"}"); return; }
      startApplyUploadedJob_();
      req->send(202, "application/json", "{\"started\":true}");
    },
    [this](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final){
      if (index == 0) { handlePackageUploadBegin_(); }
      handlePackageUploadWrite_(data, len);
      // finalize in completion handler
    }
  );

  // Progress polling
  http_.on("/api/update/progress", HTTP_GET, [this](AsyncWebServerRequest* req){
    String out = updateProgressJson_(); req->send(200, "application/json", out);
  });

  // Update config endpoints (manifest and/or GitHub latest)
  http_.on("/api/update/config", HTTP_GET, [this](AsyncWebServerRequest* req){
    String man, owner, repo, asset; loadUpdateCfg_(man, owner, repo, asset);
    JsonDocument resp; resp["manifest_url"] = man; resp["gh_owner"] = owner; resp["gh_repo"] = repo; resp["gh_asset"] = asset; String out; serializeJson(resp, out); req->send(200, "application/json", out);
  });
  http_.on("/api/update/config", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             String man = doc["manifest_url"].isNull()? String("") : String((const char*)doc["manifest_url"]);
             String owner = doc["gh_owner"].isNull()? String("") : String((const char*)doc["gh_owner"]);
             String repo  = doc["gh_repo"].isNull()?  String("") : String((const char*)doc["gh_repo"]);
             String asset = doc["gh_asset"].isNull()? String("") : String((const char*)doc["gh_asset"]);
             bool ok = saveUpdateCfg_(man, owner, repo, asset);
             req->send(ok?200:500, "application/json", ok?"{\"ok\":true}":"{\"ok\":false}");
           });
}

void WebCtrl::startRemoteUpdateJob_() {
  if (updateJobRunning_) return;
  updateJobRunning_ = true; updatePhase_ = "checking"; updateMsg_ = "GitHub latest"; updateOk_ = false; updateFwUpdated_ = false; updateFilesUpdated_ = 0; updateErr_ = "";
  // Spawn on Core 0 with moderate stack
  xTaskCreatePinnedToCore(&WebCtrl::updateTaskTrampoline_, "upd_remote", 12288, this, 1, nullptr, 0);
}

void WebCtrl::startApplyUploadedJob_() {
  if (updateJobRunning_) return;
  updateJobRunning_ = true; updatePhase_ = "applying"; updateMsg_ = "apply uploaded"; updateOk_ = false; updateFwUpdated_ = false; updateFilesUpdated_ = 0; updateErr_ = "";
  xTaskCreatePinnedToCore(&WebCtrl::updateTaskTrampoline_, "upd_apply", 12288, this, 1, nullptr, 0);
}

void WebCtrl::updateTaskTrampoline_(void* arg) {
  WebCtrl* self = static_cast<WebCtrl*>(arg);
  // Determine mode from phase
  String local = self->pkgTempPath_;
  bool remote = (self->updateMsg_.indexOf("GitHub") >= 0) || (self->updatePhase_ == "checking");
  self->updateTaskRun_(remote, local);
  vTaskDelete(nullptr);
}

void WebCtrl::updateTaskRun_(bool useRemote, const String& localTarPath) {
  // Resolve TAR URL if remote
  if (useRemote) {
    updatePhase_ = "checking"; updateMsg_ = "resolve latest";
    String tarUrl, latestTag;
    if (!ghLatestTarUrl_(tarUrl, latestTag)) {
      // Fallback to manifest
      String manUrl = manifestUrl_();
      if (!manUrl.length()) { updatePhase_ = "error"; updateErr_ = "manifest url not set"; updateJobRunning_ = false; return; }
      WiFiClientSecure client; client.setInsecure(); HTTPClient http;
      if (!http.begin(client, manUrl)) { updatePhase_ = "error"; updateErr_ = "manifest begin failed"; updateJobRunning_ = false; return; }
      int code = http.GET();
      if (code == 200) {
        JsonDocument man; if (!deserializeJson(man, http.getStream())) { tarUrl = String((const char*)man["tar_url"]); }
      }
      http.end();
      if (!tarUrl.length()) { updatePhase_ = "error"; updateErr_ = "no tar_url"; updateJobRunning_ = false; return; }
    }
    updatePhase_ = "downloading"; updateMsg_ = "download tar";
    if (!downloadToFile_(tarUrl, pkgTempPath_.c_str())) { updatePhase_ = "error"; if (updateErr_.length()==0) updateErr_ = "download failed"; updateMsg_ = updateErr_; updateJobRunning_ = false; return; }
  }
  // Apply local tar
  updatePhase_ = "applying"; updateMsg_ = "apply tar";
  bool fwUpd=false; int filesUpd=0; String err;
  bool ok = applyPackageFromFile_(pkgTempPath_.c_str(), fwUpd, filesUpd, err);
  if (ok) { LittleFS.remove(pkgTempPath_.c_str()); }
  updateOk_ = ok; updateFwUpdated_ = fwUpd; updateFilesUpdated_ = filesUpd; updateErr_ = ok? String("") : err;
  updatePhase_ = ok ? "done" : "error";
  updateJobRunning_ = false;
  if (ok && fwUpd) { rebootAt_ = millis() + 750; }
}

String WebCtrl::updateProgressJson_() {
  JsonDocument doc;
  doc["running"] = updateJobRunning_;
  doc["phase"] = updatePhase_;
  doc["msg"] = updateMsg_;
  doc["ok"] = updateOk_;
  doc["fwUpdated"] = updateFwUpdated_;
  doc["filesUpdated"] = updateFilesUpdated_;
  doc["error"] = updateErr_;
  JsonObject dl = doc["download"].to<JsonObject>();
  dl["total"] = updateDLTotal_;
  dl["sofar"] = updateDLSoFar_;
  dl["pct"] = (updateDLTotal_>0 && updateDLSoFar_>=0) ? (int)((100LL*updateDLSoFar_)/updateDLTotal_) : -1;
  JsonObject ap = doc["apply"].to<JsonObject>();
  ap["total"] = updateApplyTotal_;
  ap["sofar"] = updateApplySoFar_;
  ap["pct"] = (updateApplyTotal_>0 && updateApplySoFar_>=0) ? (int)((100LL*updateApplySoFar_)/updateApplyTotal_) : -1;
  String out; serializeJson(doc, out); return out;
}

bool WebCtrl::handlePackageUploadBegin_() {
  LittleFS.remove(pkgTempPath_.c_str());
  File f = LittleFS.open(pkgTempPath_.c_str(), FILE_WRITE);
  if (!f) return false; f.close(); return true;
}

bool WebCtrl::handlePackageUploadWrite_(const uint8_t* data, size_t len) {
  File f = LittleFS.open(pkgTempPath_.c_str(), FILE_APPEND);
  if (!f) return false; size_t w = f.write(data, len); f.close(); return w == len;
}

bool WebCtrl::downloadToFile_(const String& url, const char* path) {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String current = url;
  // Verfolge bis zu 5 Redirect-Hops (GitHub nutzt meist 2)
  for (int hop = 0; hop < 5; ++hop) {
    if (!http.begin(client, current)) { updateErr_ = "http begin failed"; return false; }
    http.setUserAgent("LuminaGrowX");
    int code = http.GET();
    // Redirects (301/302/303/307/308)
    if ((code >= 300 && code < 400)) {
      String loc = http.getLocation();
      http.end();
      if (!loc.length()) { updateErr_ = "redirect without location"; return false; }
      current = loc;
      continue; // nächsten Hop versuchen
    }
    if (code != 200) { updateErr_ = String("http code ")+String(code); http.end(); return false; }

    // Stream in Datei schreiben
    File f = LittleFS.open(path, FILE_WRITE);
    if (!f) { updateErr_ = "open temp failed"; http.end(); return false; }
    WiFiClient* stream = http.getStreamPtr();
    uint8_t buf[2048];
    int total = 0;
    int32_t remaining = http.getSize(); // -1 wenn unbekannt
    if (remaining > 0) {
      size_t total = LittleFS.totalBytes();
      size_t used = LittleFS.usedBytes();
      size_t freeB = (total > used) ? (total - used) : 0;
      // etwas Reserve (64KB) einplanen
      if ((size_t)remaining + 65536UL > freeB) {
        http.end();
        updatePhase_ = "error";
        updateErr_ = "insufficient FS space for TAR";
        return false;
      }
    }
    updateDLTotal_ = remaining;
    updateDLSoFar_ = 0;
    unsigned long lastRead = millis();
    for (;;) {
      size_t avail = stream->available();
      if (avail == 0) {
        if (!http.connected()) break;
        if (remaining == 0) break;
        if (millis() - lastRead > 15000) break; // Timeout 15s ohne Daten
        vTaskDelay(1);
        continue;
      }
      size_t nToRead = avail > sizeof(buf) ? sizeof(buf) : avail;
      int n = stream->readBytes((char*)buf, nToRead);
      if (n <= 0) break;
      lastRead = millis();
      if (f.write(buf, n) != (size_t)n) { updateErr_ = "fs write failed"; f.close(); http.end(); return false; }
      total += n;
      updateDLSoFar_ += n;
      if (updateDLTotal_ > 0) {
        int pct = (int)((100LL * updateDLSoFar_) / updateDLTotal_);
        updateMsg_ = String("download ") + String(updateDLSoFar_) + "/" + String(updateDLTotal_) + String(" (") + String(pct) + String("%)");
      } else {
        updateMsg_ = String("download ") + String(updateDLSoFar_) + String(" bytes");
      }
      if (remaining > 0) { remaining -= n; if (remaining <= 0) break; }
    }
    f.close();
    http.end();
    if (total > 0) { updateMsg_ = String("download complete ") + String(total) + String(" bytes"); }
    return total > 0;
  }
  return false;
}

bool WebCtrl::parseOctal_(const char* str, size_t n, uint32_t& out) {
  uint32_t v = 0; bool any=false; for (size_t i=0;i<n;i++){ char c=str[i]; if (c=='\0'||c==' ') break; if (c<'0'||c>'7') continue; any=true; v=(v<<3)+(uint32_t)(c-'0'); }
  out=v; return any;
}

bool WebCtrl::ensureDir_(const String& path) {
  int slash = 1; while (true) { slash = path.indexOf('/', slash); if (slash < 0) break; String dir = path.substring(0, slash); if (dir.length() > 0) LittleFS.mkdir(dir); slash++; }
  return true;
}

bool WebCtrl::isSafeAssetPath_(const String& name) {
  if (!name.length()) return false; if (name[0]=='/') return false; if (name.indexOf("..")>=0) return false; if (name.startsWith("cfg/")) return false; return true;
}

bool WebCtrl::handlePackageUploadEnd_(bool& fwUpdated, int& filesUpdated, String& err) {
  fwUpdated=false; filesUpdated=0; err="";
  if (!LittleFS.exists(pkgTempPath_.c_str())) { err="temp not found"; return false; }
  bool ok = applyPackageFromFile_(pkgTempPath_.c_str(), fwUpdated, filesUpdated, err); LittleFS.remove(pkgTempPath_.c_str()); return ok;
}

bool WebCtrl::applyPackageFromFile_(const char* tarPath, bool& fwUpdated, int& filesUpdated, String& err) {
  fwUpdated=false; filesUpdated=0; err="";
  File f = LittleFS.open(tarPath, FILE_READ); if (!f) { err="open failed"; return false; }
  const size_t BLK=512; uint8_t hdr[BLK];
  while (true) {
    size_t r = f.read(hdr, BLK); if (r == 0) break; if (r != BLK) { err="bad tar header"; f.close(); return false; }
    bool allZero=true; for (size_t i=0;i<BLK;i++){ if (hdr[i]!=0){ allZero=false; break; } } if (allZero) break;
    char nameC[101]; memcpy(nameC, hdr+0, 100); nameC[100]='\0'; String name=String(nameC);
    uint32_t size=0; if (!parseOctal_((const char*)(hdr+124), 12, size)) size=0; char typeflag=(char)hdr[156]; if (typeflag=='\0') typeflag='0';
    uint32_t toRead=size; uint32_t pad=(BLK-(size%BLK))%BLK;
    if (typeflag=='0') {
      if (name=="firmware.bin") {
        if (size==0) { err="empty firmware"; f.close(); return false; }
        if (!Update.begin((size_t)size, U_FLASH)) { err=String("Update.begin ")+Update.errorString(); f.close(); return false; }
        const size_t CH=2048; uint8_t buf[CH]; uint32_t left=toRead;
        while (left>0) { size_t n = left>CH?CH:left; int nr=f.read(buf,n); if (nr<=0){ Update.abort(); err="firmware read"; f.close(); return false; } size_t nw=Update.write(buf,nr); if (nw!=(size_t)nr){ Update.abort(); err=String("Update.write ")+Update.errorString(); f.close(); return false; } left-=nr; yield(); }
        if (!Update.end(true)) { err=String("Update.end ")+Update.errorString(); f.close(); return false; }
        fwUpdated=true; if (pad) f.seek(pad, SeekCur);
      } else if (name.startsWith("www/") && isSafeAssetPath_(name.substring(4))) {
        String rel=name.substring(4); String dest=String("/")+rel; String tmp=String("/.u_tmp_")+rel; ensureDir_(dest); int lastSlash=tmp.lastIndexOf('/'); if (lastSlash>0) { LittleFS.mkdir(tmp.substring(0,lastSlash)); }
        File wf=LittleFS.open(tmp, FILE_WRITE); if (!wf) { err=String("open fail ")+tmp; f.close(); return false; }
        const size_t CH=2048; uint8_t buf[CH]; uint32_t left=toRead;
        while (left>0) { size_t n=left>CH?CH:left; int nr=f.read(buf,n); if (nr<=0){ wf.close(); LittleFS.remove(tmp); err="asset read"; f.close(); return false; } if (wf.write(buf,nr)!=(size_t)nr){ wf.close(); LittleFS.remove(tmp); err="asset write"; f.close(); return false; } left-=nr; yield(); }
        wf.close(); LittleFS.remove(dest); ensureDir_(dest); if (!LittleFS.rename(tmp, dest)) { File rf=LittleFS.open(tmp, FILE_READ); File df=LittleFS.open(dest, FILE_WRITE); if (rf && df){ uint8_t b[1024]; int n; while ((n=rf.read(b,sizeof(b)))>0){ if (df.write(b,n)!=(size_t)n){ break; } } } if (rf) rf.close(); if (df) df.close(); LittleFS.remove(tmp); }
        filesUpdated++; if (pad) f.seek(pad, SeekCur);
      } else {
        if (toRead) f.seek(toRead, SeekCur); if (pad) f.seek(pad, SeekCur);
      }
    } else {
      if (toRead) f.seek(toRead, SeekCur); if (pad) f.seek(pad, SeekCur);
    }
  }
  f.close(); return true;
}

bool WebCtrl::loadUpdateCfg_(String& manifestUrl, String& ghOwner, String& ghRepo, String& ghAsset) {
  manifestUrl = ""; ghOwner = ""; ghRepo = ""; ghAsset = "";
  if (!LittleFS.exists(UPDATE_CFG_PATH)) return false;
  File f = LittleFS.open(UPDATE_CFG_PATH, FILE_READ); if (!f) return false;
  String s = f.readString(); f.close();
  JsonDocument doc; if (deserializeJson(doc, s)) return false;
  if (!doc["manifest_url"].isNull()) manifestUrl = String((const char*)doc["manifest_url"]);
  if (!doc["gh_owner"].isNull()) ghOwner = String((const char*)doc["gh_owner"]);
  if (!doc["gh_repo"].isNull())  ghRepo  = String((const char*)doc["gh_repo"]);
  if (!doc["gh_asset"].isNull()) ghAsset = String((const char*)doc["gh_asset"]);
  return (manifestUrl.length() + ghOwner.length() + ghRepo.length() + ghAsset.length()) > 0;
}

bool WebCtrl::saveUpdateCfg_(const String& manifestUrl, const String& ghOwner, const String& ghRepo, const String& ghAsset) {
  LittleFS.mkdir("/cfg");
  JsonDocument doc; doc["manifest_url"] = manifestUrl; doc["gh_owner"] = ghOwner; doc["gh_repo"] = ghRepo; doc["gh_asset"] = ghAsset; String out; serializeJson(doc, out);
  File f = LittleFS.open(UPDATE_CFG_PATH, FILE_WRITE); if (!f) return false; f.print(out); f.close(); return true;
}

String WebCtrl::manifestUrl_() {
  String u, o, r, a;
  if (strlen(UPDATE_MANIFEST_URL) > 0) return String(UPDATE_MANIFEST_URL);
  if (loadUpdateCfg_(u, o, r, a)) return u; return String("");
}

bool WebCtrl::ghLatestTarUrl_(String& outUrl, String& outLatestTag) {
  outUrl = ""; outLatestTag = "";
  // Wenn eine Manifest-URL fest definiert ist, bevorzugen wir diese nicht; GH hat Vorrang, falls Owner/Repo/Asset vorhanden.
  String man, owner, repo, asset;
  loadUpdateCfg_(man, owner, repo, asset);
  if (!owner.length() && strlen(GH_OWNER) > 0) owner = GH_OWNER;
  if (!repo.length()  && strlen(GH_REPO)  > 0) repo  = GH_REPO;
  if (!asset.length() && strlen(GH_ASSET) > 0) asset = GH_ASSET;
  if (!(owner.length() && repo.length() && asset.length())) return false;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  String api = String("https://api.github.com/repos/") + owner + "/" + repo + "/releases/latest";
  if (!http.begin(client, api)) return false;
  http.addHeader("User-Agent", "LuminaGrowX");
  int code = http.GET();
  if (code == 200) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (!err) {
      outLatestTag = String((const char*)doc["tag_name"]);
      // Construct stable latest-download URL with fixed asset name
      outUrl = String("https://github.com/") + owner + "/" + repo + "/releases/latest/download/" + asset;
    }
  }
  http.end();
  return outUrl.length() > 0;
}




String WebCtrl::fsInfoJson_() {
  JsonDocument doc;
  bool mounted = LittleFS.begin(false, "/littlefs", 10, "littlefs"); // mount using known label
  doc["mounted"] = mounted;
  size_t total = 0, used = 0;
  if (mounted) { total = LittleFS.totalBytes(); used = LittleFS.usedBytes(); }
  doc["total"] = (uint32_t)total;
  doc["used"] = (uint32_t)used;
  doc["free"] = (uint32_t)((total>used)?(total-used):0);
  // important files
  JsonObject files = doc["files"].to<JsonObject>();
  const char* keys[] = { "/index.html", "/update.html", "/pico.min.css", "/custom.css", "/bg.jpg" };
  for (size_t i=0;i<sizeof(keys)/sizeof(keys[0]);++i) files[keys[i]] = LittleFS.exists(keys[i]);
  String out; serializeJson(doc, out); return out;
}

String WebCtrl::fsListJson_(const char* path) {
  JsonDocument doc;
  JsonArray arr = doc["entries"].to<JsonArray>();
  bool mounted = LittleFS.begin(false, "/littlefs", 10, "littlefs");
  doc["mounted"] = mounted;
  doc["path"] = path;
  if (!mounted) { String out; serializeJson(doc, out); return out; }
  File dir = LittleFS.open(path);
  if (!dir || !dir.isDirectory()) { String out; serializeJson(doc, out); return out; }
  File f;
  while ((f = dir.openNextFile())) {
    JsonObject e = arr.add<JsonObject>();
    e["name"] = f.name();
    e["size"] = (uint32_t)f.size();
    e["isdir"] = f.isDirectory();
    f.close();
  }
  String out; serializeJson(doc, out); return out;
}

bool WebCtrl::fileExists_(const char* path) {
  bool mounted = LittleFS.begin(false, "/littlefs", 10, "littlefs");
  if (!mounted) { Serial.println(F("[FS] not mounted in fileExists_")); return false; }
  return LittleFS.exists(path);
}
