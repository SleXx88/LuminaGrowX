#include "web_ctrl.h"
#include "rtc_ctrl.h"
#include "plant_ctrl.h"
#include "net_ctrl.h"
#include <WiFi.h>

using namespace web_ctrl;
using net_ctrl::NetCtrl;

static const char* APP_CFG_PATH   = "/cfg/app.json";
static const char* GROW_CFG_PATH  = "/cfg/grow.json";
static const char* NOTIFY_CFG_PATH= "/cfg/notify.json";
static const char* FW_VERSION     = "1.0.0";

static bool readTextFile(const char* path, String& out) {
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

  http_.on("/api/info", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", makeInfoJson_()); });
  http_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", makeStatusJson_()); });

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
               if (!ntf["apikey"].isNull())  n.apikey  = String((const char*)ntf["apikey"]);
               saveNotify(n);
             }
             bool requestedReboot = doc["reboot"].as<bool>();
             if (net_) net_->saveConfig(nc);
             bool connectedNow = false;
             if (anyNetFieldTouched && net_) {
               // Sofortiger Verbindungsversuch ohne Reboot
               connectedNow = net_->reconnectSTA(true /*close AP on success*/);
             }
             // Wenn erfolgreich direkt verbunden: Reboot unterdrücken, sonst optional rebooten
             bool doReboot = (!connectedNow) && requestedReboot;
             req->send(200, "application/json", "{\"ok\":true}");
             if (doReboot) rebootAt_ = millis() + 800;
           });

  // standalone notify save
  http_.on("/api/notify", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             NotifyCfg n = notify_;
             if (!doc["enabled"].isNull()) n.enabled = doc["enabled"].as<bool>();
             if (!doc["phone"].isNull())   n.phone   = String((const char*)doc["phone"]);
             if (!doc["apikey"].isNull())  n.apikey  = String((const char*)doc["apikey"]);
             saveNotify(n);
             req->send(200, "application/json", "{\"ok\":true}");
           });

  http_.on("/api/notify/test", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
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
                          String(" | T ") + String(t,1) + String("°C, RH ") + String(rh,1) + String("%, VPD ") + String(vpd,2) + String(" kPa") +
                          String(" | LED ") + String(led,0) + String("%, Fan ") + String(fan,0) + String("%");
             bool ok = whatsappSend_(phone, key, msg);
             if (ok) req->send(200, "application/json", "{\"ok\":true}"); else req->send(500, "application/json", "{\"error\":\"send failed\"}");
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
  if (LittleFS.exists(path)) req->send(LittleFS, path, mime); else req->send(404, "text/plain", "Not found");
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
  float tIn = NAN, rhIn = NAN; double vpd = NAN;
  if (ctrl_) {
    tIn = (float)ctrl_->currentTemp();
    rhIn = (float)ctrl_->currentRh();
    vpd = ctrl_->currentVpd();
  }
  auto round1 = [](float x){ return isnan(x)? NAN : (float)roundf(x*10.f)/10.f; };
  doc["temp_c"] = round1(tIn);
  doc["humi_rh"] = round1(rhIn);
  doc["vpd_kpa"] = round1((float)vpd);
  // Actuators
  float ledPct = ctrl_ ? ctrl_->currentLedPercentEffective() : 0.0f;
  float fanPct = ctrl_ ? ctrl_->currentFanPercent() : 0.0f;
  doc["light_on"] = ledPct > 0.5f;
  doc["fan_on"]   = fanPct > 0.5f;
  doc["pump_on"]  = false; // kein Pumpen-State integriert

  JsonObject health = doc["health"].to<JsonObject>();
  health["state"] = "ok";
  health["message"] = "";

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
  if (day > 0) { if (day <= 14) phase = "Keimung/Seedling"; else if (day <= 35) phase = "Vegetationsphase"; else phase = "Blütephase"; }
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
  if (!net_ || !net_->isConnected()) return false;
  HTTPClient http;
  String url = String("https://api.callmebot.com/whatsapp.php?phone=") + urlEncode_(phone) + "&text=" + urlEncode_(message) + "&apikey=" + urlEncode_(apikey);
  bool ok=false; if (http.begin(url)) { int code=http.GET(); ok=(code==200); http.end(); }
  return ok;
}