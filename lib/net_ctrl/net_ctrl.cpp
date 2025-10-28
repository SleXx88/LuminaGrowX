#include "net_ctrl.h"
#include <ArduinoJson.h>
#include "rtc_ctrl.h"

using namespace net_ctrl;

static const char* NET_CFG_PATH = "/cfg/network.json";

static bool readTextFile(const char* path, String& out) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return false;
  out = f.readString();
  f.close();
  return true;
}

static bool writeTextFile(const char* path, const String& data) {
  // ensure directory exists
  LittleFS.mkdir("/cfg");
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) return false;
  f.print(data);
  f.close();
  return true;
}

bool NetCtrl::loadConfig(NetworkConfig& out) {
  String s;
  if (!readTextFile(NET_CFG_PATH, s)) return false;
  JsonDocument doc;
  if (deserializeJson(doc, s)) return false;
  out.ssid = String((const char*)doc["ssid"]);
  out.pass = String((const char*)doc["pass"]);
  out.useStatic = doc["use_static"].as<bool>();
  if (!doc["static"].isNull()) {
    JsonObject st = doc["static"].as<JsonObject>();
    out.ip   = String((const char*)st["ip"]);
    out.mask = String((const char*)st["mask"]);
    out.gw   = String((const char*)st["gw"]);
    out.dns  = String((const char*)st["dns"]);
  }
  cfg_ = out;
  return true;
}

bool NetCtrl::saveConfig(const NetworkConfig& c) {
  JsonDocument doc;
  doc["ssid"] = c.ssid;
  doc["pass"] = c.pass;
  doc["use_static"] = c.useStatic;
  JsonObject st = doc["static"].to<JsonObject>();
  st["ip"]   = c.ip;
  st["mask"] = c.mask;
  st["gw"]   = c.gw;
  st["dns"]  = c.dns;
  String out;
  serializeJson(doc, out);
  bool ok = writeTextFile(NET_CFG_PATH, out);
  if (ok) cfg_ = c;
  return ok;
}

void NetCtrl::tzInit() {
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
}

void NetCtrl::ntpInit() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

bool NetCtrl::shouldForceAPAtBoot(uint32_t holdMs) {
  if (resetPin_ < 0) return false;
  pinMode(resetPin_, resetActiveHigh_ ? INPUT : INPUT_PULLUP);
  uint32_t t0 = millis();
  bool active = false;
  while (millis() - t0 < holdMs) {
    bool level = digitalRead(resetPin_);
    bool isActive = resetActiveHigh_ ? level : !level;
    if (!isActive) {
      // abort if not held continuously
      return false;
    }
    delay(5);
    active = true;
  }
  return active;
}

void NetCtrl::trySetRTCOnce_() {
  if (rtcSetOnce_ || !rtc_) return;
  time_t now;
  time(&now);
  if (now < 1700000000) return; // not set yet
  // ensure RTC is initialized and connected
  // best-effort: only proceed if RTC was begun and is connected
  if (!rtc_->isConnected()) return;
  // format local time as "DD-MM-YYYY HH-MM-SS"
  struct tm tmL;
  localtime_r(&now, &tmL);
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d %02d-%02d-%02d",
           tmL.tm_mday, tmL.tm_mon + 1, tmL.tm_year + 1900,
           tmL.tm_hour, tmL.tm_min, tmL.tm_sec);
  rtc_->writeTimeFromString(String(buf));
  rtcSetOnce_ = true;
}

void NetCtrl::begin(bool forceAP, const char* mdnsName, RTC_Ctrl* rtc) {
  rtc_ = rtc;
  mdnsName_ = mdnsName ? String(mdnsName) : String();
  tzInit();
  ntpInit();

  // Load config if present
  NetworkConfig tmp;
  loadConfig(tmp);

  // Verhindere ungewolltes Auto-Connect mit alten, persistenten STA-Credentials
  // (sonst wuerde der AP sofort wieder gestoppt, wenn STA automatisch verbindet)
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(true);
  // Stelle sicher, dass kein altes Autokonfig-Connect aktiv ist
  WiFi.disconnect(false, true);

  WiFi.mode(WIFI_AP_STA);
  delay(50);
  // Sicherstellen, dass kein auto-gestarteter AP vom Core aktiv bleibt
  WiFi.softAPdisconnect(true);
  delay(20);
  // Maximale Sendeleistung und kein Leichtschlaf, damit AP stabil sichtbar ist
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  // AP SSID: LuminaGrowX-Setup-<last4 mac>
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac = mac.substring(mac.length() - 4);
  apSSID_ = String("LuminaGrowX-Setup-") + mac;
  // Statische AP-IP sicherstellen (Standard 192.168.4.1)
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  // Offener AP ohne Passwort, sichtbare SSID, Kanal 1
  WiFi.softAP(apSSID_.c_str(), "", 1, 0);
  apActive_ = true;
  Serial.printf("[NET] AP gestartet (offen): SSID=%s, IP=%s\n", apSSID_.c_str(), WiFi.softAPIP().toString().c_str());

  if (!forceAP && cfg_.ssid.length()) {
    if (cfg_.useStatic) {
      IPAddress lip, lgw, lms, ldn;
      parseIPv4(cfg_.ip, lip);
      parseIPv4(cfg_.gw, lgw);
      parseIPv4(cfg_.mask, lms);
      parseIPv4(cfg_.dns, ldn);
      if (lip && lgw && lms) WiFi.config(lip, lgw, lms, ldn);
    }
    WiFi.begin(cfg_.ssid.c_str(), cfg_.pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
      delay(250);
    }
  }

  // STA-Verbindung herstellen (falls konfiguriert); AP bei Erfolg deaktivieren
  if (WiFi.isConnected() && !forceAP && cfg_.ssid.length() > 0) {
    if (mdnsName && MDNS.begin(mdnsName)) {
      MDNS.addService("http", "tcp", 80);
      mdnsOk_ = true;
    }
    // Deaktiviere den Access Point, sobald STA verbunden ist
    WiFi.softAPdisconnect(true);
    apActive_ = false;
    Serial.printf("[NET] STA verbunden: IP=%s (AP deaktiviert)\n", WiFi.localIP().toString().c_str());
  }

  internetOK_ = probeInternet();
  trySetRTCOnce_();
}

bool NetCtrl::probeInternet() {
  if (!WiFi.isConnected()) {
    internetOK_ = false;
    return false;
  }
  HTTPClient http;
  bool ok = false;
  if (http.begin("http://clients3.google.com/generate_204")) {
    int code = http.GET();
    ok = (code == 204 || code == 200);
    http.end();
  }
  internetOK_ = ok;
  return ok;
}

void NetCtrl::startAP() {
  if (apActive_) return;
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac = mac.substring(mac.length() - 4);
  apSSID_ = String("LuminaGrowX-Setup-") + mac;
  WiFi.softAP(apSSID_.c_str());
  apActive_ = true;
}

bool NetCtrl::reconnectSTA(bool closeAPOnSuccess, uint32_t timeoutMs) {
  // try connecting to STA with current cfg_
  if (cfg_.ssid.length() == 0) return false;
  if (cfg_.useStatic) {
    IPAddress lip, lgw, lms, ldn;
    parseIPv4(cfg_.ip, lip);
    parseIPv4(cfg_.gw, lgw);
    parseIPv4(cfg_.mask, lms);
    parseIPv4(cfg_.dns, ldn);
    if (lip && lgw && lms) WiFi.config(lip, lgw, lms, ldn);
  }
  WiFi.disconnect(false, false);
  WiFi.begin(cfg_.ssid.c_str(), cfg_.pass.c_str());
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);
  }
  if (WiFi.isConnected()) {
    if (mdnsName_.length() && MDNS.begin(mdnsName_.c_str())) {
      MDNS.addService("http", "tcp", 80);
      mdnsOk_ = true;
    }
    if (closeAPOnSuccess) { WiFi.softAPdisconnect(true); apActive_ = false; }
    internetOK_ = probeInternet();
    trySetRTCOnce_();
    return true;
  }
  return false;
}
