#include "net_ctrl.h"
#include <Preferences.h>
#include "rtc_ctrl.h"

using namespace net_ctrl;

// NVS Namespace für Netzwerkdaten
const char* kNetNamespace = "network";

bool NetCtrl::loadConfig(NetworkConfig& out) {
  Preferences prefs;
  if (!prefs.begin(kNetNamespace, true)) return false;
  out.ssid = prefs.getString("ssid", "");
  out.pass = prefs.getString("pass", "");
  out.hostname = prefs.getString("hostname", "");
  out.useStatic = prefs.getBool("use_static", false);
  out.ip = prefs.getString("ip", "");
  out.mask = prefs.getString("mask", "255.255.255.0");
  out.gw = prefs.getString("gw", "");
  out.dns = prefs.getString("dns", "1.1.1.1");
  prefs.end();
  cfg_ = out;
  return true;
}

bool NetCtrl::saveConfig(const NetworkConfig& c) {
  Preferences prefs;
  if (!prefs.begin(kNetNamespace, false)) return false;
  prefs.putString("ssid", c.ssid);
  prefs.putString("pass", c.pass);
  prefs.putString("hostname", c.hostname);
  prefs.putBool("use_static", c.useStatic);
  prefs.putString("ip", c.ip);
  prefs.putString("mask", c.mask);
  prefs.putString("gw", c.gw);
  prefs.putString("dns", c.dns);
  prefs.end();
  cfg_ = c;
  return true;
}

void NetCtrl::tzInit() {
  // POSIX TZ string for Europe/Berlin (CET/CEST)
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
}

void NetCtrl::ntpInit() {
  // Set TZ and NTP servers in one call to ensure localtime_r uses DST correctly
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
}

void NetCtrl::configureResetPin(int gpio, bool activeHigh, uint32_t holdMs, uint32_t factoryHoldMs) {
  resetPin_ = gpio;
  resetActiveHigh_ = activeHigh;
  resetHoldMs_ = holdMs;
  factoryResetHoldMs_ = factoryHoldMs;
  if (resetPin_ >= 0) {
    pinMode(resetPin_, resetActiveHigh_ ? INPUT : INPUT_PULLUP);
  }
}

bool NetCtrl::shouldForceAPAtBoot() {
  if (resetPin_ < 0) return false;
  uint32_t t0 = millis();
  bool active = false;
  while (millis() - t0 < resetHoldMs_) {
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

void NetCtrl::begin(bool forceAP, const char* mdnsName, RTC_Ctrl* rtc, bool keepAP) {
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

  // AP SSID: LisaPro-Setup-<last4 mac>
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac = mac.substring(mac.length() - 4);
  apSSID_ = String("LisaPro-Setup-") + mac;

  // Hostname für DHCP (Fritzbox) und mDNS: Priorität auf gespeicherte Config, sonst Default Name + MAC
  if (cfg_.hostname.length() > 0) {
    mdnsName_ = cfg_.hostname;
  } else {
    String base = (mdnsName && strlen(mdnsName) > 0) ? String(mdnsName) : String("LisaPro");
    mdnsName_ = base + "-" + mac;
  }
  WiFi.setHostname(mdnsName_.c_str());

  // Statische AP-IP sicherstellen (Standard 192.168.4.1)
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  // Offener AP ohne Passwort, sichtbare SSID, Kanal 1
  WiFi.softAP(apSSID_.c_str(), "", 1, 0);
  apActive_ = true;
  Serial.printf("[NET] AP gestartet (offen): SSID=%s, IP=%s\n", apSSID_.c_str(), WiFi.softAPIP().toString().c_str());
  Serial.printf("[NET] Hostname gesetzt auf: %s\n", mdnsName_.c_str());

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
    if (mdnsName_.length() > 0 && MDNS.begin(mdnsName_.c_str())) {
      MDNS.addService("http", "tcp", 80);
      mdnsOk_ = true;
    }
    // Deaktiviere den Access Point, sobald STA verbunden ist - AUSSER wir sind im Setup/keepAP
    if (!keepAP) {
      WiFi.softAPdisconnect(true);
      apActive_ = false;
      Serial.printf("[NET] STA verbunden: IP=%s (AP deaktiviert)\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.printf("[NET] STA verbunden: IP=%s (AP bleibt AKTIV)\n", WiFi.localIP().toString().c_str());
    }
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
  
  // Sicherstellen, dass wir im AP+STA Modus sind
  WiFi.mode(WIFI_AP_STA);
  
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac = mac.substring(mac.length() - 4);
  apSSID_ = String("LisaPro-Setup-") + mac;

  // Statische AP-IP sicherstellen (Standard 192.168.4.1)
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  // Offener AP ohne Passwort, sichtbare SSID, Kanal 1
  WiFi.softAP(apSSID_.c_str(), "", 1, 0);
  
  apActive_ = true;
  Serial.printf("[NET] AP manuell gestartet: SSID=%s, IP=%s\n", apSSID_.c_str(), WiFi.softAPIP().toString().c_str());
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

NetCtrl::ResetEvent NetCtrl::tick() {
  if (resetPin_ < 0) return ResetEvent::NONE;

  // Auto-Close Logik für AP (falls über Button gestartet)
  if (apAutoCloseAt_ > 0 && millis() >= apAutoCloseAt_) {
    if (apActive_) {
      Serial.println(F("[NET] AP-Zeitlimit (1 Min) abgelaufen -> Schließe AP."));
      stopAP();
    }
    apAutoCloseAt_ = 0;
  }
  
  // Wir lesen den Pin. Falls activeHigh, ist HIGH = aktiv, sonst LOW = aktiv.
  bool level = digitalRead(resetPin_);
  bool isActive = resetActiveHigh_ ? level : !level;

  ResetEvent event = ResetEvent::NONE;

  if (isActive) {
    if (resetPinPressStart_ == 0) {
      resetPinPressStart_ = millis();
      apTriggeredThisPress_ = false;
      factoryTriggeredThisPress_ = false;
    } else {
      uint32_t holdTime = millis() - resetPinPressStart_;

      // 10 Sekunden: Factory Reset
      if (holdTime >= factoryResetHoldMs_ && !factoryTriggeredThisPress_) {
        Serial.println(F("[NET] Reset-Button 10s gehalten -> Starte Factory Reset!"));
        factoryTriggeredThisPress_ = true;
        event = ResetEvent::FACTORY_RESET;
      }
      // 5 Sekunden: AP Reset
      else if (holdTime >= resetHoldMs_ && !apTriggeredThisPress_ && !factoryTriggeredThisPress_) {
        if (!apActive_) {
          Serial.println(F("[NET] Reset-Button 5s gehalten -> AP für 1 Minute aktiv."));
          startAP();
          apAutoCloseAt_ = millis() + 60000; // 60 Sekunden Timer
        } else {
          Serial.println(F("[NET] Reset-Button 5s gehalten -> AP ist bereits aktiv."));
        }
        apTriggeredThisPress_ = true;
        event = ResetEvent::AP_RESET;
      }
    }
  } else {
    resetPinPressStart_ = 0;
    apTriggeredThisPress_ = false;
    factoryTriggeredThisPress_ = false;
  }

  return event;
}
