#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <time.h>

class RTC_Ctrl;

namespace net_ctrl {

struct NetworkConfig {
  String ssid;
  String pass;
  bool   useStatic = false;
  String ip;
  String mask = "255.255.255.0";
  String gw;
  String dns = "1.1.1.1";
};

class NetCtrl {
public:
  NetCtrl() {}

  // Blocking boot-time check for AP reset pin (default active LOW for 5s)
  void configureResetPin(int gpio, bool activeHigh = false, uint32_t holdMs = 5000);
  bool shouldForceAPAtBoot();

  // Start WiFi: AP is always started; optional STA connect using stored config unless forceAP
  void begin(bool forceAP, const char* mdnsName, RTC_Ctrl* rtc = nullptr, bool keepAP = false);

  // Try reconnecting STA immediately with stored cfg (non-blocking of main loop, but waits up to timeoutMs here)
  bool reconnectSTA(bool closeAPOnSuccess = true, uint32_t timeoutMs = 12000);

  // Non-blocking loop task to check reset pin for long-press
  void tick();

  // AP control
  void stopAP() { WiFi.softAPdisconnect(true); apActive_ = false; }
  void startAP();

  // Config persistence (LittleFS JSON)
  bool loadConfig(NetworkConfig& out);
  bool saveConfig(const NetworkConfig& cfg);

  // Internet probe (204 endpoint)
  bool probeInternet();

  // Status accessors
  const NetworkConfig& cfg() const { return cfg_; }
  bool isConnected() const { return WiFi.isConnected(); }
  int  rssi() const { return isConnected() ? WiFi.RSSI() : -127; }
  IPAddress staIP() const { return isConnected() ? WiFi.localIP() : IPAddress(0,0,0,0); }
  IPAddress apIP() const { return WiFi.softAPIP(); }
  const String& apSSID() const { return apSSID_; }
  bool internetOK() const { return internetOK_; }
  bool apActive() const { return apActive_; }

private:
  static bool parseIPv4(const String& s, IPAddress& out) { return out.fromString(s); }
  static void tzInit();
  void ntpInit();
  void trySetRTCOnce_();

private:
  NetworkConfig cfg_;
  String apSSID_;
  bool   apActive_ = false;
  bool   mdnsOk_ = false;
  bool   internetOK_ = false;
  RTC_Ctrl* rtc_ = nullptr;
  bool rtcSetOnce_ = false;

  String mdnsName_;

  int  resetPin_ = -1;
  bool resetActiveHigh_ = false;
  uint32_t resetHoldMs_ = 5000;
  uint32_t resetPinPressStart_ = 0;
  uint32_t apAutoCloseAt_ = 0;
};

} // namespace net_ctrl
