#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

class RTC_Ctrl;
class SHT41Ctrl;
class GP8211Ctrl;
class FanCtrl;
class StepperCtrl;
class ToFCtrl;
namespace plant_ctrl { class PlantCtrl; }
namespace net_ctrl { class NetCtrl; }

namespace web_ctrl {

struct AppCfg { String seed = "Northern Lights"; };
struct GrowState { bool started=false; uint32_t start_epoch=0; uint16_t total_days=90; };
struct NotifyCfg { bool enabled=false; String phone; String apikey; };

class WebCtrl {
public:
  WebCtrl();

  void begin(plant_ctrl::PlantCtrl* ctrl,
             RTC_Ctrl* rtc,
             net_ctrl::NetCtrl* net);

  void loop(); // broadcast status, reboot scheduling, probes

  // Config IO
  bool loadAppCfg(AppCfg& out);
  bool saveAppCfg(const AppCfg& c);
  bool loadGrow(GrowState& out);
  bool saveGrow(const GrowState& g);
  bool loadNotify(NotifyCfg& out);
  bool saveNotify(const NotifyCfg& n);

private:
  // HTTP handlers
  void setupRoutes_();
  void sendFile_(AsyncWebServerRequest* req, const char* path, const char* mime);
  String makeStatusJson_();
  String makeInfoJson_();
  static String urlEncode_(const String& s);

  // WhatsApp
  bool whatsappSend_(const String& phone, const String& apikey, const String& message);

private:
  plant_ctrl::PlantCtrl* ctrl_ = nullptr;
  RTC_Ctrl* rtc_ = nullptr;
  net_ctrl::NetCtrl* net_ = nullptr;

  AsyncWebServer http_{80};
  AsyncWebSocket ws_{"/ws"};

  AppCfg app_;
  GrowState grow_;
  NotifyCfg notify_;

  uint32_t nextPushAt_ = 0;
  uint32_t nextProbeAt_ = 0;
  uint32_t rebootAt_ = 0;
};

} // namespace web_ctrl

