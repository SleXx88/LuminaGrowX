#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>

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
struct DryingState { bool active=false; uint32_t start_epoch=0; };
struct NotifyCfg { bool enabled=false; String phone; String apikey; };
struct StepperCfg { float max_travel_mm = 440.0f; };

class WebCtrl {
public:
  WebCtrl();

  void begin(plant_ctrl::PlantCtrl* ctrl,
             RTC_Ctrl* rtc,
             net_ctrl::NetCtrl* net);

  // Optionale Hardware-Referenzen für Setup/Tests (LED-DAC, Lüfter, Stepper, ToF)
  void setHardware(GP8211Ctrl* dac, FanCtrl* fan, FanCtrl* fan2, FanCtrl* fan3, StepperCtrl* stepper, ToFCtrl* tof, SHT41Ctrl* shtIn = nullptr, SHT41Ctrl* shtOut = nullptr);

  void loop(); // broadcast status, reboot scheduling, probes

  // Config IO
  bool loadAppCfg(AppCfg& out);
  bool saveAppCfg(const AppCfg& c);
  bool loadGrow(GrowState& out);
  bool saveGrow(const GrowState& g);
  bool loadDrying(DryingState& out);
  bool saveDrying(const DryingState& d);
  bool loadNotify(NotifyCfg& out);
  bool saveNotify(const NotifyCfg& n);
  bool loadStepperCfg(StepperCfg& out);
  bool saveStepperCfg(const StepperCfg& c);

private:
  // HTTP handlers
  void setupRoutes_();
  void registerUpdateRoutes_();
  void sendFile_(AsyncWebServerRequest* req, const char* path, const char* mime);
  String makeStatusJson_();
  String makeInfoJson_();
  static String urlEncode_(const String& s);
  String makeSetupStatusJson_();
  
  // Update helpers (TAR-based package with optional firmware.bin and www/* assets)
  bool handlePackageUploadBegin_();
  bool handlePackageUploadWrite_(const uint8_t* data, size_t len);
  bool handlePackageUploadEnd_(bool& fwUpdated, int& filesUpdated, String& err);
  bool applyPackageFromFile_(const char* tarPath, bool& fwUpdated, int& filesUpdated, String& err);
  bool downloadToFile_(const String& url, const char* path);
  bool parseOctal_(const char* str, size_t n, uint32_t& out);
  bool ensureDir_(const String& path);
  bool isSafeAssetPath_(const String& name);
  // Update config (manifest URL)
  bool loadUpdateCfg_(String& manifestUrl, String& ghOwner, String& ghRepo, String& ghAsset);
  bool saveUpdateCfg_(const String& manifestUrl, const String& ghOwner, const String& ghRepo, const String& ghAsset);
  String manifestUrl_();
  bool ghLatestTarUrl_(String& outUrl, String& outLatestTag);

  // FS diagnostics
  String fsInfoJson_();
  String fsListJson_(const char* path);
  bool fileExists_(const char* path);

  // Async update job
  void startRemoteUpdateJob_();
  void startApplyUploadedJob_();
  static void updateTaskTrampoline_(void* arg);
  void updateTaskRun_(bool remote, const String& localTarPath);
  String updateProgressJson_();

  // WhatsApp
  bool whatsappSend_(const String& phone, const String& apikey, const String& message);

  void checkStepperCalibration_();

private:
  plant_ctrl::PlantCtrl* ctrl_ = nullptr;
  RTC_Ctrl* rtc_ = nullptr;
  net_ctrl::NetCtrl* net_ = nullptr;
  GP8211Ctrl* dac_ = nullptr;
  FanCtrl* fan_ = nullptr;
  FanCtrl* fan2_ = nullptr;
  FanCtrl* fan3_ = nullptr;
  StepperCtrl* step_ = nullptr;
  ToFCtrl* tof_ = nullptr;
  SHT41Ctrl* shtIn_ = nullptr;
  SHT41Ctrl* shtOut_ = nullptr;

  AsyncWebServer http_{80};
  AsyncWebSocket ws_{"/ws"};

  AppCfg app_;
  GrowState grow_;
  DryingState drying_;
  NotifyCfg notify_;
  StepperCfg stepper_cfg_;

  uint32_t nextPushAt_ = 0;
  uint32_t nextProbeAt_ = 0;
  uint32_t rebootAt_ = 0;
  // RTC sync
  uint32_t lastRtcSyncYMD_ = 0; // YYYYMMDD of last RTC sync
  bool rtcSyncedOnce_ = false;
  bool syncRTCFromSystem_();
  bool syncSystemFromRTC_();

  // Temporary file for package upload/download
  String pkgTempPath_ = "/.update_pkg.tar";

  // Update job state
  bool updateJobRunning_ = false;
  String updatePhase_ = "idle"; // idle|checking|downloading|applying|done|error
  String updateMsg_;
  bool updateOk_ = false;
  bool updateFwUpdated_ = false;
  int updateFilesUpdated_ = 0;
  String updateErr_;
  // Progress counters
  int32_t updateDLTotal_ = -1;
  int32_t updateDLSoFar_ = 0;
  int32_t updateApplyTotal_ = -1;
  int32_t updateApplySoFar_ = 0;

  // Background: daily auto-check for updates (03:00)
  bool updateCheckJobRunning_ = false;
  bool latestKnownHasUpdate_ = false;
  String latestKnownTag_;
  uint32_t lastUpdateCheckYMD_ = 0; // YYYYMMDD of last successful/attempted check
  bool firstUpdateCheckDone_ = false;
  static void updateCheckTaskTrampoline_(void* arg);
  void updateCheckTaskRun_();
};

} // namespace web_ctrl

