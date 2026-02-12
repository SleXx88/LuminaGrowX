#include "web_ctrl.h"
#include "rtc_ctrl.h"
#include "plant_ctrl.h"
#include "net_ctrl.h"
#include "../../include/health.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <Preferences.h>
#include "../../include/version.h"
#include "../../include/lumina_config.h"
#include "../../include/setup_flag.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "stepper_ctrl.h"
#include "tof_ctrl.h"
#include "sht41_ctrl.h"

using namespace web_ctrl;
using net_ctrl::NetCtrl;

// NVS Namespaces
static const char* NVS_APP = "app";
static const char* NVS_GROW = "grow";
static const char* NVS_DRY = "drying";
static const char* NVS_NOTIFY = "notify";
static const char* NVS_STEPPER = "stepper";
static const char* NVS_TOF = "tof";
static const char* NVS_UPDATE = "update_cfg";
static const char* NVS_PHASES = "cfg_phases";
static const char* NVS_MQTT = "mqtt";

// GitHub Releases (latest) defaults â€“ fest im Code hinterlegt
static const char* GH_OWNER = "SleXx88";
static const char* GH_REPO  = "LuminaGrowX";
static const char* GH_ASSET = "LuminaGrowX_Update.tar";
static const char* UPDATE_MANIFEST_URL = ""; // leer -> aus NVS lesen

WebCtrl::WebCtrl() : targetDacPct_(0.0f), currentDacPct_(0.0f), lastDacFadeMs_(0) {}

bool WebCtrl::loadAppCfg(AppCfg& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_APP, true)) return false;
  if (prefs.isKey("seed")) out.seed = prefs.getString("seed");
  else out.seed = "Northern Lights";
  if (prefs.isKey("name")) out.name = prefs.getString("name");
  else out.name = "LisaPro";
  prefs.end();
  app_ = out; return true;
}
bool WebCtrl::saveAppCfg(const AppCfg& c) {
  Preferences prefs;
  if (!prefs.begin(NVS_APP, false)) return false;
  prefs.putString("seed", c.seed);
  prefs.putString("name", c.name);
  prefs.end();
  app_ = c;
  Serial.printf("[WEB] App config saved to NVS (Name: %s, Seed: %s)\n", c.name.c_str(), c.seed.c_str());
  return true;
}
bool WebCtrl::loadGrow(GrowState& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_GROW, true)) return false;
  out.started = prefs.getBool("started", false);
  out.start_epoch = prefs.getUInt("start_epoch", 0);
  out.total_days = (uint16_t)prefs.getUInt("total_days", 90);
  prefs.end();
  grow_ = out; return true;
}
bool WebCtrl::saveGrow(const GrowState& g) {
  Preferences prefs;
  if (!prefs.begin(NVS_GROW, false)) return false;
  prefs.putBool("started", g.started);
  prefs.putUInt("start_epoch", g.start_epoch);
  prefs.putUInt("total_days", (uint32_t)g.total_days);
  prefs.end();
  grow_ = g; return true;
}
bool WebCtrl::loadDrying(DryingState& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_DRY, true)) return false;
  out.active = prefs.getBool("active", false);
  out.start_epoch = prefs.getUInt("start_epoch", 0);
  prefs.end();
  drying_ = out; return true;
}
bool WebCtrl::saveDrying(const DryingState& d) {
  Preferences prefs;
  if (!prefs.begin(NVS_DRY, false)) return false;
  prefs.putBool("active", d.active);
  prefs.putUInt("start_epoch", d.start_epoch);
  prefs.end();
  drying_ = d; return true;
}
bool WebCtrl::loadNotify(NotifyCfg& out) {
  Preferences prefs;
  if (!prefs.begin(NVS_NOTIFY, true)) return false;
  out.enabled = prefs.getBool("enabled", false);
  if (prefs.isKey("phone")) out.phone = prefs.getString("phone");
  else out.phone = "";
  if (prefs.isKey("apikey")) out.apikey = prefs.getString("apikey");
  else out.apikey = "";
  prefs.end();
  notify_ = out; return true;
}
bool WebCtrl::saveNotify(const NotifyCfg& n) {
  Preferences prefs;
  if (!prefs.begin(NVS_NOTIFY, false)) return false;
  prefs.putBool("enabled", n.enabled);
  prefs.putString("phone", n.phone);
  prefs.putString("apikey", n.apikey);
  prefs.end();
  notify_ = n; return true;
}

void WebCtrl::syncControllerStage_() {
  if (!ctrl_ || !grow_.started || !grow_.start_epoch) return;
  time_t now = time(nullptr);
  if (now < 1700000000) return; // Time not synced yet
  
  uint32_t day = 0;
  if (now >= grow_.start_epoch) {
    day = (uint32_t)((now - grow_.start_epoch)/86400UL) + 1;
    if (day > grow_.total_days) day = grow_.total_days;
  }
  
  vpd_calc::GrowthStage stage;
  if (day <= 14) stage = vpd_calc::GrowthStage::Seedling;
  else if (day <= 35) stage = vpd_calc::GrowthStage::Vegetative;
  else stage = vpd_calc::GrowthStage::Flowering;
  
  if (ctrl_->getStage() != stage) {
    ctrl_->setStage(stage);
    Serial.printf("[WEB] Growth phase synced to day %u: %s\n", (unsigned)day, 
      (stage==vpd_calc::GrowthStage::Seedling)?"Seedling":((stage==vpd_calc::GrowthStage::Vegetative)?"Veg":"Flower"));
  }
}

bool WebCtrl::loadStepperCfg(StepperCfg& out) {

    Preferences prefs;
    if (!prefs.begin(NVS_STEPPER, true)) return false;
    out.max_travel_mm = prefs.getFloat("max_travel_mm", 440.0f);
    prefs.end();
    stepper_cfg_ = out; return true;
}
bool WebCtrl::saveStepperCfg(const StepperCfg& c) {
    Preferences prefs;
    if (!prefs.begin(NVS_STEPPER, false)) return false;
    prefs.putFloat("max_travel_mm", c.max_travel_mm);
    prefs.end();
    stepper_cfg_ = c; return true;
}

bool WebCtrl::loadTofCfg(ToFCfg& out) {
    Preferences prefs;
    if (!prefs.begin(NVS_TOF, true)) return false;
    out.offset_mm = (int16_t)prefs.getShort("offset_mm", 0);
    prefs.end();
    tof_cfg_ = out; 
    return true;
}
bool WebCtrl::saveTofCfg(const ToFCfg& c) {
    Preferences prefs;
    if (!prefs.begin(NVS_TOF, false)) return false;
    prefs.putShort("offset_mm", (int16_t)c.offset_mm);
    prefs.end();
    return true;
}

void WebCtrl::setMqtt(MqttCtrl* mqtt) {
    mqtt_ = mqtt;
    if (mqtt_) {
        // Apply currently loaded config
        mqtt_->begin(mqtt_cfg_, app_.name);
    }
}

bool WebCtrl::loadMqttCfg(MqttConfig& out) {
    Preferences prefs;
    if (!prefs.begin(NVS_MQTT, true)) return false;
    out.enabled = prefs.getBool("enabled", false);
    out.server = prefs.getString("server", "");
    out.port = (uint16_t)prefs.getUInt("port", 1883);
    out.user = prefs.getString("user", "");
    out.pass = prefs.getString("pass", "");
    prefs.end();
    mqtt_cfg_ = out;
    return true;
}

bool WebCtrl::saveMqttCfg(const MqttConfig& c) {
    Preferences prefs;
    if (!prefs.begin(NVS_MQTT, false)) return false;
    prefs.putBool("enabled", c.enabled);
    prefs.putString("server", c.server);
    prefs.putUInt("port", c.port);
    prefs.putString("user", c.user);
    prefs.putString("pass", c.pass);
    prefs.end();
    mqtt_cfg_ = c;
    if (mqtt_) mqtt_->setConfig(c);
    return true;
}

#pragma pack(push, 1)
struct PhaseBlob {
  plant_ctrl::LightSchedule schedule;
  plant_ctrl::PhaseModeSettings modes[3];
};
#pragma pack(pop)

bool WebCtrl::loadPhases(plant_ctrl::PlantCtrl* ctrl) {
  if (!ctrl) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_PHASES, true)) {
    Serial.println(F("[WEB] loadPhases: Preferences.begin(PHASES, true) FAILED"));
    return false;
  }
  
  bool any = false;
  for (int i=0; i<3; ++i) {
    char key[8]; snprintf(key, sizeof(key), "p%d", i);
    if (prefs.isKey(key)) {
      PhaseBlob blob;
      size_t read = prefs.getBytes(key, &blob, sizeof(blob));
      if (read == sizeof(blob)) {
        auto stage = (vpd_calc::GrowthStage)(i + 1); // 0->1(Seedling), 1->2(Veg), 2->3(Flow)
        ctrl->setSchedule(stage, blob.schedule);
        ctrl->setStageModeSettings(stage, plant_ctrl::DayMode::Day, 
          blob.modes[0].ledPercent, blob.modes[0].fanMin, blob.modes[0].fanMax, blob.modes[0].vpdMin, blob.modes[0].vpdMax);
        ctrl->setStageModeSettings(stage, plant_ctrl::DayMode::Night, 
          blob.modes[1].ledPercent, blob.modes[1].fanMin, blob.modes[1].fanMax, blob.modes[1].vpdMin, blob.modes[1].vpdMax);
        ctrl->setStageModeSettings(stage, plant_ctrl::DayMode::NightSilent, 
          blob.modes[2].ledPercent, blob.modes[2].fanMin, blob.modes[2].fanMax, blob.modes[2].vpdMin, blob.modes[2].vpdMax);
        any = true;
        Serial.printf("[WEB] loadPhases: loaded %s (st=%d), On=%02d:%02d, Off=%02d:%02d, Silent=%s, Rise=%d, Set=%d\n", 
          key, (int)stage, blob.schedule.on.hour, blob.schedule.on.minute, blob.schedule.off.hour, blob.schedule.off.minute,
          blob.schedule.use_night_silent ? "YES" : "NO", blob.schedule.sunrise_minutes, blob.schedule.sunset_minutes);
        
        const char* mnames[] = {"day", "night", "night_silent"};
        for (int m=0; m<3; ++m) {
          Serial.printf("[WEB] loadPhases:   Mode %s: LED=%.1f%%, Fan=%.1f-%.1f%%, VPD=%.2f-%.2f\n", 
            mnames[m], blob.modes[m].ledPercent, blob.modes[m].fanMin, blob.modes[m].fanMax, blob.modes[m].vpdMin, blob.modes[m].vpdMax);
        }
      } else {
        Serial.printf("[WEB] loadPhases: size mismatch %s (exp %u, got %u)\n", key, (unsigned)sizeof(blob), (unsigned)read);
      }
    }
  }
  prefs.end();
  if (!any) Serial.println(F("[WEB] loadPhases: no saved settings found in NVS"));
  return any;
}

bool WebCtrl::savePhases(plant_ctrl::PlantCtrl* ctrl) {
  if (!ctrl) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_PHASES, false)) {
    Serial.println(F("[WEB] savePhases: Preferences.begin(PHASES, false) FAILED"));
    return false;
  }

  for (int i=0; i<3; ++i) {
    auto stage = (vpd_calc::GrowthStage)(i + 1);
    PhaseBlob blob;
    blob.schedule = ctrl->getSchedule(stage);
    blob.modes[0] = ctrl->getStageModeSettings(stage, plant_ctrl::DayMode::Day);
    blob.modes[1] = ctrl->getStageModeSettings(stage, plant_ctrl::DayMode::Night);
    blob.modes[2] = ctrl->getStageModeSettings(stage, plant_ctrl::DayMode::NightSilent);
    
    char key[8]; snprintf(key, sizeof(key), "p%d", i);
    size_t written = prefs.putBytes(key, &blob, sizeof(blob));
    Serial.printf("[WEB] savePhases: saved %s (st=%d), On=%02d:%02d, Off=%02d:%02d, size %u\n", 
      key, (int)stage, blob.schedule.on.hour, blob.schedule.on.minute, blob.schedule.off.hour, blob.schedule.off.minute, (unsigned)written);
  }
  prefs.end();
  return true;
}

static uint64_t uptime_s() { return millis() / 1000ULL; }

void WebCtrl::begin(plant_ctrl::PlantCtrl* ctrl, RTC_Ctrl* rtc, NetCtrl* net) {
  ctrl_ = ctrl; rtc_ = rtc; net_ = net;

  // RTC-Check beim Start
  if (rtc_) {
    rtcPresent_ = rtc_->isConnected(); // Einmalig prüfen
    if (!rtcPresent_) {
      Serial.println(F("[WEB] RTC nicht gefunden - deaktiviere RTC-Sync"));
    }
  }

  // Ensure all NVS namespaces exist by opening in RW mode once
  {
    const char* ns[] = {NVS_APP, NVS_GROW, NVS_DRY, NVS_NOTIFY, NVS_STEPPER, NVS_TOF, NVS_UPDATE, NVS_PHASES, NVS_MQTT};
    Preferences p; for (auto s : ns) { p.begin(s, false); p.end(); }
  }

  AppCfg tmpA; loadAppCfg(tmpA);
  GrowState tmpG; loadGrow(tmpG);
  grow_ = tmpG;
  DryingState tmpD; loadDrying(tmpD);
  drying_ = tmpD;
  NotifyCfg tmpN; loadNotify(tmpN);
  notify_ = tmpN;
  StepperCfg tmpS; loadStepperCfg(tmpS);
  stepper_cfg_ = tmpS;
  ToFCfg tmpT; loadTofCfg(tmpT);
  tof_cfg_ = tmpT;
  MqttConfig tmpM; loadMqttCfg(tmpM);
  mqtt_cfg_ = tmpM;
  
  if (ctrl_) {
    loadPhases(ctrl_);
    syncControllerStage_();
  }

  if (mqtt_) {
      mqtt_->begin(mqtt_cfg_, app_.name);
  }

  // Initialen Grow/Drying-Status an PlantCtrl weiterleiten
  if (ctrl_) {
    ctrl_->setGrowActive(grow_.started);
    ctrl_->setDryingMode(drying_.active);
  }

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

void WebCtrl::setHardware(GP8211Ctrl* dac, FanCtrl* fan, FanCtrl* fan2, FanCtrl* fan3, StepperCtrl* stepper, ToFCtrl* tof, SHT41Ctrl* shtIn, SHT41Ctrl* shtOut) {
  dac_ = dac; fan_ = fan; fan2_ = fan2; fan3_ = fan3; step_ = stepper; tof_ = tof; shtIn_ = shtIn; shtOut_ = shtOut;
  
  if (dac_) {
    currentDacPct_ = targetDacPct_ = dac_->getPercent();
  }

  if (step_) {
      // Apply loaded config
      step_->setMaxTravelMm(stepper_cfg_.max_travel_mm);
  }

  if (tof_) {
      // Apply loaded config (NVS)
      tof_->setOffsetMm(tof_cfg_.offset_mm);
  }
}

void WebCtrl::loop() {
  uint32_t now = millis();
  
  checkStepperCalibration_();

  // Manual LED Fading logic (only if grow not started)
  if (dac_ && !grow_.started) {
    if (now - lastDacFadeMs_ >= 25) { // 40 Hz update
      float diff = targetDacPct_ - currentDacPct_;
      if (fabsf(diff) > 0.1f) {
        float step = 2.5f; // 2.5% per 25ms = 100% per 1.0s
        if (diff > 0) currentDacPct_ += fminf(diff, step);
        else currentDacPct_ -= fminf(-diff, step);
        dac_->setPercent(currentDacPct_);
      } else if (currentDacPct_ != targetDacPct_) {
        currentDacPct_ = targetDacPct_;
        dac_->setPercent(currentDacPct_);
      }
      lastDacFadeMs_ = now;
    }
  }

  if (mqtt_) {
      mqtt_->loop();
      static uint32_t lastMqttPush = 0;
      if (now - lastMqttPush >= 2000) { // 2s update rate
          lastMqttPush = now;
          if (mqtt_->isConnected()) {
              mqtt_->publishState(makeStatusJson_());
          }
      }
  }

  if (now >= nextProbeAt_) {
    if (net_) net_->probeInternet();
    nextProbeAt_ = now + 60000;
  }

  // System-Zeit aus RTC setzen wenn NTP nicht verfÃ¼gbar
  static bool systemTimeSetFromRTC = false;
  if (!systemTimeSetFromRTC && rtc_ && rtcPresent_) { // Check rtcPresent_
    time_t tnow = time(nullptr);
    // Wenn Systemzeit noch nicht gesetzt (vor 2023), aus RTC initialisieren
    if (tnow < 1700000000) {
      if (syncSystemFromRTC_()) {
        systemTimeSetFromRTC = true;
        Serial.println(F("[WEB] Systemzeit aus RTC initialisiert"));
        syncControllerStage_();
      }
    }
  }

  // Periodische RTC-Synchronisierung: einmal nach erstem Internet (SNTP) und danach tÃ¤glich gegen 03:05 Lokalzeit
  if (net_ && net_->internetOK() && rtc_ && rtcPresent_) { // Check rtcPresent_
    time_t tnow = time(nullptr);
    // Nur synchronisieren, wenn Systemzeit valide (SNTP) ist
    if (tnow >= 1700000000) {
      struct tm tmL; localtime_r(&tnow, &tmL);
      uint32_t ymd = (uint32_t)((tmL.tm_year+1900)*10000 + (tmL.tm_mon+1)*100 + tmL.tm_mday);
      // Erste Synchronisierung nach Boot sobald Internetzeit vorhanden
      if (!rtcSyncedOnce_) {
        if (syncRTCFromSystem_()) { rtcSyncedOnce_ = true; lastRtcSyncYMD_ = ymd; }
      } else {
        // TÃ¤gliche Synchronisierung: zwischen 03:05:00 und 03:10:59, maximal einmal pro Tag
        if (ymd != lastRtcSyncYMD_ && tmL.tm_hour == 3 && tmL.tm_min >= 5 && tmL.tm_min <= 10) {
          if (syncRTCFromSystem_()) { lastRtcSyncYMD_ = ymd; }
        }
      }
    }
  }
  if (now >= nextPushAt_) {
    if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
    nextPushAt_ = now + 2000;
  }

  // Sync stage from epoch periodically
  static uint32_t lastStageSync = 0;
  if (now - lastStageSync >= 60000) {
    lastStageSync = now;
    syncControllerStage_();
  }

  if (rebootAt_ && now >= rebootAt_) {
    delay(100);
    ESP.restart();
  }
}

void WebCtrl::setupRoutes_() {
  http_.on("/", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!setup_flag::is_done()) { req->redirect("/setup"); return; }
    sendFile_(req, "/index.html", "text/html; charset=utf-8");
  });
  http_.on("/index.html", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!setup_flag::is_done()) { req->redirect("/setup"); return; }
    sendFile_(req, "/index.html", "text/html; charset=utf-8");
  });
  http_.on("/pico.min.css", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/pico.min.css", "text/css"); });
  http_.on("/custom.css", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/custom.css", "text/css"); });
  http_.on("/bg.jpg", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/bg.jpg", "image/jpeg"); });
  http_.on("/CHANGELOG.md", HTTP_GET, [this](AsyncWebServerRequest* req){ sendFile_(req, "/CHANGELOG.md", "text/plain; charset=utf-8"); });

  // Setup-Seite und APIs
  http_.on("/setup", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (setup_flag::is_done()) { req->redirect("/"); return; }
    sendFile_(req, "/setup.html", "text/html; charset=utf-8");
  });
  http_.on("/api/setup/status", HTTP_GET, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", makeSetupStatusJson_()); });
  http_.on("/api/rtc/set", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; if (rtc_ && !doc["time"].isNull()) ok = rtc_->writeTimeFromString(String((const char*)doc["time"]));
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  // Body-only JSON endpoint; onRequest is empty, logic in body handler
  http_.on("/api/tof/calibrate", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t*, size_t, size_t, size_t){
             JsonDocument resp; bool ok=false; int measured=-1; int target=lumina::calib::TOF_CAL_TARGET_MM; int off=0;
             if (tof_) {
               // Vor Messung Offset neutralisieren, um RAW zu lesen
               int16_t prevOff = tof_->getOffsetMm();
               tof_->setOffsetMm(0);
               for (int attempt=0; attempt<3 && measured<0; ++attempt) {
                 measured = tof_->readAvgMm(lumina::calib::TOF_CAL_SAMPLES);
                 if (measured < 0) { delay(10); tof_->reinit(); delay(10); }
               }
               if (measured >= 0) {
                 off = measured - target; 
                 tof_->setOffsetMm((int16_t)off);
                 tof_cfg_.offset_mm = (int16_t)off;
                 ok = saveTofCfg(tof_cfg_);
                 if (ok) Serial.printf("[WEB] ToF Offset kalibriert und in NVS gespeichert: %d mm\n", off);
               } else {
                 // Restore vorherigen Offset, wenn Messung fehlgeschlagen ist
                 tof_->setOffsetMm(prevOff);
               }
             }
             resp["ok"]=ok; resp["measured"]=measured; resp["target"]=target; resp["offset"]=off; String out; serializeJson(resp, out);
             req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/dac", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; bool on = doc["on"].as<bool>();
             if (dac_) {
               if (grow_.started) {
                 dac_->setPercent(on ? 100.0f : 0.0f);
               } else {
                 targetDacPct_ = on ? 100.0f : 0.0f;
                 // Initialize current if it's the first time
                 if (lastDacFadeMs_ == 0) currentDacPct_ = dac_->getPercent();
               }
               ok = true;
             }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/fan", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; bool on = doc["on"].as<bool>(); if (fan_) { fan_->setPercent(on?100.0f:0.0f); ok=true; }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/fan2", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; bool on = doc["on"].as<bool>(); if (fan2_) { fan2_->setPercent(on?100.0f:0.0f); ok=true; }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/fan3", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; bool on = doc["on"].as<bool>(); if (fan3_) { fan3_->setPercent(on?100.0f:0.0f); ok=true; }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/pump", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=true; bool on = doc["on"].as<bool>();
             int p = lumina::pins::PUMP_EN;
             if (p >= 0) { digitalWrite(p, on ? HIGH : LOW); }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(200, "application/json", out);
           });
  http_.on("/api/stepper/jog", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool ok=false; String dir = String((const char*)doc["dir"]); float mm = doc["mm"].isNull()?10.0f:doc["mm"].as<float>();
             if (step_) { if (dir=="up") { step_->moveUp(mm, lumina::plant::SPEED_LEVEL); ok=true; } else if (dir=="down") { step_->moveDown(mm, lumina::plant::SPEED_LEVEL); ok=true; } }
             JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
           });
  http_.on("/api/stepper/home", HTTP_POST, [this](AsyncWebServerRequest* req){
    bool ok=false; if (step_) { step_->resetHoming(); ok = step_->startHoming(); }
    JsonDocument resp; resp["ok"]=ok; String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
  });
  http_.on("/api/stepper/status", HTTP_GET, [this](AsyncWebServerRequest* req){
    JsonDocument resp; bool ok=false; if (step_) { auto st=step_->status(); ok=true; resp["ok"]=true; resp["pos_mm"]=st.position_mm; resp["moving"]=st.isMoving; resp["homing"]=st.isHoming; resp["lastDone"]=st.lastOpDone; resp["uart_ok"]=st.uart_ok; resp["max_travel_mm"] = stepper_cfg_.max_travel_mm; }
    String out; serializeJson(resp, out); req->send(ok?200:500, "application/json", out);
  });

  // Phases API
  http_.on("/api/settings/phases", HTTP_GET, [this](AsyncWebServerRequest* req){
    if (!ctrl_) { req->send(500, "application/json", "{\"error\":\"no ctrl\"}"); return; }
    JsonDocument doc;
    JsonArray arr = doc["phases"].to<JsonArray>();
    const char* names[] = {"Seedling", "Vegetative", "Flowering"};
    for (int i=0; i<3; ++i) {
      JsonObject p = arr.add<JsonObject>();
      p["id"] = i;
      p["name"] = names[i];
      auto sch = ctrl_->getSchedule((vpd_calc::GrowthStage)(i + 1));
      JsonObject s = p["schedule"].to<JsonObject>();
      char buf[8];
      snprintf(buf, sizeof(buf), "%02d:%02d", sch.on.hour, sch.on.minute); s["on"] = buf;
      snprintf(buf, sizeof(buf), "%02d:%02d", sch.off.hour, sch.off.minute); s["off"] = buf;
      s["sunrise_min"] = sch.sunrise_minutes;
      s["sunset_min"] = sch.sunset_minutes;
      s["use_silent"] = sch.use_night_silent;
      
      JsonObject modes = p["settings"].to<JsonObject>();
      const char* mnames[] = {"day", "night", "night_silent"};
      for (int m=0; m<3; ++m) {
        auto sett = ctrl_->getStageModeSettings((vpd_calc::GrowthStage)(i + 1), (plant_ctrl::DayMode)m);
        JsonObject mo = modes[mnames[m]].to<JsonObject>();
        mo["led"] = sett.ledPercent;
        mo["fan_min"] = sett.fanMin;
        mo["fan_max"] = sett.fanMax;
        mo["vpd_min"] = sett.vpdMin;
        mo["vpd_max"] = sett.vpdMax;
      }
    }
    String out; serializeJson(doc, out); req->send(200, "application/json", out);
  });
  
  http_.on("/api/settings/phases/reset", HTTP_POST, [this](AsyncWebServerRequest* req){
    if (!ctrl_) { req->send(500, "application/json", "{\"error\":\"no ctrl\"}"); return; }
    Serial.println(F("[WEB] POST /api/settings/phases/reset: restoring factory defaults..."));
    
    // Reset Schedules
    ctrl_->setSchedule(vpd_calc::GrowthStage::Seedling, lumina::schedule::SEEDLING);
    ctrl_->setSchedule(vpd_calc::GrowthStage::Vegetative, lumina::schedule::VEGETATIVE);
    ctrl_->setSchedule(vpd_calc::GrowthStage::Flowering, lumina::schedule::FLOWERING);
    
    // Reset Mode Settings
    const vpd_calc::GrowthStage stages[] = {
      vpd_calc::GrowthStage::Seedling, 
      vpd_calc::GrowthStage::Vegetative, 
      vpd_calc::GrowthStage::Flowering
    };
    for (int i=0; i<3; ++i) {
      for (int m=0; m<3; ++m) {
        auto& d = lumina::defaults::PHASE_MODE[i][m];
        ctrl_->setStageModeSettings(stages[i], (plant_ctrl::DayMode)m, 
                                    d.ledPercent, d.fanMin, d.fanMax, d.vpdMin, d.vpdMax);
      }
    }
    savePhases(ctrl_);
    req->send(200, "application/json", "{\"ok\":true}");
  });
  
  http_.on("/api/settings/phases", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             if (!ctrl_) { req->send(500, "application/json", "{\"error\":\"no ctrl\"}"); return; }
             JsonDocument doc; DeserializationError err = deserializeJson(doc, data, len);
             if (err) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             
             JsonArray phases = doc["phases"];
             if (phases.isNull()) { req->send(400, "application/json", "{\"error\":\"missing phases array\"}"); return; }
             
             Serial.println(F("[WEB] POST /api/settings/phases: processing..."));
             for (JsonObject p : phases) {
               int id = p["id"] | -1;
               if (id < 0 || id > 2) continue;
               auto stage = (vpd_calc::GrowthStage)(id + 1); // Fix: 0->1, 1->2, 2->3
               
               // Schedule
               if (!p["schedule"].isNull()) {
                 JsonObject s = p["schedule"];
                 auto sch = ctrl_->getSchedule(stage);
                 if (s["on"].is<const char*>()) { 
                   int h, m; if (sscanf(s["on"].as<const char*>(), "%d:%d", &h, &m)==2) { sch.on.hour=h; sch.on.minute=m; }
                 }
                 if (s["off"].is<const char*>()) {
                   int h, m; if (sscanf(s["off"].as<const char*>(), "%d:%d", &h, &m)==2) { sch.off.hour=h; sch.off.minute=m; }
                 }
                 if (!s["sunrise_min"].isNull()) sch.sunrise_minutes = s["sunrise_min"];
                 if (!s["sunset_min"].isNull()) sch.sunset_minutes = s["sunset_min"];
                 if (!s["use_silent"].isNull()) sch.use_night_silent = s["use_silent"];
                 ctrl_->setSchedule(stage, sch);
                 Serial.printf("[WEB] Update Phase %d Schedule: On=%02d:%02d, Off=%02d:%02d, Silent=%s, Rise=%d, Set=%d\n", 
                               id, sch.on.hour, sch.on.minute, sch.off.hour, sch.off.minute, 
                               sch.use_night_silent ? "YES" : "NO", sch.sunrise_minutes, sch.sunset_minutes);
               }
               
               // Settings
               if (!p["settings"].isNull()) {
                 JsonObject modes = p["settings"];
                 const char* mnames[] = {"day", "night", "night_silent"};
                 for (int m=0; m<3; ++m) {
                   if (!modes[mnames[m]].isNull()) {
                     JsonObject mo = modes[mnames[m]];
                     auto sett = ctrl_->getStageModeSettings(stage, (plant_ctrl::DayMode)m);
                     bool changed = false;
                     if (!mo["led"].isNull()) { sett.ledPercent = mo["led"]; changed = true; }
                     if (!mo["fan_min"].isNull()) { sett.fanMin = mo["fan_min"]; changed = true; }
                     if (!mo["fan_max"].isNull()) { sett.fanMax = mo["fan_max"]; changed = true; }
                     if (!mo["vpd_min"].isNull()) { sett.vpdMin = mo["vpd_min"]; changed = true; }
                     if (!mo["vpd_max"].isNull()) { sett.vpdMax = mo["vpd_max"]; changed = true; }
                     
                     if (changed) {
                       ctrl_->setStageModeSettings(stage, (plant_ctrl::DayMode)m, sett.ledPercent, sett.fanMin, sett.fanMax, sett.vpdMin, sett.vpdMax);
                       Serial.printf("[WEB] Update Phase %d Mode %s: LED=%d%%, Fan=%d-%d%%, VPD=%.2f-%.2f\n", 
                                     id, mnames[m], (int)sett.ledPercent, (int)sett.fanMin, (int)sett.fanMax, sett.vpdMin, sett.vpdMax);
                     }
                   }
                 }
               }
             }
             savePhases(ctrl_);
             req->send(200, "application/json", "{\"ok\":true}");
           });

  // WLAN Scan (asynchron gegen Watchdog-Crash)
  http_.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest* req){
    int n = WiFi.scanComplete();
    if (n == -2) {
      // Scan nicht gestartet oder fehlgeschlagen -> neu anstoÃŸen
      WiFi.scanNetworks(true);
      req->send(202, "application/json", "{\"status\":\"started\"}");
    } else if (n == -1) {
      // Scan lÃ¤uft noch
      req->send(202, "application/json", "{\"status\":\"running\"}");
    } else {
      // Ergebnis liegt vor
      JsonDocument doc;
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < n; ++i) {
        JsonObject o = arr.add<JsonObject>();
        o["ssid"] = WiFi.SSID(i);
        o["rssi"] = WiFi.RSSI(i);
        o["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
      }
      WiFi.scanDelete();
      String out; serializeJson(doc, out);
      req->send(200, "application/json", out);
    }
  });

  http_.on("/api/setup/done", HTTP_POST, [this](AsyncWebServerRequest* req){
    bool ok=setup_flag::set_done(true);
    bool doReboot = true;
    if (req->hasParam("reboot")) {
      String rb = req->getParam("reboot")->value();
      if (rb == "false" || rb == "0") doReboot = false;
    }
    if (ok && doReboot) rebootAt_ = millis()+800;
    req->send(ok?202:500, "application/json", ok?"{\"ok\":true}":"{\"ok\":false}");
  });
  http_.on("/api/setup/reset", HTTP_POST, [this](AsyncWebServerRequest* req){
    // Setup-Flag zurÃ¼cksetzen und Neustart auslÃ¶sen
    bool ok = setup_flag::set_done(false);
    if (ok) {
      rebootAt_ = millis() + 800;
      req->send(202, "application/json", "{\"ok\":true}");
    } else {
      req->send(500, "application/json", "{\"ok\":false}");
    }
  });
  http_.on("/api/setup/abort", HTTP_POST, [this](AsyncWebServerRequest* req){ req->send(200, "application/json", "{\"ok\":true}"); });
  // Status der digitalen EingÃ¤nge (TÃ¼r & Wasser)
  http_.on("/api/door/status", HTTP_GET, [this](AsyncWebServerRequest* req){
    JsonDocument resp;
    
    // TÃ¼r (CON2)
    int dPin = lumina::plant::DOOR_SWITCH_PIN;
    bool dClosed = false;
    if (dPin >= 0) {
      pinMode(dPin, INPUT); // Externer Pullup
      dClosed = (digitalRead(dPin) == LOW);
    }
    resp["door_closed"] = dClosed;
    resp["door_present"] = (dPin >= 0);

    // Wasser (CON1)
    int wPin = lumina::pins::WATER_LEVEL_STATE;
    bool wLow = false; // "Geschlossen" bedeutet Pegel OK oder AusgelÃ¶st? User sagt "wenn geschlossen auf false/0".
    // Laut GEMINI.md: "TRUE wenn Wasserstand zu niedrig" (Schwimmer oben/unten?)
    // Wir geben einfach den Zustand "Geschlossen/Offen" weiter.
    if (wPin >= 0) {
      pinMode(wPin, INPUT); // Externer Pullup
      wLow = (digitalRead(wPin) == LOW);
    }
    resp["water_closed"] = wLow;
    resp["water_present"] = (wPin >= 0);

    String out; serializeJson(resp, out);
    req->send(200, "application/json", out);
  });

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
  
  http_.on("/api/mqtt", HTTP_GET, [this](AsyncWebServerRequest* req){
      JsonDocument doc;
      doc["enabled"] = mqtt_cfg_.enabled;
      doc["server"] = mqtt_cfg_.server;
      doc["port"] = mqtt_cfg_.port;
      doc["user"] = mqtt_cfg_.user;
      doc["pass"] = mqtt_cfg_.pass;
      String out; serializeJson(doc, out); req->send(200, "application/json", out);
  });
  http_.on("/api/mqtt", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
      [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
          JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
          MqttConfig c = mqtt_cfg_;
          if (!doc["enabled"].isNull()) c.enabled = doc["enabled"].as<bool>();
          if (!doc["server"].isNull()) c.server = String((const char*)doc["server"]);
          if (!doc["port"].isNull()) c.port = (uint16_t)doc["port"].as<uint16_t>();
          if (!doc["user"].isNull()) c.user = String((const char*)doc["user"]);
          if (!doc["pass"].isNull()) c.pass = String((const char*)doc["pass"]);
          
          saveMqttCfg(c);
          req->send(200, "application/json", "{\"ok\":true}");
  });

  // Register updater routes
  registerUpdateRoutes_();

  http_.on("/api/app", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             bool changed = false;
             if (!doc["seed"].isNull()) { app_.seed = String((const char*)doc["seed"]); changed = true; }
             if (!doc["name"].isNull()) { 
                 app_.name = String((const char*)doc["name"]); 
                 changed = true;
                 if (mqtt_) mqtt_->setDeviceName(app_.name);
             }
             if (changed) saveAppCfg(app_);
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
             bool requestedReboot = doc["reboot"] | false;
             bool connectNow = doc["connect_now"] | true;

             if (net_) net_->saveConfig(nc);
             bool connectedNow = false;
             if (anyNetFieldTouched && net_ && connectNow) {
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
               Serial.println("[WA] Notifications disabled (switch off) â€“ aborting send");
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
               // PrÃ¼fen ob Trocknung aktiv ist
               if (drying_.active) {
                 req->send(400, "application/json", "{\"error\":\"Drying mode active\"}");
                 return;
               }
               
               // Seed-Name speichern, falls vorhanden
               if (!doc["seed"].isNull()) {
                 app_.seed = String((const char*)doc["seed"]);
                 saveAppCfg(app_);
                 if (mqtt_) mqtt_->publishState(makeStatusJson_());
               }

               grow_.started = true; 
               if (!doc["start_epoch"].isNull()) grow_.start_epoch = doc["start_epoch"].as<uint32_t>();
               else grow_.start_epoch = (uint32_t)now;
               
               if (!doc["total_days"].isNull()) grow_.total_days = (uint16_t)doc["total_days"].as<uint16_t>();
               saveGrow(grow_);
               if (ctrl_) {
                 ctrl_->setGrowActive(true);
                 syncControllerStage_();
               }
               req->send(200, "application/json", "{\"ok\":true}");
               if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
             } else if (action == "stop") {
               grow_.started = false; grow_.start_epoch = 0; saveGrow(grow_);
               if (ctrl_) ctrl_->setGrowActive(false);
               req->send(200, "application/json", "{\"ok\":true}");
               if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
             } else {
               req->send(400, "application/json", "{\"error\":\"unknown action\"}");
             }
           });

  http_.on("/api/drying", HTTP_POST, [](AsyncWebServerRequest*){}, NULL,
           [this](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t){
             JsonDocument doc; if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"error\":\"bad json\"}"); return; }
             String action = String((const char*)doc["action"]);
             time_t now = time(nullptr);
             if (action == "start") {
               // PrÃ¼fen ob Grow aktiv ist
               if (grow_.started) {
                 req->send(400, "application/json", "{\"error\":\"Grow mode active\"}");
                 return;
               }
               drying_.active = true; drying_.start_epoch = (uint32_t)now;
               saveDrying(drying_);
               // PlantCtrl Ã¼ber Trocknungsmodus informieren
               if (ctrl_) ctrl_->setDryingMode(true);
               req->send(200, "application/json", "{\"ok\":true}");
               if (ws_.count() > 0) ws_.textAll(makeStatusJson_());
             } else if (action == "stop") {
               drying_.active = false; drying_.start_epoch = 0; saveDrying(drying_);
               // PlantCtrl Trocknungsmodus beenden
               if (ctrl_) ctrl_->setDryingMode(false);
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

String WebCtrl::makeSetupStatusJson_() {
  JsonDocument doc;
  doc["setup_done"] = setup_flag::is_done();
  doc["rtc_ok"] = rtc_ ? rtc_->isConnected() : false;
  const auto& hs = health::state();
  JsonObject mods = doc["modules"].to<JsonObject>();
  mods["i2c0"] = hs.mod.i2c0_ok;
  mods["i2c1"] = hs.mod.i2c1_ok;
  mods["sht_in"] = hs.mod.sht_in_ok;
  mods["sht_out"] = hs.mod.sht_out_ok;
  mods["dac"] = hs.mod.dac_ok;
  mods["fan"] = hs.mod.fan_ok;
  mods["rtc"] = hs.mod.rtc_ok;
  mods["tof"] = hs.mod.tof_ok;
  mods["fs"] = hs.mod.fs_ok;
  if (tof_) {
    int mm = tof_->readAvgMm(5);
    doc["tof_mm"] = mm;
    doc["tof_offset"] = (int)tof_->getOffsetMm();
  }
  if (fan_) {
    doc["fan_rpm"] = fan_->getRPM();
  }
  if (fan2_) {
    doc["fan2_rpm"] = fan2_->getRPM();
  }
  if (fan3_) {
    doc["fan3_rpm"] = fan3_->getRPM();
  }
  if (step_) {
    auto st = step_->status();
    JsonObject s = doc["stepper"].to<JsonObject>();
    s["pos_mm"] = st.position_mm;
    s["moving"] = st.isMoving;
    s["homing"] = st.isHoming;
    s["lastDone"] = st.lastOpDone;
    s["uart_ok"] = st.uart_ok;
    s["max_travel_mm"] = stepper_cfg_.max_travel_mm;
  }
  if (net_) {
    doc["wifi_ssid"] = net_->cfg().ssid;
  }
  doc["initial_setup_done"] = setup_flag::is_done();
  String out; serializeJson(doc, out); return out;
}

bool WebCtrl::syncRTCFromSystem_() {
  if (!rtc_) return false;
  time_t now = time(nullptr);
  if (now <= 0) return false;
  struct tm tmL; localtime_r(&now, &tmL);
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d %02d-%02d-%02d",
           tmL.tm_mday, tmL.tm_mon + 1, tmL.tm_year + 1900,
           tmL.tm_hour, tmL.tm_min, tmL.tm_sec);
  return rtc_->writeTimeFromString(String(buf));
}

bool WebCtrl::syncSystemFromRTC_() {
  if (!rtc_) return false;
  uint16_t y; uint8_t mo, d, hh, mi, ss;
  if (!rtc_->readComponents(y, mo, d, hh, mi, ss)) return false;

  // Konvertiere RTC-Zeit zu Unix-Timestamp
  struct tm tmRTC = {0};
  tmRTC.tm_year = y - 1900;
  tmRTC.tm_mon = mo - 1;
  tmRTC.tm_mday = d;
  tmRTC.tm_hour = hh;
  tmRTC.tm_min = mi;
  tmRTC.tm_sec = ss;
  tmRTC.tm_isdst = -1; // Auto-DST

  time_t rtcTime = mktime(&tmRTC);
  if (rtcTime == -1) return false;

  // Setze Systemzeit
  struct timeval tv = { .tv_sec = rtcTime, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  return true;
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
  doc["ssid"] = net_ ? net_->cfg().ssid : String("");
  doc["use_static"] = net_ ? net_->cfg().useStatic : false;
  doc["wifi_connected"] = net_ ? net_->isConnected() : false;
  doc["internet_ok"] = net_ ? net_->internetOK() : false;
  doc["rssi"] = net_ ? net_->rssi() : -127;
  doc["uptime_s"] = (uint64_t)uptime_s();
  // Lokale Zeit bevorzugt aus System (TZ via NetCtrl::tzInit). Fallback: RTC.
  {
    String tstr;
    time_t now = time(nullptr);
    // Nutze Systemzeit nur, wenn SNTP gesetzt ist (nach 2023-11-14 ca.).
    if (now >= 1700000000) {
      struct tm tmL;
      localtime_r(&now, &tmL);
      char buf[24];
      snprintf(buf, sizeof(buf), "%02d.%02d.%02d %02d:%02d:%02d",
               tmL.tm_mday, tmL.tm_mon + 1, (tmL.tm_year + 1900) % 100,
               tmL.tm_hour, tmL.tm_min, tmL.tm_sec);
      tstr = String(buf);
    } else {
      tstr = rtc_ ? rtc_->readTimeString() : String("");
    }
    doc["ntp_time"] = tstr;
  }
  // Env
  float tIn = NAN, rhIn = NAN; double vpd = NAN; float tOut = NAN; float rhOut = NAN;
  if (ctrl_) {
    tIn = (float)ctrl_->currentTemp();
    rhIn = (float)ctrl_->currentRh();
    vpd = ctrl_->currentVpd();
    tOut = (float)ctrl_->currentTempOut();
    rhOut = (float)ctrl_->currentRhOut();
  }
  // Fallback im Setup-Modus: direkt aus SHT41 lesen, wenn Werte fehlen
  if ((isnan(tIn) || isnan(rhIn)) && shtIn_) {
    float ti= NAN, rhi = NAN; if (shtIn_->read(ti, rhi)) { tIn = ti; rhIn = rhi; }
  }
  if ((isnan(tOut) || isnan(rhOut)) && shtOut_) {
    float to = NAN, rho = NAN; if (shtOut_->read(to, rho)) { if (isnan(tOut)) tOut = to; if (isnan(rhOut)) rhOut = rho; }
  }
  auto round1 = [](float x){ return isnan(x)? NAN : (float)roundf(x*10.f)/10.f; };
  doc["temp_c"] = round1(tIn);
  doc["humi_rh"] = round1(rhIn);
  doc["vpd_kpa"] = round1((float)vpd);
  doc["temp_out_c"] = round1(tOut);
  doc["humi_out_rh"] = round1(rhOut);

  // Taupunkt berechnen (Magnus-Formel)
  float dewC = NAN;
  if (!isnan(tIn) && !isnan(rhIn) && rhIn > 0.0f) {
    const float a = 17.27f, b = 237.7f;
    float gamma = (a * tIn) / (b + tIn) + logf(rhIn / 100.0f);
    dewC = (b * gamma) / (a - gamma);
  }
  doc["dew_c"] = round1(dewC);

  // Actuators
  float ledPct = 0.0f;
  if (grow_.started && ctrl_) {
    ledPct = ctrl_->currentLedPercentEffective();
  } else if (dac_) {
    ledPct = dac_->getPercent();
  }
  
  float fanPct = fan_ ? fan_->getPercent() : 0.0f;
  int fanRpm = fan_ ? fan_->getRPM() : 0;
  int fan2Rpm = fan2_ ? fan2_->getRPM() : 0;
  float fan2Pct = fan2_ ? fan2_->getPercent() : 0.0f;
  float fan3Pct = fan3_ ? fan3_->getPercent() : 0.0f;
  
  doc["light_on"] = ledPct > 0.5f;
  doc["light_pct"] = round1(ledPct);
  doc["fan_on"]   = fanPct > 0.5f;
  doc["fan_pct"]  = round1(fanPct);
  doc["fan_rpm"]  = fanRpm;
  doc["fan2_on"]  = fan2Pct > 0.5f;
  doc["fan2_pct"] = round1(fan2Pct);
  doc["fan2_rpm"] = fan2Rpm;
  doc["fan3_on"]  = fan3Pct > 0.5f;
  doc["fan3_pct"] = round1(fan3Pct);
  
  int pPin = lumina::pins::PUMP_EN;
  doc["pump_on"]  = (pPin >= 0) && (digitalRead(pPin) == HIGH);

  // TÃ¼r-Status
  bool doorOpen = ctrl_ ? ctrl_->isDoorOpen() : false;
  doc["door_open"] = doorOpen;

  if (step_) {
    auto st = step_->status();
    JsonObject s = doc["stepper"].to<JsonObject>();
    s["pos_mm"] = st.position_mm;
    s["moving"] = st.isMoving;
    s["homing"] = st.isHoming;
    s["lastDone"] = st.lastOpDone;
    s["uart_ok"] = st.uart_ok;
    s["max_travel_mm"] = stepper_cfg_.max_travel_mm;
  }

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

  doc["tof_offset"] = tof_cfg_.offset_mm;
  if (tof_) {
    doc["tof_mm"] = tof_->readAvgMm(1);
  }

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

  // Drying block
  JsonObject dry = doc["drying"].to<JsonObject>();
  dry["active"] = drying_.active;
  dry["start_epoch"] = drying_.active ? drying_.start_epoch : 0;
  uint16_t dryDay = 0;
  if (drying_.active && drying_.start_epoch) {
    time_t now = time(nullptr);
    if (now >= drying_.start_epoch) {
      uint32_t days = (uint32_t)((now - drying_.start_epoch)/86400UL) + 1;
      if (days > lumina::drying::DURATION_DAYS) days = lumina::drying::DURATION_DAYS;
      dryDay = (uint16_t)days;
    }
  }
  dry["day"] = dryDay;

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
  doc["device_name"] = app_.name;
  doc["fw"] = FW_VERSION;
  doc["build"] = String(__DATE__) + String(" ") + String(__TIME__);
  doc["ts"] = (uint64_t)millis();

  // Append last known update info (from background daily check)
  JsonObject upd = doc["update"].to<JsonObject>();
  upd["has_update"] = latestKnownHasUpdate_;
  upd["latest"] = latestKnownTag_;

  // MQTT status
  JsonObject mqttObj = doc["mqtt"].to<JsonObject>();
  mqttObj["enabled"] = mqtt_cfg_.enabled;
  mqttObj["connected"] = mqtt_ ? mqtt_->isConnected() : false;

  // Schedule background update check: once after first NTP sync (boot) and then daily ~03:00 if internet is OK
  if (!updateCheckJobRunning_ && net_ && net_->internetOK()) {
    time_t now = time(nullptr);
    if (now > 0) {
      struct tm tmL; localtime_r(&now, &tmL);
      uint32_t ymd = (uint32_t)((tmL.tm_year+1900)*10000 + (tmL.tm_mon+1)*100 + tmL.tm_mday);
      if (!firstUpdateCheckDone_) {
        firstUpdateCheckDone_ = true;
        updateCheckJobRunning_ = true;
        xTaskCreatePinnedToCore(&WebCtrl::updateCheckTaskTrampoline_, "upd_check", 6144, this, 1, nullptr, 0);
      } else if (tmL.tm_hour >= 3 && lastUpdateCheckYMD_ != ymd) {
        updateCheckJobRunning_ = true;
        xTaskCreatePinnedToCore(&WebCtrl::updateCheckTaskTrampoline_, "upd_check", 6144, this, 1, nullptr, 0);
      }
    }
  }

  String out; serializeJson(doc, out); return out;
}

String WebCtrl::makeInfoJson_() {
  JsonDocument doc;
  doc["seed"] = app_.seed;
  doc["device_name"] = app_.name;
  JsonObject netw = doc["network"].to<JsonObject>();
  if (net_) {
    netw["ssid"] = net_->cfg().ssid;
    netw["pass"] = net_->cfg().pass.length() > 0 ? "********" : "";
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
  
  doc["tof_offset"] = tof_cfg_.offset_mm;
  
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
    sendFile_(req, "/update.html", "text/html; charset=utf-8");
  });
  // Settings page
  http_.on("/settings", HTTP_GET, [this](AsyncWebServerRequest* req){
    sendFile_(req, "/settings.html", "text/html; charset=utf-8");
  });
  http_.on("/info", HTTP_GET, [this](AsyncWebServerRequest* req){
    sendFile_(req, "/info.html", "text/html; charset=utf-8");
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
    String ghUrl, latestTag, changelog;
    if (ghLatestTarUrl_(ghUrl, latestTag, changelog)) {
      resp["latest"] = latestTag;
      resp["tar_url"] = ghUrl;
      resp["changelog"] = changelog;
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
    // Pausiere die Regelung direkt beim Start des Updates
    health::state().control_paused = true;
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
      // Wenn wir direkt aus der Upload-Phase kommen, erlaube den Ãœbergang ins Anwenden
      if (updateJobRunning_ && updatePhase_ != "uploading") { req->send(409, "application/json", "{\"error\":\"update running\"}"); return; }
      // Upload ist abgeschlossen, beende Upload-Phase und starte Apply-Job
      updateJobRunning_ = false;
      startApplyUploadedJob_();
      req->send(202, "application/json", "{\"started\":true}");
    },
    [this](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final){
      if (index == 0) {
        // Begin upload: prepare temp file and set progress state
        handlePackageUploadBegin_();
        // Pausiere die Regelung bereits zum Upload-Start, um Ressourcen frei zu machen
        health::state().control_paused = true;
        updateJobRunning_ = true;
        updatePhase_ = "uploading";
        updateMsg_ = "upload tar";
        // Try to get total content length (may be -1)
        int32_t cl = (int32_t)req->contentLength();
        updateDLTotal_ = cl > 0 ? cl : -1;
        updateDLSoFar_ = 0;
      }
      if (len) {
        handlePackageUploadWrite_(data, len);
        updateDLSoFar_ += (int32_t)len;
        if (updateDLTotal_ > 0) {
          int pct = (int)((100LL * updateDLSoFar_) / updateDLTotal_);
          updateMsg_ = String("upload ") + String(updateDLSoFar_) + "/" + String(updateDLTotal_) + String(" (") + String(pct) + String("%)");
        } else {
          updateMsg_ = String("upload ") + String(updateDLSoFar_) + String(" bytes");
        }
      }
      if (final) {
        // Upload finished; the main handler will start the apply job and keep running=true
        updateMsg_ = String("upload complete ") + String(updateDLSoFar_) + String(" bytes");
      }
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
  // WÃ¤hrend des Updates Regelung pausieren
  health::state().control_paused = true;
  // Spawn on Core 0 with moderate stack
  xTaskCreatePinnedToCore(&WebCtrl::updateTaskTrampoline_, "upd_remote", 12288, this, 1, nullptr, 0);
}

void WebCtrl::startApplyUploadedJob_() {
  if (updateJobRunning_) return;
  updateJobRunning_ = true; updatePhase_ = "applying"; updateMsg_ = "apply uploaded"; updateOk_ = false; updateFwUpdated_ = false; updateFilesUpdated_ = 0; updateErr_ = "";
  health::state().control_paused = true;
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
    String tarUrl, latestTag, changelog;
    if (!ghLatestTarUrl_(tarUrl, latestTag, changelog)) {
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
  // Regelung wieder freigeben, wenn kein Reboot folgt
  if (!(ok && fwUpd)) { health::state().control_paused = false; }
  if (ok && fwUpd) { rebootAt_ = millis() + 1000; }
}

void WebCtrl::updateCheckTaskTrampoline_(void* arg) {
  WebCtrl* self = static_cast<WebCtrl*>(arg);
  self->updateCheckTaskRun_();
  vTaskDelete(nullptr);
}

void WebCtrl::updateCheckTaskRun_() {
  // Perform a lightweight "latest" check (GitHub preferred, manifest fallback), update cached fields
  String latestTag, url, changelog;
  bool ok = ghLatestTarUrl_(url, latestTag, changelog);
  if (!ok) {
    // Fallback: manifest
    String manUrl = manifestUrl_();
    if (manUrl.length()) {
      WiFiClientSecure client; client.setInsecure();
      HTTPClient http; if (http.begin(client, manUrl)) { int code = http.GET(); if (code==200) {
            JsonDocument man; if (!deserializeJson(man, http.getStream())) { latestTag = String((const char*)man["version"]); ok = latestTag.length()>0; }
          } http.end(); }
    }
  }
  if (ok) {
    latestKnownTag_ = latestTag;
    latestKnownHasUpdate_ = (latestTag.length() && latestTag != String(FW_VERSION));
  }
  // stamp day to avoid repeated checks until next day window
  time_t now = time(nullptr); if (now>0) { struct tm tmL; localtime_r(&now, &tmL); lastUpdateCheckYMD_ = (uint32_t)((tmL.tm_year+1900)*10000 + (tmL.tm_mon+1)*100 + tmL.tm_mday); }
  updateCheckJobRunning_ = false;
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
      continue; // nÃ¤chsten Hop versuchen
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
  // Nur Upload finalisieren; das Anwenden erfolgt asynchron in startApplyUploadedJob_()
  fwUpdated=false; filesUpdated=0; err="";
  if (!LittleFS.exists(pkgTempPath_.c_str())) { err="temp not found"; return false; }
  // Paket ist vollstÃ¤ndig hochgeladen und liegt unter pkgTempPath_; Anwendung startet asynchron
  return true;
}

bool WebCtrl::applyPackageFromFile_(const char* tarPath, bool& fwUpdated, int& filesUpdated, String& err) {
  fwUpdated=false; filesUpdated=0; err="";
  File f = LittleFS.open(tarPath, FILE_READ); if (!f) { err="open failed"; return false; }
  const size_t BLK=512; uint8_t hdr[BLK];
  // Reset Apply-Progress
  updateApplyTotal_ = 0;
  updateApplySoFar_ = 0;
  updateMsg_ = "apply tar";
  while (true) {
    size_t r = f.read(hdr, BLK); if (r == 0) break; if (r != BLK) { err="bad tar header"; f.close(); return false; }
    bool allZero=true; for (size_t i=0;i<BLK;i++){ if (hdr[i]!=0){ allZero=false; break; } } if (allZero) break;
    char nameC[101]; memcpy(nameC, hdr+0, 100); nameC[100]='\0'; String name=String(nameC);
    uint32_t size=0; if (!parseOctal_((const char*)(hdr+124), 12, size)) size=0; char typeflag=(char)hdr[156]; if (typeflag=='\0') typeflag='0';
    uint32_t toRead=size; uint32_t pad=(BLK-(size%BLK))%BLK;
    // ZÃ¤hle Gesamtmenge der anzuwendenden Daten hoch, wenn regulÃ¤re Datei
    if (typeflag=='0') {
      if (name=="firmware.bin" || name.startsWith("www/")) {
        if (updateApplyTotal_ >= 0) updateApplyTotal_ += (int32_t)size;
      }
    }
    if (typeflag=='0') {
      if (name=="firmware.bin") {
        if (size==0) { err="empty firmware"; f.close(); return false; }
        updateMsg_ = "apply firmware";
        Serial.printf("[UPDATE] Applying firmware (%u bytes)...\n", (unsigned)size);
        if (!Update.begin((size_t)size, U_FLASH)) { err=String("Update.begin ")+Update.errorString(); f.close(); return false; }
        const size_t CH=2048; uint8_t buf[CH]; uint32_t left=toRead;
        while (left>0) { 
          size_t n = left>CH?CH:left; 
          int nr=f.read(buf,n); 
          if (nr<=0){ Update.abort(); err="firmware read"; f.close(); return false; } 
          size_t nw=Update.write(buf,nr); 
          if (nw!=(size_t)nr){ Update.abort(); err=String("Update.write ")+Update.errorString(); f.close(); return false; } 
          left-=nr; 
          updateApplySoFar_ += nr; 
          if (left % 32768 == 0) yield(); 
        }
        if (!Update.end()) { // changed from true to false to ensure size match
          err=String("Update.end ")+Update.errorString(); 
          Serial.printf("[UPDATE] Firmware error: %s\n", err.c_str());
          f.close(); return false; 
        }
        Serial.println(F("[UPDATE] Firmware update SUCCESS"));
        fwUpdated=true; 
        if (pad) f.seek(pad, SeekCur);
      } else if (name.startsWith("www/") && isSafeAssetPath_(name.substring(4))) {
        String rel=name.substring(4); String dest=String("/")+rel; String tmp=String("/.u_tmp_")+rel; ensureDir_(dest); int lastSlash=tmp.lastIndexOf('/'); if (lastSlash>0) { LittleFS.mkdir(tmp.substring(0,lastSlash)); }
        File wf=LittleFS.open(tmp, FILE_WRITE); if (!wf) { err=String("open fail ")+tmp; f.close(); return false; }
        const size_t CH=2048; uint8_t buf[CH]; uint32_t left=toRead;
        updateMsg_ = String("apply ")+rel;
        while (left>0) { size_t n=left>CH?CH:left; int nr=f.read(buf,n); if (nr<=0){ wf.close(); LittleFS.remove(tmp); err="asset read"; f.close(); return false; } if (wf.write(buf,nr)!=(size_t)nr){ wf.close(); LittleFS.remove(tmp); err="asset write"; f.close(); return false; } left-=nr; updateApplySoFar_ += nr; yield(); }
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
  Preferences prefs;
  if (!prefs.begin(NVS_UPDATE, true)) return false;
  manifestUrl = prefs.isKey("man_url") ? prefs.getString("man_url") : "";
  ghOwner = prefs.isKey("gh_owner") ? prefs.getString("gh_owner") : "";
  ghRepo = prefs.isKey("gh_repo") ? prefs.getString("gh_repo") : "";
  ghAsset = prefs.isKey("gh_asset") ? prefs.getString("gh_asset") : "";
  prefs.end();
  return (manifestUrl.length() + ghOwner.length() + ghRepo.length() + ghAsset.length()) > 0;
}

bool WebCtrl::saveUpdateCfg_(const String& manifestUrl, const String& ghOwner, const String& ghRepo, const String& ghAsset) {
  Preferences prefs;
  if (!prefs.begin(NVS_UPDATE, false)) return false;
  prefs.putString("man_url", manifestUrl);
  prefs.putString("gh_owner", ghOwner);
  prefs.putString("gh_repo", ghRepo);
  prefs.putString("gh_asset", ghAsset);
  prefs.end();
  return true;
}

String WebCtrl::manifestUrl_() {
  String u, o, r, a;
  if (strlen(UPDATE_MANIFEST_URL) > 0) return String(UPDATE_MANIFEST_URL);
  if (loadUpdateCfg_(u, o, r, a)) return u; return String("");
}

bool WebCtrl::ghLatestTarUrl_(String& outUrl, String& outLatestTag, String& outChangelog) {
  outUrl = ""; outLatestTag = ""; outChangelog = "";
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
      outChangelog = String((const char*)doc["body"]);
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

void WebCtrl::checkStepperCalibration_() {
    if (step_ && step_->checkNewCalibration()) {
        stepper_cfg_.max_travel_mm = step_->getMaxTravelMm();
        saveStepperCfg(stepper_cfg_);
        Serial.printf("[WEB] Saved new stepper calibration: %.2f mm\n", stepper_cfg_.max_travel_mm);
    }
}

