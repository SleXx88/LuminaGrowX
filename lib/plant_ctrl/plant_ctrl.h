// ==============================
// File: lib/plant_ctrl/plant_ctrl.h
// ==============================
/*
 * plant_ctrl.h | Pflanzensteuerung (Umgebung + Abstand)
 *
 * Ziele
 * - Beibehalten der bisherigen VPD-Steuerung (LED/Lüfter) aus env_ctrl
 * - Hinzufügen der LED-zu-Pflanze-Abstandsregelung mit ToF + Steppermotor
 * - Tür-/Zeitplan-Logik für sichere Absenkungen
 */

#pragma once

#include <Arduino.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "vpd_calc.h"
#include "stepper_ctrl.h"
#include "tof_ctrl.h"
#include "rtc_ctrl.h"

namespace plant_ctrl {

// Tagesmodi
enum class DayMode : uint8_t { Day=0, Night=1, NightSilent=2 };

struct TimeHM {
  uint8_t hour;
  uint8_t minute;
};

struct LightSchedule {
  TimeHM on;                // Start (Licht an)
  TimeHM off;               // Ende  (Licht aus)
  uint16_t sunrise_minutes; // Ramp-up Dauer
  uint16_t sunset_minutes;  // Ramp-down Dauer
  bool use_night_silent;    // Nutze NightSilent-Modus statt Night
};

// Konfiguration pro Phase & Modus
struct PhaseModeSettings {
  float ledPercent;   // fester LED-Prozentwert (0..100)
  float fanMin;       // Lüfter Mindestprozentsatz
  float fanMax;       // Lüfter Höchstprozentsatz
  float vpdMin;       // VPD-Zielbereich untere Grenze (kPa)
  float vpdMax;       // VPD-Zielbereich obere Grenze (kPa)
};

class PlantCtrl {
public:
  PlantCtrl();

  // Hardware binden (Objekte müssen extern initialisiert werden)
  void begin(SHT41Ctrl& sensorIndoor,
             SHT41Ctrl& sensorOutdoor,
             GP8211Ctrl& led,
             FanCtrl& fan,
             StepperCtrl& stepper,
             ToFCtrl& tof,
             RTC_Ctrl* rtc = nullptr,
             int doorPin = -1);

  // Phase/Modus
  void setStage(vpd_calc::GrowthStage stage);
  vpd_calc::GrowthStage getStage() const { return stage_; }
  void setMode(DayMode mode);

  // Überschreiben pro Phase/Modus
  void setStageModeSettings(vpd_calc::GrowthStage st, DayMode md,
                            float ledPercent, float fanMin, float fanMax,
                            float vpdMin, float vpdMax);
  PhaseModeSettings getStageModeSettings(vpd_calc::GrowthStage st, DayMode md) const;

  // Zeitpläne
  void setSchedule(vpd_calc::GrowthStage st, const LightSchedule& sch);
  LightSchedule getSchedule(vpd_calc::GrowthStage st) const;

  // Regelungsparameter
  void setKpFan(float kp);
  void setKiFan(float ki) { kiFan_ = fabsf(ki); } // (%/s)/kPa
  void setDeadband(float vpd_kPa);
  void setRateLimit(float pct_per_s);
  void setSmoothingAlpha(float alpha);

  // Temperaturschutz
  void setMaxTemperature(float tempC);
  void setOverTempReduction(float pct_points);
  void setTempHighFanBoost(float tempC, float boostPercent);

  // Feuchte/Taupunkt-Überschreibung
  void setHumidityOverride(float rh_high_percent, float dewpoint_gap_minC,
                           bool allow_silent_override, float extraBoostPercent);

  // Tür-/Transientenerkennung
  void setDoorDetection(float deltaRh_percent, float deltaT_C, uint32_t hold_ms);

  // Haltezeit nach Moduswechsel
  void setModeChangeHold(uint32_t hold_ms);

  // Außen feuchter -> Lüfter sperren
  void setOutsideHumidBlock(bool enable, float hysteresisC = 0.5f) {
    blockOutsideHumid_ = enable; dpHumidHyst_ = hysteresisC;
  }

  // Zentrale Konfiguration anwenden
  void applyLuminaConfig();

  // Trocknungsmodus aktivieren/deaktivieren
  void setDryingMode(bool active);
  bool isDryingMode() const { return dryingMode_; }
  bool isDoorOpen() const { return !lastDoorClosed_; }

  // Grow-Modus aktivieren/deaktivieren
  void setGrowActive(bool active);
  bool isGrowActive() const { return growActive_; }

  // Periodisches Update; gibt false zurück, wenn Sensorabfrage fehlgeschlagen
  bool update();

  // Start: blockierendes Heranfahren auf Mindestabstand (nach Homing)
  void runStartupApproachBlocking();

  // Zustand
  double currentVpd()  const { return vpdIn_; }
  double currentTemp() const { return tIn_; }
  double currentRh()   const { return rhIn_; }
  double currentVpdFiltered() const { return vpdFilt_; }

  float  currentLedPercent() const { return ledOut_; }
  // Effektive Anzeige (auf Basis des zuletzt angewendeten LED-Werts)
  float  currentLedPercentEffective() const {
    float p = !isnan(ledApplied_) ? ledApplied_ : ledOut_;
    if (!led_) return (p <= 0.0f ? 0.0f : (p < 1.0f ? 1.0f : (p > 100.0f ? 100.0f : p)));
    return led_->effectivePercent(p);
  }
  float  currentFanPercent() const { return fanOut_; }

  double currentTempOut() const { return tOut_; }
  double currentRhOut()   const { return rhOut_; }

  double dewPointIn()  const { return dpIn_; }
  double dewPointOut() const { return dpOut_; }
  int8_t fanVpdSign()  const { return fanSign_; }

  bool   isHoldActive() const { return millis() < holdUntilMs_; }

private:
  static int idxStage(vpd_calc::GrowthStage s) {
    switch (s) {
      case vpd_calc::GrowthStage::Seedling:   return 0;
      case vpd_calc::GrowthStage::Vegetative: return 1;
      case vpd_calc::GrowthStage::Flowering:  return 2;
      default: return 0;
    }
  }
  static int idxMode(DayMode m) {
    switch (m) {
      case DayMode::Day:         return 0;
      case DayMode::Night:       return 1;
      case DayMode::NightSilent: return 2;
      default: return 0;
    }
  }

  PhaseModeSettings& cur() { return settings_[idxStage(stage_)][idxMode(mode_)]; }

  static float clamp01(float p) { if (p<0) return 0; if (p>100) return 100; return p; }
  static float clamp(float x, float a, float b) { return x<a? a : (x>b? b : x); }

  float limitRate(float tgt, float last, float dt_s) const;
  void detectDoorOrTransient(float t_now, float rh_now, uint32_t now_ms);
  void updateFanSignFromDewpoints();

  // Abstand/ToF/Stepper
  void distanceTick_(uint32_t now);
  bool isDoorClosed_();
  bool allowDownAdjustNow_(uint32_t now);
  float targetDistanceMm_() const;

private:
  // Phase/Modus
  vpd_calc::GrowthStage stage_ = vpd_calc::GrowthStage::Seedling;
  DayMode mode_ = DayMode::Day;

  // Standardtabelle [3][3]
  PhaseModeSettings settings_[3][3];
  // Zeitpläne je Phase [3]
  LightSchedule schedules_[3];

  // Regelungsparameter
  float kpFan_              = 18.0f;
  float kiFan_              = 0.015f;   // (%/s)/kPa
  float deadband_           = 0.06f;    // kPa
  float rateLimitPctPerS_   = 6.0f;     // % per second
  float emaAlpha_           = 0.25f;    // VPD Glättung

  // Temperaturgrenzen / LED-Schutz / Lüfter-Boost
  float maxTemp_            = 30.0f;
  float ledReducePct_       = 5.0f;
  float tempHighFanC_       = 31.0f;
  float tempHighFanBoost_   = 15.0f;

  // Feuchte/Taupunkt-Überschreibung
  float rhHighThr_          = 85.0f;
  float dewGapMinC_         = 1.5f;
  bool  allowSilentOv_      = true;
  float humidBoost_         = 20.0f;

  // Sperren wenn außen feuchter
  bool  blockOutsideHumid_  = true;
  float dpHumidHyst_        = 0.5f;

  // Trocknungsmodus
  bool  dryingMode_         = false;
  bool  growActive_         = false;

  // Tür/Transiente
  float doorDeltaRh_        = 8.0f;
  float doorDeltaT_         = 2.0f;
  uint32_t doorHoldMs_      = 15000;
  uint32_t modeHoldMs_      = 8000;

  // IO / Zustand
  SHT41Ctrl*  sensorIn_  = nullptr;
  SHT41Ctrl*  sensorOut_ = nullptr;
  GP8211Ctrl* led_       = nullptr;
  FanCtrl*    fan_       = nullptr;
  StepperCtrl* step_     = nullptr;
  ToFCtrl*     tof_      = nullptr;
  RTC_Ctrl*    rtc_      = nullptr;
  int          doorPin_  = -1; // LOW = Tür zu (gegen GND)

  double tIn_ = NAN, rhIn_ = NAN, vpdIn_ = NAN, dpIn_ = NAN;
  double tOut_ = NAN, rhOut_ = NAN, dpOut_ = NAN;
  double vpdFilt_ = NAN, prevTIn_ = NAN, prevRhIn_ = NAN;

  float  ledOut_ = 0.0f, fanOut_ = 0.0f, lastFanOut_ = 0.0f;
  float  ledApplied_ = NAN; // zuletzt an den DAC geschrieben
  int8_t fanSign_ = +1; // +1: Lüfter erhöht VPD; -1: Lüfter senkt VPD

  // PI Zustand & Zeit
  float    iTermFan_  = 0.0f;
  uint32_t lastMs_    = 0;
  uint32_t holdUntilMs_ = 0;

  // Abstand / ToF / Stepper Laufzeit
  bool     initialApproachDone_ = false;
  uint32_t lastTofReadMs_ = 0;
  int      lastTofMm_ = -1; // -1 ungültig / außerhalb Messbereich, -2 zu nah
  bool     lastDoorClosed_ = true;
  uint32_t doorLastChangeMs_ = 0;

  // Zeitplan-Auslöser (einmal pro Ereignis/Tag) – 64-bit, um Überläufe zu vermeiden
  unsigned long long lastSunriseToken_ = 0ULL; // yyyymmddHHMM
  unsigned long long lastSunsetToken_  = 0ULL;

  // Nachregel-Fenster nach Events (Tür zu / Sunrise / Sunset)
  bool     adjustActive_ = false;
  uint32_t adjustUntilMs_ = 0;

  // Schimmelprävention / Per-Phase-Grenzen
  float rhCap_[3]         = {80.0f, 65.0f, 50.0f};
  float rhCapHyst_[3]     = {3.0f,  3.0f,  2.0f};
  float dpGapMin_[3]      = {2.0f,  5.0f,  7.0f};
  float minTempPhase_[3]  = {20.0f, 20.5f, 20.0f};
  float minTempFanMaxScale_ = 0.6f;  // Deckel-Faktor für FanMax bei Untertemperatur
  bool  humidityPriorityStrict_ = true;
  bool  humidityPriorityActive_ = false;
  // Humidity-Priority Cooldown
  uint32_t hpCooldownMs_ = 30000;
  uint32_t hpCooldownUntilMs_ = 0;
};

} // namespace plant_ctrl
