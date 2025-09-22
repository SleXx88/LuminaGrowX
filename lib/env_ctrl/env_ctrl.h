/*
 * env_ctrl.h
 *
 * Zweck
 * -----
 * Diese Library steuert die Umweltbedingungen einer kompakten Growbox (ESP32).
 * Regelgröße ist der VPD (Vapor Pressure Deficit) im Innenraum, gemessen über
 * Temperatur und rel. Feuchte (SHT41). Geregelt wird ausschließlich der LÜFTER.
 * Die LED hat pro Phase/Modus einen festen Wert und wird nur bei Übertemperatur
 * temporär etwas abgesenkt.
 *
 * Praxis-Tücken & Robustheit
 * --------------------------
 * - Außenluft kann mal trockener, mal feuchter sein → Wirkung „mehr Lüften“
 *   auf den VPD ist NICHT immer gleich. Daher: *Adaptive Polarity*.
 * - Plötzliche Änderungen (Tür öffnen, Tag/Nacht-Umschaltung) → Transienten.
 *   Daher: Haltezeiten (Hold), sanfte Rampen & Rate-Limiter.
 * - Schimmel-/Kondensationsgefahr → Feuchte-/Taupunkt-Override hebt Lüfter
 *   kurzzeitig an (auch in Silent).
 *
 * Einbindung
 * ----------
 *  SHT41Ctrl  liest T/RH   → vpd_calc::computeVpd()/dewPoint() rechnen VPD/Taupunkt
 *  GP8211Ctrl setzt LED %   → fester LED-Basiswert je Phase/Modus (+ Übertemp-Reduktion)
 *  FanCtrl    setzt Lüfter% → Proportionalregelung auf VPD + Rate-Limit + Overrides
 *
 */

#pragma once

#include <Arduino.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "vpd_calc.h"

namespace env_ctrl {

// Tagesmodi
enum class DayMode : uint8_t { Day=0, Night=1, NightSilent=2 };

// Konfiguration je Pflanzenphase & Modus
struct PhaseModeSettings {
  float ledPercent;   // fester LED-Wert in % (0..100)
  float fanMin;       // Lüfter-Min in %
  float fanMax;       // Lüfter-Max in %
  float vpdMin;       // Zielbereich VPD in kPa (untere Grenze)
  float vpdMax;       // Zielbereich VPD in kPa (obere Grenze)
};

class EnvCtrl {
public:
  EnvCtrl();

  // Hardware binden (Objekte müssen extern bereits initialisiert sein)
  void begin(SHT41Ctrl& sensor, GP8211Ctrl& led, FanCtrl& fan);

  // Phase/Modus setzen
  void setStage(vpd_calc::GrowthStage stage);
  void setMode(DayMode mode);

  // Werte je Phase/Modus überschreiben (z. B. via Menü/Main)
  void setStageModeSettings(vpd_calc::GrowthStage st, DayMode md,
                            float ledPercent, float fanMin, float fanMax,
                            float vpdMin, float vpdMax);

  // Stellgrößen- und Robustheits-Parameter
  void setKpFan(float kp);                        // Proportionalfaktor Lüfter
  void setDeadband(float vpd_kPa);                // Totband um Soll (kPa)
  void setRateLimit(float pct_per_s);             // max. % Änderung pro Sekunde
  void setSmoothingAlpha(float alpha);            // 0..1, EMA für VPD

  // Adaptive Polarity (Fan↑ → VPD↑ oder ↓ automatisch ermitteln)
  void setAdaptiveTest(float step_percent, uint32_t interval_ms,
                       uint32_t duration_ms, float min_error_kPa);

  // Temperatur-Überwachung (LED/ Lüfter-Schutz)
  void setMaxTemperature(float tempC);            // LED ab hier reduzieren
  void setOverTempReduction(float pct_points);    // um wieviel %-Punkte LED runter
  void setTempHighFanBoost(float tempC, float boostPercent);

  // Feuchte-/Taupunkt-Override
  void setHumidityOverride(float rh_high_percent, float dewpoint_gap_minC,
                           bool allow_silent_override, float extraBoostPercent);

  // Tür-/Transienten-Erkennung
  void setDoorDetection(float deltaRh_percent, float deltaT_C, uint32_t hold_ms);

  // Haltezeit nach Moduswechsel
  void setModeChangeHold(uint32_t hold_ms);

  // Zyklisch aufrufen; gibt false zurück, wenn Sensorlesung fehlschlägt
  bool update();

  // Zustände abfragen
  double currentVpd()  const { return vpd_; }
  double currentTemp() const { return temp_; }
  double currentRh()   const { return rh_; }
  double currentVpdFiltered() const { return vpdFilt_; }

  float  currentLedPercent() const { return ledOut_; }
  float  currentFanPercent() const { return fanOut_; }

  // Debug/Status
  int8_t learnedFanPolarity() const { return fanSign_; } // +1: Fan↑→VPD↑, -1: Fan↑→VPD↓
  bool   isHoldActive() const { return millis() < holdUntilMs_; }
  bool   isTestActive() const { return testActive_; }

private:
  // Indizes für Arrays
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

  // Klemmen
  static float clamp01(float p) { if (p<0) return 0; if (p>100) return 100; return p; }
  static float clamp(float x, float a, float b) { return x<a? a : (x>b? b : x); }

  // Intern: einen Schritt Stellgröße begrenzen (Rate-Limiter)
  float limitRate(float tgt, float last, float dt_s) const;

  // Intern: Tür-/Transienten-Erkennung
  void detectDoorOrTransient(float t_now, float rh_now, uint32_t now_ms);

  // Intern: Adaptive Polarity Test
  void maybeStartTest(float error_kPa, uint32_t now_ms);
  void maybeFinishTest(uint32_t now_ms);

private:
  // Phase/Modus
  vpd_calc::GrowthStage stage_ = vpd_calc::GrowthStage::Seedling;
  DayMode mode_ = DayMode::Day;

  // Tabellen mit Defaultwerten [3][3]
  PhaseModeSettings settings_[3][3];

  // Regelparameter
  float kpFan_              = 20.0f;
  float deadband_           = 0.05f;     // kPa
  float rateLimitPctPerS_   = 5.0f;      // % pro Sekunde
  float emaAlpha_           = 0.25f;     // Glättung VPD

  // Adaptive Polarity
  bool     adaptive_        = true;
  float    testStepPct_     = 5.0f;
  uint32_t testIntervalMs_  = 90000;
  uint32_t testDurationMs_  = 7000;
  float    minErrorForTest_ = 0.08f;
  uint32_t lastTestMs_      = 0;
  bool     testActive_      = false;
  float    testFanSaved_    = 0.0f;
  double   testVpdStart_    = 0.0;
  int8_t   fanSign_         = +1; // Start-Annahme: Fan↑ → VPD↑ (trocken)
  float    fanSignBlend_    = 1.0f; // für spätere Glättung (optional)

  // Temperaturgrenzen / LED-Schutz / Lüfter-Boost
  float maxTemp_            = 30.0f;
  float ledReducePct_       = 5.0f;
  float tempHighFanC_       = 31.0f;
  float tempHighFanBoost_   = 15.0f;  // %-Punkte oberhalb fanMin

  // Feuchte-/Taupunkt-Override
  float rhHighThr_          = 85.0f;
  float dewGapMinC_         = 1.5f;
  bool  allowSilentOv_      = true;
  float humidBoost_         = 20.0f;  // %-Punkte oberhalb fanMin

  // Tür-/Transientenerkennung
  float doorDeltaRh_        = 8.0f;   // %RH Sprung
  float doorDeltaT_         = 2.0f;   // °C Sprung
  uint32_t doorHoldMs_      = 15000;
  uint32_t modeHoldMs_      = 8000;

  // Zustände / Messwerte
  SHT41Ctrl*  sensor_   = nullptr;
  GP8211Ctrl* led_      = nullptr;
  FanCtrl*    fan_      = nullptr;

  double temp_ = 0.0, rh_ = 0.0, vpd_ = 0.0;
  double vpdFilt_ = 0.0, prevTemp_ = NAN, prevRh_ = NAN;
  float  ledOut_ = 0.0f, fanOut_ = 0.0f, lastFanOut_ = 0.0f;

  // Zeitverwaltung
  uint32_t lastMs_ = 0;
  uint32_t holdUntilMs_ = 0;
};

} // namespace env_ctrl