/*
 * env_ctrl.h | env → Environment (Umgebung)
 *
 * Ziel
 * ----
 * Regelung des VPD (nur über Lüfter) mit **zwei SHT41**:
 *  - INDOOR: Temperatur/Feuchte in der Box (Regelgröße)
 *  - OUTDOOR: Temperatur/Feuchte außerhalb (bestimmt Lüfter-Wirkungsrichtung)
 *
 * LED ist pro Phase/Modus fix und wird nur bei Übertemperatur leicht reduziert.
 * Vergleich von **Taupunkten** (bzw. absoluter Feuchte) innen/außen bestimmt:
 *   - Taupunkt_out  << Taupunkt_in  → Außenluft trockener  → Fan↑ macht VPD↑
 *   - Taupunkt_out  >> Taupunkt_in  → Außenluft feuchter   → Fan↑ macht VPD↓
 *
 * Robustheit:
 *  - Deadband um den Soll-VPD
 *  - Rate-Limiter für sanfte Stelländerung
 *  - Tür-/Transienten-Hold (Vermeidung von Eskalation bei plötzlichen Sprüngen)
 *  - Feuchte-/Taupunkt-Override und Temperatur-Override
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
  void begin(SHT41Ctrl& sensorIndoor,
             SHT41Ctrl& sensorOutdoor,
             GP8211Ctrl& led,
             FanCtrl& fan);

  // Phase/Modus setzen
  void setStage(vpd_calc::GrowthStage stage);
  void setMode(DayMode mode);

  // Werte je Phase/Modus überschreiben (z. B. via Menü/Main)
  void setStageModeSettings(vpd_calc::GrowthStage st, DayMode md,
                            float ledPercent, float fanMin, float fanMax,
                            float vpdMin, float vpdMax);

  // --- schlanke Regel-Parameter ---
  void setKpFan(float kp);                        // Proportionalfaktor Lüfter
  void setDeadband(float vpd_kPa);                // Totband um Soll (kPa)
  void setRateLimit(float pct_per_s);             // max. % Änderung pro Sekunde
  void setSmoothingAlpha(float alpha);            // 0..1, EMA für VPD

  // Temperatur-Überwachung (LED/ Lüfter-Schutz)
  void setMaxTemperature(float tempC);            // LED ab hier reduzieren
  void setOverTempReduction(float pct_points);    // %-Punkte LED runter
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
  double currentVpd()  const { return vpdIn_; }
  double currentTemp() const { return tIn_; }
  double currentRh()   const { return rhIn_; }
  double currentVpdFiltered() const { return vpdFilt_; }

  float  currentLedPercent() const { return ledOut_; }
  float  currentFanPercent() const { return fanOut_; }

  // Außenwerte
  double currentTempOut() const { return tOut_; }
  double currentRhOut()   const { return rhOut_; }

  // Abgeleitete Werte
  double dewPointIn()  const { return dpIn_; }
  double dewPointOut() const { return dpOut_; }
  int8_t fanVpdSign()  const { return fanSign_; } // +1: Fan↑→VPD↑, -1: Fan↑→VPD↓

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

  // Klemmen
  static float clamp01(float p) { if (p<0) return 0; if (p>100) return 100; return p; }
  static float clamp(float x, float a, float b) { return x<a? a : (x>b? b : x); }

  // Intern: einen Schritt Stellgröße begrenzen (Rate-Limiter)
  float limitRate(float tgt, float last, float dt_s) const;

  // Intern: Tür-/Transienten-Erkennung
  void detectDoorOrTransient(float t_now, float rh_now, uint32_t now_ms);

  // Intern: Polarity aus Taupunkten ableiten
  void updateFanSignFromDewpoints();

private:
  // Phase/Modus
  vpd_calc::GrowthStage stage_ = vpd_calc::GrowthStage::Seedling;
  DayMode mode_ = DayMode::Day;

  // Tabellen mit Defaultwerten [3][3]
  PhaseModeSettings settings_[3][3];

  // Regelparameter (schlank gehalten)
  float kpFan_              = 18.0f;
  float deadband_           = 0.06f;     // kPa
  float rateLimitPctPerS_   = 6.0f;      // % pro Sekunde
  float emaAlpha_           = 0.25f;     // Glättung VPD

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
  SHT41Ctrl*  sensorIn_  = nullptr;
  SHT41Ctrl*  sensorOut_ = nullptr;
  GP8211Ctrl* led_       = nullptr;
  FanCtrl*    fan_       = nullptr;

  double tIn_ = NAN, rhIn_ = NAN, vpdIn_ = NAN, dpIn_ = NAN;
  double tOut_ = NAN, rhOut_ = NAN, dpOut_ = NAN;
  double vpdFilt_ = 0.0, prevTIn_ = NAN, prevRhIn_ = NAN;

  float  ledOut_ = 0.0f, fanOut_ = 0.0f, lastFanOut_ = 0.0f;
  int8_t fanSign_ = +1; // +1: Fan↑→VPD↑ (trockener); -1: Fan↑→VPD↓ (feuchter)

  // Zeitverwaltung
  uint32_t lastMs_ = 0;
  uint32_t holdUntilMs_ = 0;
};

} // namespace env_ctrl