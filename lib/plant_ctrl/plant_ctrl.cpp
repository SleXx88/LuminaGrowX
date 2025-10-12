// ==============================
// Datei: lib/plant_ctrl/plant_ctrl.cpp
// ==============================

#include "plant_ctrl.h"
#include "../../include/lumina_config.h" // zentrale Defaults/Parameter
#include <math.h>

using namespace plant_ctrl;
using vpd_calc::computeVpd;
using vpd_calc::computeDewPoint;

static inline float midpoint(float a, float b) { return (a + b) * 0.5f; }

PlantCtrl::PlantCtrl() {
  // Standardwerte aus zentraler Konfiguration (Phasen/Modi) übernehmen
  for (int si = 0; si < 3; ++si) {
    for (int mi = 0; mi < 3; ++mi) {
      settings_[si][mi] = lumina::defaults::PHASE_MODE[si][mi];
    }
  }
}

void PlantCtrl::begin(SHT41Ctrl& sensorIndoor,
                      SHT41Ctrl& sensorOutdoor,
                      GP8211Ctrl& led,
                      FanCtrl& fan,
                      StepperCtrl& stepper,
                      ToFCtrl& tof,
                      RTC_Ctrl* rtc,
                      int doorPin) {
  sensorIn_  = &sensorIndoor;
  sensorOut_ = &sensorOutdoor;
  led_       = &led;
  fan_       = &fan;
  step_      = &stepper;
  tof_       = &tof;
  rtc_       = rtc;
  doorPin_   = doorPin;
  if (doorPin_ >= 0) {
    pinMode(doorPin_, INPUT_PULLUP); // LOW = Tür zu (gegen GND)
    lastDoorClosed_ = (digitalRead(doorPin_) == LOW);
  } else {
    lastDoorClosed_ = true; // ohne Sensor: als geschlossen behandeln
  }

  lastMs_    = millis();
  lastFanOut_ = fanOut_ = settings_[idxStage(stage_)][idxMode(mode_)].fanMin;
  // LED auf Basiswert setzen
  ledOut_ = settings_[idxStage(stage_)][idxMode(mode_)].ledPercent;
  if (led_) led_->setPercent(ledOut_);
  initialApproachDone_ = false;
}

void PlantCtrl::setStage(vpd_calc::GrowthStage s) {
  stage_ = s;
  holdUntilMs_ = millis() + modeHoldMs_;
}

void PlantCtrl::setMode(DayMode m) {
  mode_ = m;
  holdUntilMs_ = millis() + modeHoldMs_;
}

void PlantCtrl::setStageModeSettings(vpd_calc::GrowthStage st, DayMode md,
                                     float ledPercent, float fanMin, float fanMax,
                                     float vpdMin, float vpdMax) {
  int si = idxStage(st), mi = idxMode(md);
  if (ledPercent < 0) ledPercent = 0; if (ledPercent > 100) ledPercent = 100;
  if (fanMin < 0) fanMin = 0; if (fanMax > 100) fanMax = 100;
  if (fanMax < fanMin) fanMax = fanMin;
  if (vpdMin < 0) vpdMin = 0; if (vpdMax < vpdMin) vpdMax = vpdMin;
  settings_[si][mi] = { ledPercent, fanMin, fanMax, vpdMin, vpdMax };
}

void PlantCtrl::setKpFan(float kp)                 { kpFan_ = kp; }
void PlantCtrl::setDeadband(float vpd_kPa)         { deadband_ = fabsf(vpd_kPa); }
void PlantCtrl::setRateLimit(float pct_per_s)      { rateLimitPctPerS_ = fabsf(pct_per_s); }
void PlantCtrl::setSmoothingAlpha(float alpha)     { emaAlpha_ = clamp(alpha, 0.0f, 1.0f); }

void PlantCtrl::setMaxTemperature(float tC)        { maxTemp_ = tC; }
void PlantCtrl::setOverTempReduction(float pct)    { ledReducePct_ = fabsf(pct); }
void PlantCtrl::setTempHighFanBoost(float tC, float boost) { tempHighFanC_ = tC; tempHighFanBoost_ = fabsf(boost); }

void PlantCtrl::setHumidityOverride(float rh_high, float dew_gap_minC,
                                    bool allow_silent_override, float extraBoost) {
  rhHighThr_     = clamp(rh_high, 0.0f, 100.0f);
  dewGapMinC_    = dew_gap_minC;
  allowSilentOv_ = allow_silent_override;
  humidBoost_    = fabsf(extraBoost);
}

void PlantCtrl::setDoorDetection(float dRh, float dT, uint32_t hold_ms) {
  doorDeltaRh_ = fabsf(dRh);
  doorDeltaT_  = fabsf(dT);
  doorHoldMs_  = hold_ms;
}

void PlantCtrl::setModeChangeHold(uint32_t hold_ms) { modeHoldMs_ = hold_ms; }

void PlantCtrl::applyLuminaConfig() {
  const auto& p = lumina::ctrl::PARAMS;
  setKpFan(p.Kp);
  setKiFan(p.Ki);
  setDeadband(p.deadband_kPa);
  setRateLimit(p.rate_limit_pct_s);
  setSmoothingAlpha(p.ema_alpha);

  setOutsideHumidBlock(p.outside_humid_block, p.dp_hysteresis_C);

  setHumidityOverride(p.rh_high_thr, p.dew_gap_min_C, p.allow_silent_override, p.humid_boost_pct);
  setMaxTemperature(p.max_temp_C);
  setOverTempReduction(p.led_reduce_pct);
  setTempHighFanBoost(p.temp_high_fan_C, p.temp_high_fan_boost);
  setDoorDetection(p.door_dRh, p.door_dT, p.door_hold_ms);
  setModeChangeHold(p.mode_hold_ms);
}

float PlantCtrl::limitRate(float tgt, float last, float dt_s) const {
  const float maxDelta = rateLimitPctPerS_ * (dt_s > 0 ? dt_s : 0.0f);
  const float lo = last - maxDelta;
  const float hi = last + maxDelta;
  return clamp(tgt, lo, hi);
}

void PlantCtrl::detectDoorOrTransient(float t_now, float rh_now, uint32_t now_ms) {
  if (!isnan(prevTIn_) && !isnan(prevRhIn_)) {
    const float dT  = fabsf(t_now - prevTIn_);
    const float dRh = fabsf(rh_now - prevRhIn_);
    if (dT >= doorDeltaT_ || dRh >= doorDeltaRh_) {
      if (now_ms + doorHoldMs_ > holdUntilMs_) holdUntilMs_ = now_ms + doorHoldMs_;
    }
  }
  prevTIn_ = t_now;
  prevRhIn_ = rh_now;
}

void PlantCtrl::updateFanSignFromDewpoints() {
  float gap = (float)(dpOut_ - dpIn_);
  if (gap > 0.5f)       fanSign_ = -1; // außen feuchter
  else if (gap < -0.5f) fanSign_ = +1; // außen trockener
  // nahe 0: Vorzeichen beibehalten
}

bool PlantCtrl::update() {
  if (!sensorIn_ || !sensorOut_ || !fan_ || !led_) return false;

  const uint32_t now = millis();
  const float dt_s = (lastMs_ == 0) ? 0.0f : (float)(now - lastMs_) / 1000.0f;
  lastMs_ = now;

  // 1) Sensoren lesen
  float tC_in, rh_in, tC_out, rh_out;
  if (!sensorIn_->read(tC_in, rh_in)) return false;
  if (!sensorOut_->read(tC_out, rh_out)) return false;
  tIn_ = tC_in; rhIn_ = rh_in;
  tOut_ = tC_out; rhOut_ = rh_out;

  // 2) VPD (innen) & Taupunkte
  vpdIn_ = computeVpd(tIn_, rhIn_);
  dpIn_  = computeDewPoint(tIn_, rhIn_);
  dpOut_ = computeDewPoint(tOut_, rhOut_);
  vpdFilt_ = emaAlpha_ * vpdIn_ + (1.0f - emaAlpha_) * vpdFilt_;

  // 3) Tür-/Transienten-Erkennung
  detectDoorOrTransient(tIn_, rhIn_, now);

  // 4) Basiswerte laden
  PhaseModeSettings& ps = cur();
  const float vpdTarget = midpoint(ps.vpdMin, ps.vpdMax);
  const float baseFan   = ps.fanMin;
  float ledTarget       = ps.ledPercent;

  // 5) LED-Reduktion bei Übertemperatur
  if (tIn_ > maxTemp_) {
    ledTarget = clamp01(ledTarget - ledReducePct_);
  }
  if (led_) {
    if (fabsf(ledTarget - ledOut_) >= 0.5f) {
      ledOut_ = ledTarget;
      led_->setPercent(ledOut_);
    } else {
      ledOut_ = ledTarget;
    }
  }

  // 6) Außen feuchter -> Lüfter blockieren
  if (blockOutsideHumid_ && (!isnan(dpIn_) && !isnan(dpOut_)) && (dpOut_ >= dpIn_ + dpHumidHyst_)) {
    float fanCmd = 0.0f;
    fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
    lastFanOut_ = fanOut_ = fanCmd;
    fan_->setPercent(fanOut_);
    iTermFan_ *= 0.98f; // Integrator abbauen
    // Distanzverwaltung trotzdem laufen lassen
    distanceTick_(now);
    return true;
  }

  // 7) Overrides (Feuchte/Taupunkt/Temperatur)
  bool overrideActive = false;
  const double dewGapIn = tIn_ - dpIn_;

  float fanCmd = baseFan + iTermFan_;
  if (rhIn_ >= rhHighThr_ || dewGapIn <= dewGapMinC_) {
    float maxLimit = ps.fanMax;
    if (mode_ == DayMode::NightSilent && allowSilentOv_) {
      maxLimit = fminf(100.0f, maxLimit + 20.0f);
    }
    fanCmd = clamp(fmaxf(ps.fanMin + humidBoost_, fanOut_), ps.fanMin, maxLimit);
    overrideActive = true;
  }
  if (!overrideActive && tIn_ >= tempHighFanC_) {
    fanCmd = clamp(fmaxf(ps.fanMin + tempHighFanBoost_, fanOut_), ps.fanMin, ps.fanMax);
    overrideActive = true;
  }

  // 8) Holds (Tür/Modus)
  if (!overrideActive && now < holdUntilMs_) {
    fanCmd = clamp(baseFan + iTermFan_, ps.fanMin, ps.fanMax);
  }

  // 9) PI-Regelung
  if (!overrideActive && now >= holdUntilMs_) {
    updateFanSignFromDewpoints();

    const float vpdRange = fmaxf(0.0f, ps.vpdMax - ps.vpdMin);
    const float localDeadband = fminf(deadband_, 0.5f * vpdRange);

    const float error = (float)vpdFilt_ - vpdTarget;
    float e = 0.0f;
    if      (error >  localDeadband) e = error - localDeadband;
    else if (error < -localDeadband) e = error + localDeadband;
    else e = 0.0f;

    fanCmd = baseFan + iTermFan_ - (float)fanSign_ * kpFan_ * e;

    const float iDelta = (-(float)fanSign_) * kiFan_ * e * dt_s;
    float preClamp = fanCmd + iDelta;
    float clamped  = clamp(preClamp, ps.fanMin, ps.fanMax);
    if (fabsf(preClamp - clamped) < 1e-3f) {
      iTermFan_ += iDelta;
      if (iTermFan_ >  20.0f) iTermFan_ =  20.0f;
      if (iTermFan_ < -20.0f) iTermFan_ = -20.0f;
      fanCmd = clamped;
    } else {
      iTermFan_ *= 0.995f;
      fanCmd = clamped;
    }
  } else {
    iTermFan_ *= 0.995f;
  }

  // 10) Rate-Limiter + setzen
  fanCmd = clamp(fanCmd, ps.fanMin, ps.fanMax);
  fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
  lastFanOut_ = fanOut_ = fanCmd;
  fan_->setPercent(fanOut_);

  // Distanzverwaltung (Tür/Schedule/Eventfenster + 1 mm Schritte)
  distanceTick_(now);

  return true;
}

// Türlogik: LOW = Tür zu (gegen GND)
bool PlantCtrl::isDoorClosed_() {
  if (doorPin_ < 0) return true;
  return digitalRead(doorPin_) == LOW;
}

static inline int makeTokenYMDHM(int y, int m, int d, int hh, int mm) {
  return (((y*100 + m)*100 + d)*100 + hh)*100 + mm;
}

bool PlantCtrl::allowDownAdjustNow_(uint32_t now) {
  bool closed = isDoorClosed_();
  if (closed != lastDoorClosed_) {
    lastDoorClosed_ = closed;
    doorLastChangeMs_ = now;
    if (closed) {
      Serial.println(F("[DOOR] geschlossen"));
      return true; // Kante: offen->geschlossen
    } else {
      Serial.println(F("[DOOR] offen"));
      adjustActive_ = false; // Sicherheits-Abbruch
    }
  }

  if (rtc_) {
    uint16_t y; uint8_t mo, d, hh, mi, ss;
    if (rtc_->readComponents(y, mo, d, hh, mi, ss)) {
      // Zeitplan je nach Phase
      const lumina::LightSchedule* sch = nullptr;
      using vpd_calc::GrowthStage;
      switch (stage_) {
        case GrowthStage::Seedling:   sch = &lumina::schedule::SEEDLING; break;
        case GrowthStage::Vegetative: sch = &lumina::schedule::VEGETATIVE; break;
        case GrowthStage::Flowering:  sch = &lumina::schedule::FLOWERING; break;
        default: sch = &lumina::schedule::VEGETATIVE; break;
      }
      if (sch) {
        int token = makeTokenYMDHM(y, mo, d, hh, mi);
        if (hh == sch->on.hour  && mi == sch->on.minute  && token != lastSunriseToken_) { lastSunriseToken_ = token; Serial.println(F("[SCHEDULE] Sunrise adjust")); return true; }
        if (hh == sch->off.hour && mi == sch->off.minute && token != lastSunsetToken_)  { lastSunsetToken_  = token; Serial.println(F("[SCHEDULE] Sunset adjust"));  return true; }
      }
    }
  }
  return false;
}

float PlantCtrl::targetDistanceMm_() const {
  const float* arr = lumina::plant::MIN_DIST_MM;
  int idx = idxStage(stage_);
  return arr ? arr[idx] : 100.0f;
}

void PlantCtrl::distanceTick_(uint32_t now) {
  if (!step_ || !tof_) return;
  // Tür offen? Keine Bewegung zulassen
  if (!isDoorClosed_()) { adjustActive_ = false; return; }

  // ToF lesen
  if (now - lastTofReadMs_ >= lumina::plant::TOF_READ_INTERVAL_MS) {
    lastTofReadMs_ = now;
    int v = tof_->readAvgMm(lumina::plant::TOF_AVG_SAMPLES);
    if (v >= 0) lastTofMm_ = v;
  }
  if (lastTofMm_ < 0) return;

  const float offset = (float)lumina::plant::TOF_LED_OFFSET_MM;
  const float ledPlantMm = (float)lastTofMm_ + offset;
  const float target = targetDistanceMm_();
  const float hyst = lumina::plant::ADJUST_HYST_MM;
  const float stepMm = lumina::plant::ADJUST_STEP_MM; // 1 mm Schritte im Normalbetrieb
  const uint8_t spd = lumina::plant::SPEED_LEVEL;

  // Ereignisfenster (Tür-zu / Sunrise / Sunset) aktivieren
  if (allowDownAdjustNow_(now)) { adjustActive_ = true; adjustUntilMs_ = now + lumina::plant::ADJUST_WINDOW_MS; }
  if (!adjustActive_) return;
  if (now > adjustUntilMs_) { adjustActive_ = false; return; }

  // Nur bewegen, wenn Stepper frei
  if (step_->status().isMoving) return;

  if (ledPlantMm < (target - hyst)) {
    float delta = fminf(stepMm, (target - ledPlantMm));
    step_->moveUp(delta, spd);
  } else if (ledPlantMm > (target + hyst)) {
    float delta = fminf(stepMm, (ledPlantMm - target));
    step_->moveDown(delta, spd);
  } else {
    adjustActive_ = false; // Ziel erreicht
  }
}

// Dynamische, blockierende Annäherung beim Setup
void PlantCtrl::runStartupApproachBlocking() {
  if (!step_ || !tof_) { initialApproachDone_ = true; return; }

  const float target = targetDistanceMm_();
  const float offset = (float)lumina::plant::TOF_LED_OFFSET_MM;
  const float hyst   = lumina::plant::ADJUST_HYST_MM;
  const uint8_t spd  = lumina::plant::SPEED_LEVEL;

  uint32_t lastRead = 0;
  for (;;) {
    uint32_t now = millis();
    if (now - lastRead >= lumina::plant::TOF_READ_INTERVAL_MS) {
      lastRead = now;
      int v = tof_->readAvgMm(lumina::plant::TOF_AVG_SAMPLES);
      if (v >= 0) lastTofMm_ = v;
    }
    if (lastTofMm_ >= 0) {
      float ledPlantMm = (float)lastTofMm_ + offset;
      if (ledPlantMm > (target + hyst)) {
        float err = ledPlantMm - target;
        // dynamische Schrittweite
        float stepMm = 1.0f;
        if (err > 50.0f) stepMm = 10.0f;
        else if (err > 20.0f) stepMm = 5.0f;
        else if (err > 10.0f) stepMm = 3.0f;
        else stepMm = 1.0f;
        float delta = fminf(stepMm, err);
        step_->moveDown(delta, spd);
        // auf Bewegungsende warten
        for (;;) { step_->tick(); auto s = step_->status(); if (!s.isMoving || s.lastOpDone) break; delay(1); }
        continue;
      }
      break; // innerhalb Bandbreite
    }
    // noch kein gültiger Messwert
    step_->tick();
    delay(5);
  }
  initialApproachDone_ = true;
}
