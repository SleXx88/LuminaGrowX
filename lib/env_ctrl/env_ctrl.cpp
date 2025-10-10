// ==============================
// File: src/env_ctrl.cpp
// ==============================
/*
 * env_ctrl.cpp
 */

#include "env_ctrl.h"
#include "../../include/lumina_config.h" // zentrale Defaults übernehmen
#include <math.h>

using namespace env_ctrl;
using vpd_calc::computeVpd;
using vpd_calc::computeDewPoint;

static inline float midpoint(float a, float b) { return (a + b) * 0.5f; }

EnvCtrl::EnvCtrl() {
  // Defaults aus zentraler Config übernehmen (Phasen/Modi)
  for (int si = 0; si < 3; ++si) {
    for (int mi = 0; mi < 3; ++mi) {
      settings_[si][mi] = lumina::defaults::PHASE_MODE[si][mi];
    }
  }
  // Hinweis: Regel-Parameter werden in applyLuminaConfig() gesetzt
}

void EnvCtrl::begin(SHT41Ctrl& sensorIndoor,
                    SHT41Ctrl& sensorOutdoor,
                    GP8211Ctrl& led,
                    FanCtrl& fan) {
  sensorIn_  = &sensorIndoor;
  sensorOut_ = &sensorOutdoor;
  led_       = &led;
  fan_       = &fan;
  lastMs_    = millis();
  lastFanOut_ = fanOut_ = settings_[idxStage(stage_)][idxMode(mode_)].fanMin;
  // LED sofort auf den Basiswert setzen
  ledOut_ = settings_[idxStage(stage_)][idxMode(mode_)].ledPercent;
  if (led_) led_->setPercent(ledOut_);
}

void EnvCtrl::setStage(vpd_calc::GrowthStage s) {
  stage_ = s;
  holdUntilMs_ = millis() + modeHoldMs_; // kurze Haltezeit nach Wechsel
}

void EnvCtrl::setMode(DayMode m) {
  mode_ = m;
  holdUntilMs_ = millis() + modeHoldMs_; // kurze Haltezeit nach Wechsel
}

void EnvCtrl::setStageModeSettings(vpd_calc::GrowthStage st, DayMode md,
                                   float ledPercent, float fanMin, float fanMax,
                                   float vpdMin, float vpdMax) {
  int si = idxStage(st), mi = idxMode(md);
  if (ledPercent < 0) ledPercent = 0; if (ledPercent > 100) ledPercent = 100;
  if (fanMin < 0) fanMin = 0; if (fanMax > 100) fanMax = 100;
  if (fanMax < fanMin) fanMax = fanMin;
  if (vpdMin < 0) vpdMin = 0; if (vpdMax < vpdMin) vpdMax = vpdMin;
  settings_[si][mi] = { ledPercent, fanMin, fanMax, vpdMin, vpdMax };
}

void EnvCtrl::setKpFan(float kp)                 { kpFan_ = kp; }
void EnvCtrl::setDeadband(float vpd_kPa)         { deadband_ = fabs(vpd_kPa); }
void EnvCtrl::setRateLimit(float pct_per_s)      { rateLimitPctPerS_ = fabs(pct_per_s); }
void EnvCtrl::setSmoothingAlpha(float alpha)     { emaAlpha_ = clamp(alpha, 0.0f, 1.0f); }

void EnvCtrl::setMaxTemperature(float tC)        { maxTemp_ = tC; }
void EnvCtrl::setOverTempReduction(float pct)    { ledReducePct_ = fabs(pct); }
void EnvCtrl::setTempHighFanBoost(float tC, float boost) { tempHighFanC_ = tC; tempHighFanBoost_ = fabs(boost); }

void EnvCtrl::setHumidityOverride(float rh_high, float dew_gap_minC,
                                  bool allow_silent_override, float extraBoost) {
  rhHighThr_     = clamp(rh_high, 0.0f, 100.0f);
  dewGapMinC_    = dew_gap_minC;
  allowSilentOv_ = allow_silent_override;
  humidBoost_    = fabs(extraBoost);
}

void EnvCtrl::setDoorDetection(float dRh, float dT, uint32_t hold_ms) {
  doorDeltaRh_ = fabs(dRh);
  doorDeltaT_  = fabs(dT);
  doorHoldMs_  = hold_ms;
}

void EnvCtrl::setModeChangeHold(uint32_t hold_ms) {
  modeHoldMs_ = hold_ms;
}

void EnvCtrl::applyLuminaConfig() {
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

float EnvCtrl::limitRate(float tgt, float last, float dt_s) const {
  const float maxDelta = rateLimitPctPerS_ * (dt_s > 0 ? dt_s : 0.0f);
  const float lo = last - maxDelta;
  const float hi = last + maxDelta;
  return clamp(tgt, lo, hi);
}

void EnvCtrl::detectDoorOrTransient(float t_now, float rh_now, uint32_t now_ms) {
  if (!isnan(prevTIn_) && !isnan(prevRhIn_)) {
    const float dT  = fabs(t_now - prevTIn_);
    const float dRh = fabs(rh_now - prevRhIn_);
    if (dT >= doorDeltaT_ || dRh >= doorDeltaRh_) {
      // Tür geöffnet / starker Transient → kurze Haltezeit
      holdUntilMs_ = max(holdUntilMs_, now_ms + doorHoldMs_);
    }
  }
  prevTIn_ = t_now;
  prevRhIn_   = rh_now;
}

// Vorzeichen aus Taupunktdifferenz bestimmen (Proxy für absolute Feuchte)
void EnvCtrl::updateFanSignFromDewpoints() {
  // dpOut << dpIn  => außen trockener => Fan↑ trocknet => VPD↑ => fanSign = +1
  // dpOut >> dpIn  => außen feuchter  => Fan↑ befeuchtet => VPD↓ => fanSign = -1
  if (isnan(dpIn_) || isnan(dpOut_)) return;
  const double gap = dpOut_ - dpIn_;
  // Hysterese: erst ab ±0.5 °C umschalten
  if (gap > 0.5)       fanSign_ = -1;
  else if (gap < -0.5) fanSign_ = +1;
  // in der Nähe von 0: Sign beibehalten
}

bool EnvCtrl::update() {
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

  // 2) VPD (innen) & Taupunkte berechnen
  vpdIn_ = computeVpd(tIn_, rhIn_);
  dpIn_  = computeDewPoint(tIn_, rhIn_);
  dpOut_ = computeDewPoint(tOut_, rhOut_);
  vpdFilt_ = emaAlpha_ * vpdIn_ + (1.0 - emaAlpha_) * vpdFilt_;

  // 3) Transienten erkennen (Tür, Stoßlüften, etc.) nur auf INDOOR
  detectDoorOrTransient(tIn_, rhIn_, now);

  // 4) Basiswerte laden
  PhaseModeSettings& ps = cur();
  const float vpdTarget = midpoint(ps.vpdMin, ps.vpdMax);
  // Bias controller toward minimum to allow very low airflow when humidifying
  const float baseFan   = ps.fanMin;
  float ledTarget       = ps.ledPercent;

  // 5) LED Übertemperatur-Reduktion
  if (tIn_ > maxTemp_) {
    ledTarget = clamp01(ledTarget - ledReducePct_);
  }
  if (led_) {
    if (fabs(ledTarget - ledOut_) >= 0.5f) {
      ledOut_ = ledTarget;
      led_->setPercent(ledOut_);
    } else {
      ledOut_ = ledTarget;
    }
  }

  // 6) „Außen feuchter ⇒ Fan aus“ (harte Blockade, träge & logisch)
  // Taupunkt außen deutlich höher als innen ⇒ Außenluft befeuchtet die Box.
  if (blockOutsideHumid_ && (!isnan(dpIn_) && !isnan(dpOut_)) && (dpOut_ >= dpIn_ + dpHumidHyst_)) {
    float fanCmd = 0.0f;                           // wirklich AUS
    fanCmd = limitRate(fanCmd, lastFanOut_, dt_s); // sanft auslaufen
    lastFanOut_ = fanOut_ = fanCmd;
    fan_->setPercent(fanOut_);
    // Integrator langsam abbauen (kein Windup im Block-Zustand)
    iTermFan_ *= 0.98f;
    return true;
  }

  // 7) Overrides (Feuchte/Taupunkt/Temperatur) – höchste Priorität
  bool overrideActive = false;
  const double dewGapIn = tIn_ - dpIn_; // Abstand zu Taupunkt innen

  float fanCmd = baseFan + iTermFan_; // PI-Basis
  if (rhIn_ >= rhHighThr_ || dewGapIn <= dewGapMinC_) {
    float maxLimit = ps.fanMax;
    if (mode_ == DayMode::NightSilent && allowSilentOv_) {
      maxLimit = fminf(100.0f, maxLimit + 20.0f); // Silent kurzzeitig lockern
    }
    fanCmd = clamp(fmaxf(ps.fanMin + humidBoost_, fanOut_), ps.fanMin, maxLimit);
    overrideActive = true;
  }
  if (!overrideActive && tIn_ >= tempHighFanC_) {
    fanCmd = clamp(fmaxf(ps.fanMin + tempHighFanBoost_, fanOut_), ps.fanMin, ps.fanMax);
    overrideActive = true;
  }

  // 8) Haltezeiten (Tür/Modus) – mittlere Priorität
  if (!overrideActive && now < holdUntilMs_) {
    fanCmd = clamp(baseFan + iTermFan_, ps.fanMin, ps.fanMax);
  }

  // 9) PI-Regelung nur wenn keine Overrides/Hold aktiv
  if (!overrideActive && now >= holdUntilMs_) {
    // Polarity aus Taupunkten (Außen/Innen) ableiten
    updateFanSignFromDewpoints();

    // Use a local deadband tied to the configured VPD range
    const float vpdRange = fmaxf(0.0f, ps.vpdMax - ps.vpdMin);
    const float localDeadband = fminf(deadband_, 0.5f * vpdRange);

    const float error = (float)vpdFilt_ - vpdTarget;
    float e = 0.0f;
    if      (error >  localDeadband) e = error - localDeadband;
    else if (error < -localDeadband) e = error + localDeadband;
    else e = 0.0f;

    // Stellwert: Fan = Base + I - sign*Kp*e
    fanCmd = baseFan + iTermFan_ - (float)fanSign_ * kpFan_ * e;

    // I-Anteil – sehr klein & limitiert (Anti-Windup)
    const float iDelta = (-(float)fanSign_) * kiFan_ * e * dt_s;
    float preClamp = fanCmd + iDelta; // Tentative
    float clamped  = clamp(preClamp, ps.fanMin, ps.fanMax);
    if (fabsf(preClamp - clamped) < 1e-3f) {
      iTermFan_ += iDelta;
      if (iTermFan_ >  20.0f) iTermFan_ =  20.0f;
      if (iTermFan_ < -20.0f) iTermFan_ = -20.0f;
      fanCmd = clamped;
    } else {
      // Bei Sättigung Integrator leicht abbauen
      iTermFan_ *= 0.995f;
      fanCmd = clamped;
    }
  } else {
    // Integrator langsam entspannen, wenn wir nicht regeln
    iTermFan_ *= 0.995f;
  }

  // 10) Rate-Limiter anwenden + Clamp
  fanCmd = clamp(fanCmd, ps.fanMin, ps.fanMax);
  fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
  lastFanOut_ = fanOut_ = fanCmd;
  fan_->setPercent(fanOut_);

  return true;
}
