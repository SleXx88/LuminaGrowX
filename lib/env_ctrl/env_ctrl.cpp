/*
 * env_ctrl.cpp
 */

#include "env_ctrl.h"
#include <math.h>

using namespace env_ctrl;
using vpd_calc::computeVpd;
using vpd_calc::computeDewPoint;

static inline float midpoint(float a, float b) { return (a + b) * 0.5f; }

EnvCtrl::EnvCtrl() {
  // ------------------------------------------------------------
  // Defaultwerte für jede Pflanzenphase (Seedling, Vegetative, Flowering)
  // und Tagesmodus (Day, Night, NightSilent)
  //
  // { LED%, FanMin%, FanMax%, VPDmin[kPa], VPDmax[kPa] }
  // ------------------------------------------------------------

  // SEEDLING – hohe Feuchte, wenig Licht
  settings_[0][0] = { 40.0f, 20.0f, 80.0f, 0.40f, 0.80f }; // Day
  settings_[0][1] = {  0.0f, 20.0f, 60.0f, 0.40f, 0.80f }; // Night
  settings_[0][2] = {  0.0f, 10.0f, 40.0f, 0.40f, 0.80f }; // NightSilent

  // VEGETATIVE – stärkeres Wachstum, mehr Licht, trockener
  settings_[1][0] = { 65.0f, 20.0f, 100.0f, 0.80f, 1.20f }; // Day
  settings_[1][1] = {  0.0f, 20.0f,  60.0f, 0.80f, 1.20f }; // Night
  settings_[1][2] = {  0.0f, 10.0f,  40.0f, 0.80f, 1.20f }; // NightSilent

  // FLOWERING – maximale Lichtleistung, trockene Luft gegen Schimmel
  settings_[2][0] = { 90.0f, 30.0f, 100.0f, 1.20f, 1.60f }; // Day
  settings_[2][1] = {  0.0f, 20.0f,  70.0f, 1.20f, 1.60f }; // Night
  settings_[2][2] = {  0.0f, 10.0f,  50.0f, 1.20f, 1.60f }; // NightSilent
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
  if (gap > 0.5)      fanSign_ = -1;
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
  const float baseFan   = midpoint(ps.fanMin, ps.fanMax);
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

  // 6) Overrides (Feuchte/Taupunkt/Temperatur) – höchste Priorität
  bool overrideActive = false;
  const double dewGapIn = tIn_ - dpIn_; // Abstand zu Taupunkt innen

  float fanCmd = baseFan;
  if (rhIn_ >= rhHighThr_ || dewGapIn <= dewGapMinC_) {
    // Feuchte-Override: Lüfter deutlich anheben
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

  // 7) Haltezeiten (Tür/Modus) – mittlere Priorität
  if (!overrideActive && now < holdUntilMs_) {
    fanCmd = clamp(baseFan, ps.fanMin, ps.fanMax);
  }

  // 8) Proportionale Regelung nur wenn keine Overrides/Hold aktiv
  if (!overrideActive && now >= holdUntilMs_) {
    // Polarity aus Taupunkten (Außen/Innen) ableiten
    updateFanSignFromDewpoints();

    const float error = (float)vpdFilt_ - vpdTarget;
    float e = 0.0f;
    if      (error >  deadband_) e = error - deadband_;
    else if (error < -deadband_) e = error + deadband_;
    else e = 0.0f;

    // Stellwert berechnen: Δfan = -(sign)*Kp*e
    fanCmd = baseFan - (float)fanSign_ * kpFan_ * e;
    fanCmd = clamp(fanCmd, ps.fanMin, ps.fanMax);
  }

  // 9) Rate-Limiter anwenden + Mindestlüftung erzwingen
  fanCmd = clamp(fanCmd, ps.fanMin, ps.fanMax);
  fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
  lastFanOut_ = fanOut_ = fanCmd;
  fan_->setPercent(fanOut_);

  return true;
}