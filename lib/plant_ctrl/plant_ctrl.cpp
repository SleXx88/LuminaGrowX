// ==============================
// Datei: lib/plant_ctrl/plant_ctrl.cpp
// ==============================

#include "plant_ctrl.h"
#include "../../include/lumina_config.h" // zentrale Defaults/Parameter
#include <math.h>
#include <time.h>

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
  // Zeitpläne aus defaults kopieren
  schedules_[0] = lumina::schedule::SEEDLING;
  schedules_[1] = lumina::schedule::VEGETATIVE;
  schedules_[2] = lumina::schedule::FLOWERING;
}

void PlantCtrl::begin(SHT41Ctrl& sensorIndoor,
                      SHT41Ctrl& sensorOutdoor,
                      GP8211Ctrl& led,
                      FanCtrl& fan,
                      FanCtrl* fan2,
                      StepperCtrl& stepper,
                      ToFCtrl& tof,
                      RTC_Ctrl* rtc,
                      int doorPin) {
  sensorIn_  = &sensorIndoor;
  sensorOut_ = &sensorOutdoor;
  led_       = &led;
  fan_       = &fan;
  fan2_      = fan2;
  step_      = &stepper;
  tof_       = &tof;
  rtc_       = rtc;
  doorPin_   = doorPin;
  if (doorPin_ >= 0) {
    pinMode(doorPin_, INPUT); // LOW = Tür zu (gegen GND)
    lastDoorClosed_ = (digitalRead(doorPin_) == LOW);
  } else {
    lastDoorClosed_ = true; // ohne Sensor: als geschlossen behandeln
  }

  lastMs_    = millis();
  lastFanOut_ = fanOut_ = settings_[idxStage(stage_)][idxMode(mode_)].fanMin;
  // LED auf Basiswert setzen
  ledOut_ = settings_[idxStage(stage_)][idxMode(mode_)].ledPercent;
  if (led_) { led_->setPercent(ledOut_); ledApplied_ = ledOut_; }
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

PhaseModeSettings PlantCtrl::getStageModeSettings(vpd_calc::GrowthStage st, DayMode md) const {
  return settings_[idxStage(st)][idxMode(md)];
}

void PlantCtrl::setSchedule(vpd_calc::GrowthStage st, const LightSchedule& sch) {
  int idx = idxStage(st);
  schedules_[idx] = sch;
  Serial.printf("[CTRL] setSchedule[%d]: On=%02d:%02d Off=%02d:%02d\n", idx, sch.on.hour, sch.on.minute, sch.off.hour, sch.off.minute);
}

LightSchedule PlantCtrl::getSchedule(vpd_calc::GrowthStage st) const {
  return schedules_[idxStage(st)];
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

  // Schimmelprävention / Per-Phase-Parameter übernehmen
  rhCap_[0] = p.seedling_rh_cap;
  rhCap_[1] = p.vegetative_rh_cap;
  rhCap_[2] = p.flowering_rh_cap;
  if (p.use_flowering_late_cap) {
    rhCap_[2] = p.flowering_late_rh_cap;
  }
  rhCapHyst_[0] = p.rh_cap_hyst; // Seedling
  rhCapHyst_[1] = p.rh_cap_hyst; // Vegetative
  rhCapHyst_[2] = p.flowering_rh_cap_hyst; // Flowering hat eigene Hysterese

  // Taupunkt-Mindestabstände
  dpGapMin_[0] = p.seedling_dp_gap_min_C;
  dpGapMin_[1] = p.vegetative_dp_gap_min_C;
  dpGapMin_[2] = p.flowering_dp_gap_min_C;

  minTempPhase_[0] = p.seedling_min_tempC;
  minTempPhase_[1] = p.vegetative_min_tempC;
  minTempPhase_[2] = p.flowering_min_tempC;
  minTempFanMaxScale_ = p.minTemp_fanMax_scale;
  humidityPriorityStrict_ = p.humidity_priority_strict;
  hpCooldownMs_ = p.hp_cooldown_ms;
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
  // Passive Zuluft unten, Abluft oben: Mehr Lüfter -> Innenklima nähert sich Außenklima
  // Außen feuchter (dpOut > dpIn): Mehr Lüfter -> RH steigt -> VPD sinkt -> fanSign +1
  // Außen trockener (dpOut < dpIn): Mehr Lüfter -> RH sinkt -> VPD steigt -> fanSign -1
  float gap = (float)(dpOut_ - dpIn_);
  if (gap > dpHumidHyst_)       fanSign_ = +1; // außen feuchter: Lüfter senkt VPD
  else if (gap < -dpHumidHyst_) fanSign_ = -1; // außen trockener: Lüfter erhöht VPD
  // In der Hysterese-Zone: Vorzeichen beibehalten, um Schwingungen zu vermeiden
}

void PlantCtrl::setDryingMode(bool active) {
  dryingMode_ = active;
}

void PlantCtrl::setGrowActive(bool active) {
  growActive_ = active;
}

bool PlantCtrl::update() {
  if (!sensorIn_ || !sensorOut_ || !fan_ || !led_) return false;

  const uint32_t now = millis();
  const float dt_s = (lastMs_ == 0) ? 0.0f : (float)(now - lastMs_) / 1000.0f;
  lastMs_ = now;

  // 1) Sensoren lesen mit Backoff bei Fehlern
  // Retry-Intervalle für defekte Sensoren
  static uint32_t lastRetryIn = 0;
  static uint32_t lastRetryOut = 0;
  const uint32_t RETRY_MS = 10000; // alle 10s versuchen, falls Sensor weg

  float tC_in = NAN, rh_in = NAN;
  float tC_out = NAN, rh_out = NAN;
  bool okIn = false, okOut = false;

  // Sensor Innen
  if (sensorIn_) {
    // Wenn letzter Lesevorgang OK war, lesen wir normal.
    // Wenn nicht, warten wir RETRY_MS.
    if (!sensorIn_->hasError() || (now - lastRetryIn > RETRY_MS)) {
      if (sensorIn_->read(tC_in, rh_in)) {
        tIn_ = tC_in; rhIn_ = rh_in;
        okIn = true;
        lastRetryIn = now; // Zeitstempel aktualisieren bei Erfolg
      } else {
        lastRetryIn = now; // Fehlerzeitpunkt merken für Backoff
      }
    } else {
      // Im Fehler-Backoff -> letzte bekannte Werte nutzen (oder NAN lassen)
      tIn_ = NAN; rhIn_ = NAN; // Als ungültig markieren
    }
  }

  // Sensor Außen
  if (sensorOut_) {
    if (!sensorOut_->hasError() || (now - lastRetryOut > RETRY_MS)) {
      if (sensorOut_->read(tC_out, rh_out)) {
        tOut_ = tC_out; rhOut_ = rh_out;
        okOut = true;
        lastRetryOut = now;
      } else {
        lastRetryOut = now;
      }
    } else {
      tOut_ = NAN; rhOut_ = NAN;
    }
  }

  // Falls Sensoren fehlen, brechen wir hier nicht hart ab, sondern arbeiten mit Defaultwerten 
  // oder deaktivieren die Logik, aber erlauben dem System weiterzulaufen (WLAN!).
  if (!okIn && !okOut) {
    // Kritischer Fehler: Keine Sensoren -> Keine Regelung möglich.
    // Aber: return true, damit der Rest (WLAN, Webserver) weiterlaufen kann!
    // Alle Aktoren sicherheitshalber aus.
    if (fan_) fan_->setPercent(0.0f);
    if (fan2_) fan2_->setPercent(0.0f);
    if (led_) led_->setPercent(0.0f);
    
    // VPD auf NAN setzen, damit UI Fehler anzeigt
    vpdIn_ = NAN; 
    
    // Wir tun so, als wäre alles "erledigt", damit loop() nicht blockiert.
    return true; 
  }

  // Wenn weder Grow noch Trocknung aktiv: Alle Hardware deaktivieren
  if (!growActive_ && !dryingMode_) {
    vpdIn_ = computeVpd(tIn_, rhIn_);
    dpIn_  = computeDewPoint(tIn_, rhIn_);
    dpOut_ = computeDewPoint(tOut_, rhOut_);
    if (isnan(vpdFilt_)) vpdFilt_ = vpdIn_;
    else if (emaAlpha_ > 0.0f && emaAlpha_ < 1.0f) vpdFilt_ = emaAlpha_ * vpdIn_ + (1.0f - emaAlpha_) * vpdFilt_;
    else vpdFilt_ = vpdIn_;

    // Alle Aktoren ausschalten
    if (ledOut_ != 0.0f || fanOut_ != 0.0f) {
      ledOut_ = 0.0f;
      led_->setPercent(0.0f);
      ledApplied_ = 0.0f;

      fanOut_ = 0.0f;
      fan_->setPercent(0.0f);
      if (fan2_) fan2_->setPercent(0.0f);
      lastFanOut_ = 0.0f;

      Serial.println(F("[CTRL] Kein Grow/Trocknung aktiv - Hardware deaktiviert"));
    }

    // Stepper/LED-Abstandsregelung deaktiviert
    return true;
  }

  // Trocknungsmodus: Einfache Regelung für 45-55% RH, LED 0%, min. 20% Lüfter
  if (dryingMode_) {
    // VPD berechnen (f�r Monitoring)
    vpdIn_ = computeVpd(tIn_, rhIn_);
    dpIn_  = computeDewPoint(tIn_, rhIn_);
    dpOut_ = computeDewPoint(tOut_, rhOut_);
    if (isnan(vpdFilt_)) vpdFilt_ = vpdIn_;
    else if (emaAlpha_ > 0.0f && emaAlpha_ < 1.0f) vpdFilt_ = emaAlpha_ * vpdIn_ + (1.0f - emaAlpha_) * vpdFilt_;
    else vpdFilt_ = vpdIn_;

    // LED immer aus
    ledOut_ = lumina::drying::LED_PERCENT;
    led_->setPercent(ledOut_);
    ledApplied_ = ledOut_;

    // Lüfterregelung für Ziel 45-55% RH
    const float targetRhMin = lumina::drying::TARGET_RH_MIN;
    const float targetRhMax = lumina::drying::TARGET_RH_MAX;
    const float targetRhMid = (targetRhMin + targetRhMax) / 2.0f;
    const float fanMin = lumina::drying::FAN_MIN_PERCENT;
    const float fanMax = lumina::drying::FAN_MAX_PERCENT;

    // Einfache Proportionalregelung: je höher RH über Ziel, desto mehr Lüfter
    float fanCmd = fanMin;
    if (rhIn_ < targetRhMin) {
      // Zu trocken: Minimall�fter
      fanCmd = fanMin;
    } else if (rhIn_ > targetRhMax) {
      // Zu feucht: Maximal
      fanCmd = fanMax;
    } else {
      // Im Bereich: linear interpolieren
      float ratio = (rhIn_ - targetRhMin) / (targetRhMax - targetRhMin);
      fanCmd = fanMin + ratio * (fanMax - fanMin);
    }

    // Rate limiting
    fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
    lastFanOut_ = fanOut_ = fanCmd;
    fan_->setPercent(fanOut_);
    if (fan2_) fan2_->setPercent(fanOut_);

    // Während Trocknung: Stepper/LED-Abstandsregelung DEAKTIVIERT
    // Motor bleibt aus, egal ob Tür offen/zu
    return true;
  }

  // 2) VPD (innen) & Taupunkte
  vpdIn_ = computeVpd(tIn_, rhIn_);
  dpIn_  = computeDewPoint(tIn_, rhIn_);
  dpOut_ = computeDewPoint(tOut_, rhOut_);
  // Robustes EMA: bei Alpha außerhalb [0,1] Filter umgehen
  if (isnan(vpdFilt_)) {
    vpdFilt_ = vpdIn_;
  } else if (emaAlpha_ <= 0.0f || emaAlpha_ >= 1.0f) {
    vpdFilt_ = vpdIn_;
  } else {
    vpdFilt_ = emaAlpha_ * vpdIn_ + (1.0f - emaAlpha_) * vpdFilt_; // EMA-Filter
  }

  // 3) Tür-/Transienten-Erkennung
  detectDoorOrTransient(tIn_, rhIn_, now);

  // 4) Tageszeit + Sunrise/Sunset-Rampen => LED-Ziel + Moduswahl (Day/Night/NightSilent)
  float ledTarget = ledOut_; // Standard: letzten Wert beibehalten (gegen Toggling bei RTC-Fehlern)
  {
    const LightSchedule* sch = &schedules_[idxStage(stage_)];
    if (sch) {
      uint16_t y; uint8_t mo, d, hh, mi, ss;
      bool timeOk = false;

      // 1. Versuch: Systemzeit nutzen (wird von NTP oder RTC synchronisiert)
      time_t now_ts = time(nullptr);
      struct tm tmL;
      localtime_r(&now_ts, &tmL);
      if (tmL.tm_year >= 120) { // Jahr >= 2020 (1900 + 120)
        y = tmL.tm_year + 1900;
        mo = tmL.tm_mon + 1;
        d = tmL.tm_mday;
        hh = tmL.tm_hour;
        mi = tmL.tm_min;
        ss = tmL.tm_sec;
        timeOk = true;
      } 
      // 2. Versuch: Direkte Abfrage der RTC (Fallback bei fehlendem NTP/Systemzeit)
      else if (rtc_ && rtc_->readComponents(y, mo, d, hh, mi, ss)) {
        timeOk = true;
      }

      if (timeOk) {
        auto toSec = [](uint8_t H, uint8_t M, uint8_t S) -> int { return ((int)H * 60 + (int)M) * 60 + (int)S; };
        const int nowSec = toSec(hh, mi, ss);
        const int onSec  = toSec(sch->on.hour, sch->on.minute, 0);
        const int offSec = toSec(sch->off.hour, sch->off.minute, 0);
        bool dayActive;
        if (onSec <= offSec) dayActive = (nowSec >= onSec) && (nowSec < offSec);
        else                 dayActive = (nowSec >= onSec) || (nowSec < offSec);

        DayMode desired = dayActive ? DayMode::Day : (sch->use_night_silent ? DayMode::NightSilent : DayMode::Night);
        if (desired != mode_) {
          setMode(desired);
        }
        const PhaseModeSettings& dayPs = settings_[idxStage(stage_)][idxMode(DayMode::Day)];
        const float dayLedBase = clamp01(dayPs.ledPercent);

        if (!dayActive) {
          ledTarget = 0.0f;
        } else {
          const uint16_t riseMin = sch->sunrise_minutes;
          const uint16_t setMin  = sch->sunset_minutes;
          const int riseSecTotal = (int)riseMin * 60;
          const int setSecTotal  = (int)setMin  * 60;
          int sinceOnSec = (nowSec - onSec); if (sinceOnSec < 0) sinceOnSec += 24*3600;
          int toOffSec   = (offSec - nowSec); if (toOffSec   < 0) toOffSec   += 24*3600;
          bool inRise = (riseSecTotal > 0) && (sinceOnSec >= 0) && (sinceOnSec <  riseSecTotal);
          bool inSet  = (setSecTotal  > 0) && (toOffSec   > 0) && (toOffSec   <= setSecTotal);
          if (inRise) {
            float f = (float)sinceOnSec / (float)riseSecTotal; if (f < 0) f = 0; if (f > 1) f = 1;
            ledTarget = dayLedBase * f;
          } else if (inSet) {
            float f = (float)toOffSec / (float)setSecTotal; if (f < 0) f = 0; if (f > 1) f = 1;
            ledTarget = dayLedBase * f;
          } else {
            ledTarget = dayLedBase;
          }
        }
      }
      // Falls timeOk false ist, bleibt ledTarget = ledOut_, was Sprünge verhindert.
    } else {
      // Ohne Zeitplan: statischer LED-Wert des aktuellen Modus
      ledTarget = clamp01(cur().ledPercent);
    }
  }

  // 4b) Basiswerte für Regelung laden (nach evtl. Mode-Autowechsel)
  PhaseModeSettings& ps = cur();
  const float vpdTarget = midpoint(ps.vpdMin, ps.vpdMax);
  const float baseFan   = ps.fanMin;

  // 5) LED-Reduktion bei ?bertemperatur
  if (tIn_ > maxTemp_) {
    ledTarget = clamp01(ledTarget - ledReducePct_);
  }
  if (led_) {
    // Ziel immer speichern
    ledOut_ = ledTarget;
    // BUGFIX: Beim Wechsel auf 0% (Sonnenuntergang) darf keine Totzone verhindern,
    //         dass 0% gesetzt wird. Sonst bleibt die LED z.B. bei 0.3% -> effektiv 1% stehen.
    const bool forceOff = (!isnan(ledApplied_) && ledTarget <= 0.0f && ledApplied_ > 0.0f); // von >0 auf 0
    // Nur schreiben, wenn sich der Wert deutlich geändert hat ODER wir 0% erzwingen müssen
    if (isnan(ledApplied_) || forceOff || fabsf(ledTarget - ledApplied_) >= 0.25f) { // Schwelle reduziert auf 0.25%
      ledApplied_ = ledTarget;
      led_->setPercent(ledApplied_);
    }
  }

  // Temperatur-Leitplanken: effektives FanMax bei Untertemperatur begrenzen
  const int si = idxStage(stage_);
  float fanMaxEff = ps.fanMax;
  const float minTempPhase = minTempPhase_[si];
  if (tIn_ < minTempPhase) {
    fanMaxEff = fmaxf(ps.fanMin, ps.fanMax * minTempFanMaxScale_);
  } else if (tIn_ >= (minTempPhase + 0.5f)) {
    fanMaxEff = ps.fanMax;
  }
  if (fanMaxEff < ps.fanMin) fanMaxEff = ps.fanMin;
  if (fanMaxEff > 100.0f)    fanMaxEff = 100.0f;

  // Priorität 1: Schimmelprävention (Humidity-Priority)
  const float cap   = rhCap_[si];
  const float hyst  = rhCapHyst_[si];
  const float band  = fmaxf(0.0f, hyst * 0.5f); // symmetrisches Hysterese-Band
  const float dpGapReq = dpGapMin_[si];
  const bool dpTooNear = ((float)(tIn_ - dpIn_) < dpGapReq);

  if (humidityPriorityStrict_) {
    // Eintritt: sofort bei Taupunkt-Gefahr; sonst RH = cap + band und kein aktiver Cooldown
    if (!humidityPriorityActive_) {
      if (dpTooNear) {
        humidityPriorityActive_ = true;
      } else {
        if ((float)rhIn_ >= (cap + band) && millis() >= hpCooldownUntilMs_) {
          humidityPriorityActive_ = true;
        }
      }
    }
    // Austritt: erst wenn RH = cap - band und ΔTdp ausreichend
    if (humidityPriorityActive_) {
      if (((float)rhIn_ <= (cap - band)) && !dpTooNear) {
        humidityPriorityActive_ = false;
        // Cooldown starten (Taupunkt-Gefahr darf Cooldown übersteuern)
        hpCooldownUntilMs_ = millis() + hpCooldownMs_;
      }
    }
  } else {
    humidityPriorityActive_ = false;
  }
  // 6) Außen feuchter -> Lüfter blockieren
  // Outside humidity block: when the outdoor dew point is higher than the indoor
  // dew point by more than dpHumidHyst_, we should avoid pulling moist air into
  // the box.  Previously this block was only applied if no humidity priority
  // override was active.  In practice, however, external humidity can spike
  // quickly (e.g. someone breathing on the sensor).  To ensure that humid air
  // isn�t driven into the enclosure we now always evaluate this block before
  // applying any other overrides.  If the condition is met we ramp the fan
  // toward a minimum using the configured rate limit and return immediately.  This
  // allows the box to retain its drier air even when indoor RH is high.
  // FIX: Heat-Emergency BEFORE humid-block - bei Hitze trotzdem kuehlen
  const bool heatEmergency = (tIn_ > (maxTemp_ + 0.5f));
  // FIX: Moisture-Emergency - bei extremer Feuchte (z.B. >85%) Blockade ignorieren, um "Sumpf" bei Regen zu vermeiden
  const bool moisturePanic = (rhIn_ >= rhHighThr_);

  if (!heatEmergency && !moisturePanic && blockOutsideHumid_ && (!isnan(dpIn_) && !isnan(dpOut_)) && ((float)dpOut_ >= (float)dpIn_ + dpHumidHyst_)) {
    // Mindest-Luftwechsel beibehalten (15%) für CO2-Nachschub, auch wenn außen feuchter
    // Kleine Box braucht kontinuierlichen Luftaustausch für Pflanzenstoffwechsel
    float fanCmd = 15.0f; // war 0.0f - vollständige Blockade riskiert CO2-Mangel
    fanCmd = limitRate(fanCmd, lastFanOut_, dt_s);
    lastFanOut_ = fanOut_ = fanCmd;
    fan_->setPercent(fanOut_);
    if (fan2_) fan2_->setPercent(fanOut_);
    // Decay the integrator a bit so that it doesn't wind up during the block.
    iTermFan_ *= 0.98f;
    // Still tick the distance controller while the fan is blocked
    distanceTick_(now);
    return true;
  }

  // 7) Overrides (Feuchte/Taupunkt/Temperatur)
  bool overrideActive = humidityPriorityActive_;
  const double dewGapIn = tIn_ - dpIn_;

  float fanCmd = baseFan + iTermFan_;
  if (humidityPriorityActive_) {
    //
    // When humidity priority is active we previously forced the fan directly to
    // its maximum (fanMaxEff).  This leads to large swings between 20% and
    // 70% in flowering because the fan saturates at 70% whenever the dewpoint
    // gap or RH thresholds are crossed and then drops back down as soon as the
    // thresholds clear.  To obtain smoother behaviour we scale the fan speed
    // based on how far the measured relative humidity and dewpoint gap exceed
    // their critical limits.  This allows the controller to ramp the fan
    // proportionally instead of immediately jumping to the hard maximum.

    // overshootRH: positive amount by which the current indoor RH exceeds the
    // mould-prevention cap for this phase.  Values below zero are ignored.
    float overshootRH = (float)rhIn_ - cap;
    if (overshootRH < 0.0f) overshootRH = 0.0f;

    // overshootDP: positive amount by which the dewpoint gap (Temp���Taupunkt)
    // falls below the required minimum gap.  Values below zero are ignored.
    float overshootDP = dpGapReq - (float)(tIn_ - dpIn_);
    if (overshootDP < 0.0f) overshootDP = 0.0f;

    // Tuneable weights: how many percent of additional fan speed should be
    // commanded per percentage point of RH overshoot and per degree of dewpoint
    // gap deficit.  These values have been chosen empirically and can be
    // exposed via a setter if you wish to fine-tune them later.
    const float wRH = 1.2f; // % fan increase per 1%RH overshoot
    const float wDP = 2.0f; // % fan increase per 1°C dewpoint gap deficit

    // Compute the additional fan demand.  Start from the minimum fan for this
    // phase and add scaled overshoot contributions.  Note that this may
    // overshoot the currently commanded fan; that is intentional.
    float targetFan = ps.fanMin + (wRH * overshootRH) + (wDP * overshootDP);

    // Determine the absolute maximum fan that may be commanded.  In NightSilent
    // mode the configuration may allow a +20% boost above fanMaxEff to rescue
    // the climate even if it means briefly being louder.  We also ensure
    // maxLimit never drops below fanMin to avoid pathological cases.
    float maxLimit = fanMaxEff;
    if (mode_ == DayMode::NightSilent && allowSilentOv_) {
      maxLimit = fminf(100.0f, maxLimit + 20.0f);
    }
    if (maxLimit < ps.fanMin) maxLimit = ps.fanMin;

    // Clamp the target fan demand into the allowed range.  This prevents the
    // demand from exceeding the configured maximum or falling below fanMin.
    float scaledFan = clamp(targetFan, ps.fanMin, maxLimit);

    // To avoid introducing high-frequency oscillations, do not immediately
    // reduce the fan if the newly computed scaled value is below the current
    // output; instead hold the current output until RH/DP overshoot rises
    // again.  This introduces a small amount of hysteresis on the
    // humidity-priority branch.
    if (scaledFan < fanOut_) {
      fanCmd = fanOut_;
    } else {
      fanCmd = scaledFan;
    }

    // While humidity override is active we still slowly bleed down the
    // integrator to prevent windup.
    iTermFan_ *= 0.995f;
  }
  if (!overrideActive && (rhIn_ >= rhHighThr_ || dewGapIn <= dewGapMinC_)) {
    float maxLimit = fanMaxEff;
    if (mode_ == DayMode::NightSilent && allowSilentOv_) {
      maxLimit = fminf(100.0f, maxLimit + 20.0f);
    }
    fanCmd = clamp(fmaxf(ps.fanMin + humidBoost_, fanOut_), ps.fanMin, maxLimit);
    overrideActive = true;
  }
  if (!overrideActive && (tIn_ >= tempHighFanC_ || heatEmergency)) {
    fanCmd = clamp(fmaxf(ps.fanMin + tempHighFanBoost_, fanOut_), ps.fanMin, fanMaxEff);
    overrideActive = true;
  }

  // 8) Holds (Tür/Modus)
  if (!overrideActive && now < holdUntilMs_) {
    fanCmd = clamp(baseFan + iTermFan_, ps.fanMin, fanMaxEff);
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

    // *** KORRIGIERTE PI-FORMEL ***
    fanCmd = baseFan + iTermFan_ + (float)fanSign_ * kpFan_ * e;

    // Integrator-Update mit Anti-Windup (Clamping)
    const float iDelta = ((float)fanSign_) * kiFan_ * e * dt_s;
    float preClamp = fanCmd + iDelta;
    float clamped  = clamp(preClamp, ps.fanMin, fanMaxEff);
    if (fabsf(preClamp - clamped) < 1e-3f) {
      iTermFan_ += iDelta; // Integrator nur übernehmen, wenn keine Sättigung vorliegt
      if (iTermFan_ >  20.0f) iTermFan_ =  20.0f;
      if (iTermFan_ < -20.0f) iTermFan_ = -20.0f;
    } else {
      iTermFan_ *= 0.98f; // stärkere Reduktion während Sättigung
    }
    fanCmd = clamped;
  } else {
    // Integrator bei Hold/Override leicht entspannen
    iTermFan_ *= 0.995f;
  }

      if (fanMaxEff < ps.fanMin) fanMaxEff = ps.fanMin;
  fanCmd = clamp(fanCmd, ps.fanMin, fanMaxEff);
  fanCmd = limitRate(fanCmd, lastFanOut_, dt_s); // Sanfter Übergang
  // Kleinsignal-Abschaltung: sehr kleine Werte auf 0 setzen, um minPercent-Mapping des Lüfters nicht zu triggern
  const float offEps = 0.5f; // %
  if (fanCmd > 0.0f && fanCmd < offEps) {
    fanCmd = 0.0f;
  }
  lastFanOut_ = fanOut_ = fanCmd;
  fan_->setPercent(fanOut_);
  if (fan2_) fan2_->setPercent(fanOut_);

  // Diagnoseausgabe alle 3s (intern)
  {
    static uint32_t dbgLastMs = 0;
    if (now - dbgLastMs >= 3000) {
      dbgLastMs = now;
      const int si_dbg = idxStage(stage_);
      const float cap = rhCap_[si_dbg];
      const float hyst = rhCapHyst_[si_dbg];
      const float dpGapReq = dpGapMin_[si_dbg];
      float dpGap = (float)(tIn_ - dpIn_);
      float dpOutGap = (float)(dpOut_ - dpIn_);
      Serial.printf("[CTRL] HP:%d RH:%.1f cap:%.1f(%.1f) dTdp:%.1f req:%.1f dpOut-in:%.1f fanMaxEff:%.1f cmd:%.1f hold:%d sign:%d\n",
                    (int)humidityPriorityActive_, (float)rhIn_, cap, hyst,
                    dpGap, dpGapReq, dpOutGap, fanMaxEff, fanOut_, (now < holdUntilMs_), (int)fanSign_);
    }
  }

  // Distanzverwaltung (Tür/Schedule/Eventfenster + fließende P-Regelung)
  distanceTick_(now);

  return true;
}

// Türlogik: LOW = Tür zu (gegen GND)
bool PlantCtrl::isDoorClosed_() {
  if (doorPin_ < 0) return true;
  return digitalRead(doorPin_) == LOW;
}

static inline unsigned long long makeTokenYMDHM(int y, int m, int d, int hh, int mm) {
  unsigned long long Y = (unsigned long long)y;
  unsigned long long M = (unsigned long long)m;
  unsigned long long D = (unsigned long long)d;
  unsigned long long H = (unsigned long long)hh;
  unsigned long long Min = (unsigned long long)mm;
  return (((Y*100ULL + M)*100ULL + D)*100ULL + H)*100ULL + Min;
}

bool PlantCtrl::allowDownAdjustNow_(uint32_t now) {
  bool closed = isDoorClosed_();
  if (closed != lastDoorClosed_) {
    // Entprellung: Nur reagieren, wenn der Zustand sich geändert hat UND 
    // seit dem letzten Wechsel mindestens 50ms vergangen sind.
    if (now - doorLastChangeMs_ > 50) {
      lastDoorClosed_ = closed;
      doorLastChangeMs_ = now;
      
      if (closed) {
        Serial.println(F("[DOOR] geschlossen (stabil)"));
        if (lumina::plant::AUTO_APPROACH_DOOR) return true; 
      } else {
        Serial.println(F("[DOOR] offen"));
        adjustActive_ = false; // Sicherheits-Abbruch bei Öffnen
      }
    }
  }

  if (rtc_) {
    uint16_t y; uint8_t mo, d, hh, mi, ss;
    if (rtc_->readComponents(y, mo, d, hh, mi, ss)) {
      // 1) Zeitplan-Events (Sunrise/Sunset)
      const LightSchedule* sch = &schedules_[idxStage(stage_)];
      if (sch) {
        unsigned long long token = makeTokenYMDHM(y, mo, d, hh, mi);
        if (hh == sch->on.hour  && mi == sch->on.minute  && token != lastSunriseToken_) { 
          lastSunriseToken_ = token; 
          if (lumina::plant::AUTO_APPROACH_SUNRISE) {
            Serial.println(F("[SCHEDULE] Sunrise smooth adjust")); 
            return true; 
          }
        }
        if (hh == sch->off.hour && mi == sch->off.minute && token != lastSunsetToken_)  { 
          lastSunsetToken_  = token; 
          if (lumina::plant::AUTO_APPROACH_SUNSET) {
            Serial.println(F("[SCHEDULE] Sunset smooth adjust"));  
            return true; 
          }
        }
      }

      // 2) Periodische Annäherung (z.B. alle 4h)
      if (hh != lastApproachHour_) {
        bool isTargetHour = false;
        for (int i = 0; i < lumina::plant::APPROACH_HOURS_COUNT; i++) {
          if (hh == lumina::plant::APPROACH_HOURS[i]) {
            isTargetHour = true;
            break;
          }
        }
        
        lastApproachHour_ = hh;
        if (isTargetHour && lumina::plant::AUTO_APPROACH_PERIODIC) {
          Serial.printf("[SCHEDULE] Periodic smooth adjust at %02d:00\n", hh);
          return true;
        }
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

  // Tür-Status immer prüfen für Edge-Detection (Sunrise/Sunset/Door)
  bool canAdjust = allowDownAdjustNow_(now);

  // Tür offen? Keine Bewegung zulassen
  if (!isDoorClosed_()) { 
    if (adjustActive_) {
      step_->stop();
      adjustActive_ = false;
    }
    return; 
  }

  // ToF lesen
  uint32_t interval = adjustActive_ ? lumina::plant::TOF_READ_INTERVAL_MS : 10000;
  if (now - lastTofReadMs_ >= interval) {
    lastTofReadMs_ = now;
    int v = tof_->readAvgMm(lumina::plant::TOF_AVG_SAMPLES);
    if (v >= 0) {
      lastTofMm_ = v;
    } else if (adjustActive_) {
      // Wenn wir aktiv regeln und der Sensor ausfaellt oder Out-of-Range meldet -> Abbruch
      Serial.println(F("[CTRL] ERROR: Ungueltiger ToF-Abstand waehrend Fahrt. Abgebrochen!"));
      step_->stop();
      adjustActive_ = false;
    }
  }
  if (lastTofMm_ < 0) return;

  const float offset = (float)lumina::plant::TOF_LED_OFFSET_MM;
  const float ledPlantMm = (float)lastTofMm_ + offset;
  const float target = targetDistanceMm_();
  const float hyst = lumina::plant::ADJUST_HYST_MM;
  const float err = ledPlantMm - target;

  // Ereignisfenster aktivieren
  if (canAdjust) { 
    adjustActive_ = true; 
    adjustUntilMs_ = now + lumina::plant::AUTO_APPROACH_TIMEOUT_MS; 
    lastTofReadMs_ = 0; // Mess-Intervall zurücksetzen -> sofortige Messung erzwingen
    Serial.printf("[CTRL] Starte sanfte Abstands-Anpassung. Dist: %.1f mm, Ziel: %.1f mm\n", ledPlantMm, target);
  }

  if (!adjustActive_) return;

  // Timeout-Schutz oder Ziel erreicht
  if (now > adjustUntilMs_) { 
    step_->stop();
    adjustActive_ = false; 
    return; 
  }

  // Physikalische Grenzen prüfen: Nur stoppen, wenn wir in die Richtung der Grenze fahren wollen
  float curPos = step_->getPositionMm();
  if (!step_->status().isMoving && fabsf(err) > hyst) {
    bool wantUp = (err < 0);   // Zu nah -> muss nach oben
    bool wantDown = (err > 0); // Zu weit weg -> muss nach unten
    
    if ((wantUp && curPos <= 0.5f) || (wantDown && curPos >= (step_->getMaxTravelMm() - 0.5f))) {
      Serial.println(F("[CTRL] Physikalische Grenze erreicht - Keine Fahrt in diese Richtung möglich."));
      adjustActive_ = false;
      return;
    }
  }

  // Ziel erreicht?
  if (fabsf(err) <= hyst) {
    step_->stop();
    adjustActive_ = false;
    Serial.printf("[CTRL] Anpassung beendet. Abstand: %.1f mm\n", ledPlantMm);
    return;
  }

  // Proportionale Geschwindigkeitsregelung (wie in runStartupApproachBlocking)
  float targetHz = fabsf(err) * lumina::plant::APPROACH_P_FACTOR;
  targetHz = clamp(targetHz, lumina::plant::APPROACH_MIN_HZ, lumina::plant::APPROACH_MAX_HZ);
  
  // Fahrtrichtung bestimmen und Jog starten/anpassen
  if (err > 0) {
    // Zu weit weg -> nach unten
    if (step_->status().isMoving) {
      step_->setTargetSpeedHz(targetHz);
    } else {
      step_->jogDown(lumina::plant::SPEED_LEVEL);
      step_->setTargetSpeedHz(targetHz);
    }
  } else {
    // Zu nah -> nach oben
    if (step_->status().isMoving) {
      step_->setTargetSpeedHz(targetHz);
    } else {
      step_->jogUp(lumina::plant::SPEED_LEVEL);
      step_->setTargetSpeedHz(targetHz);
    }
  }
}

// Dynamische, blockierende Annäherung beim Setup
void PlantCtrl::runStartupApproachBlocking() {
  if (!step_ || !tof_) { initialApproachDone_ = true; return; }

  const float target = targetDistanceMm_();
  const float offset = (float)lumina::plant::TOF_LED_OFFSET_MM;
  const float hyst   = lumina::plant::ADJUST_HYST_MM;
  
  Serial.printf("[STARTUP] Starte sanfte Annaeherung. Ziel: %.1f mm (ToF-Offset: %.1f)\n", target, offset);
  
  // Kontinuierliche Fahrt nach unten starten (Down ist logical +1)
  step_->jogDown(lumina::plant::SPEED_LEVEL);
  
  uint32_t lastRead = 0;
  uint32_t lastPrint = 0;
  bool reached = false;

  for (;;) {
    uint32_t now = millis();
    step_->tick(); // WICHTIG: Stepper-Logik am Laufen halten

    if (now - lastRead >= lumina::plant::TOF_READ_INTERVAL_MS) {
      lastRead = now;
      int v = tof_->readAvgMm(lumina::plant::TOF_AVG_SAMPLES);
      if (v >= 0) {
        lastTofMm_ = v;
        float ledPlantMm = (float)lastTofMm_ + offset;
        float err = ledPlantMm - target;

        if (err <= hyst) {
          step_->stop();
          reached = true;
          Serial.printf("[STARTUP] Ziel erreicht! Abstand: %.1f mm (Target: %.1f)\n", ledPlantMm, target);
        } else {
          // Proportionale Geschwindigkeitsregelung
          float targetHz = err * lumina::plant::APPROACH_P_FACTOR;
          targetHz = clamp(targetHz, lumina::plant::APPROACH_MIN_HZ, lumina::plant::APPROACH_MAX_HZ);
          step_->setTargetSpeedHz(targetHz);

          if (now - lastPrint >= 500) {
            lastPrint = now;
            Serial.printf("[STARTUP] Dist: %.1f mm, Err: %.1f mm, Speed: %.0f Hz, Pos: %.1f mm\n", 
                          ledPlantMm, err, targetHz, step_->getPositionMm());
          }
        }
      } else {
        // Fehler oder Out-of-Range während der blockierenden Annäherung
        Serial.println(F("[STARTUP] ERROR: Ungueltiger ToF-Abstand! Fahrt zur Pflanze unterbrochen."));
        step_->stop();
        break;
      }
    }

    if (reached) {
      if (!step_->status().isMoving) break;
    }
    
    // Timeout-Schutz: Wenn nach 40s nichts passiert, abbrechen
    if (now > 40000 && !reached && lastTofMm_ < 0) {
       Serial.println(F("[STARTUP] ERROR: Kein ToF-Signal nach 40s (Timeout)!"));
       step_->stop();
       break;
    }

    delay(5);
  }

  initialApproachDone_ = true;
  // Nach der Annäherung noch kurz ticken lassen, damit die Auto-Disable Logik greifen kann
  uint32_t startWait = millis();
  while (millis() - startWait < 600) {
    step_->tick();
    delay(5);
  }
}
