#include "stepper_ctrl.h"

// DIAG-Flag (Fehlerindikator, nicht Stall)
volatile bool StepperCtrl::diagTriggered_ = false;
void IRAM_ATTR StepperCtrl::onDiagRiseISR_() { StepperCtrl::diagTriggered_ = true; }

StepperCtrl::StepperCtrl(const StepperPins &pins,
                         const StepperKinematics &kin,
                         const StepperLimits &lim,
                         const TMC2209Config &tmc,
                         const HomingParams &hp,
                         const SpeedTable &spd)
    : p_(pins), k_(kin), l_(lim), c_(tmc), h_(hp), s_(spd),
      driver_(p_.uart, c_.r_sense, c_.driver_addr)
{
}

void StepperCtrl::recomputeSoftLimits_()
{
  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1; // +1: forward=UP
  const long bottomSigned = mmToSteps(l_.max_travel_mm) * (-axisSign);
  const long top = 0; // oberer Arbeitsrand = 0 mm

  softMinSteps_ = (bottomSigned < top) ? bottomSigned : top;
  softMaxSteps_ = (bottomSigned > top) ? bottomSigned : top;
}

void StepperCtrl::begin()
{
  // IOs
  pinMode(p_.pin_step, OUTPUT);
  pinMode(p_.pin_dir, OUTPUT);
  pinMode(p_.pin_en, OUTPUT);
  pinMode(p_.pin_diag, INPUT_PULLUP);

  // UART zum TMC2209
  p_.uart->begin(p_.uart_baud, SERIAL_8N1, p_.uart_rx, p_.uart_tx);

  // TMC-Treiber
  setupDriver_();

  // FastAccelStepper Engine initialisieren und Stepper verbinden
  engine_.init(); // <— Werttyp, kein Pointer
  fas_ = engine_.stepperConnectToPin(p_.pin_step);
  if (!fas_)
  {
    if (debug_)
      Serial.println("[INIT] FastAccelStepper connectToPin FAILED");
    return;
  }

  fas_->setDirectionPin(p_.pin_dir);
  fas_->setEnablePin(p_.pin_en);
  fas_->setAutoEnable(true);
  fas_->setDelayToEnable(2);
  fas_->setDelayToDisable(500);
  fas_->setSpeedInHz(2500);
  fas_->setAcceleration(5000);
  fas_->setCurrentPosition(0);

  // Softlimits passend zum Achsensinn setzen (oben = 0 mm)
  recomputeSoftLimits_();

  homingFinished_ = false;
  attachInterrupt(digitalPinToInterrupt(p_.pin_diag), StepperCtrl::onDiagRiseISR_, RISING);

  if (debug_)
  {
    Serial.printf(
        "[INIT] microsteps=%u lead=%.2fmm Irun=%umA Ihold=%umA max=%.1fmm (Soft: %ld..%ld)\n",
        k_.microsteps, k_.lead_mm, c_.irun_mA, c_.ihold_mA, l_.max_travel_mm,
        softMinSteps_, softMaxSteps_);
  }
}

void StepperCtrl::setupDriver_()
{
  driver_.begin();
  driver_.toff(5);
  driver_.blank_time(24);
  driver_.microsteps(k_.microsteps);
  driver_.pwm_autoscale(true);
  driver_.pwm_autograd(true);

  driver_.en_spreadCycle(false); // StealthChop
  driver_.TPWMTHRS(0);
  driver_.TCOOLTHRS(0xFFFFF);
  driver_.SGTHRS(c_.sg_thrs_diag);

  // Einfache CoolStep-Standardwerte
  driver_.semin(5);
  driver_.semax(2);
  driver_.sedn(0b01);
  driver_.seimin(false);

  setCurrents(c_.irun_mA, c_.ihold_mA);

  digitalWrite(p_.pin_en, LOW); // Treiber aktivieren
}

void StepperCtrl::tick()
{
  // Schritte müssen nicht manuell erzeugt werden; FastAccelStepper läuft über ISR.
  switch (mode_)
  {
  case Mode::CONTINUOUS:
    // läuft autonom
    break;
  case Mode::GOTO:
    if (fas_ && !fas_->isRunning())
    {
      mode_ = Mode::IDLE;
      lastOpDone_ = true;
      if (debug_)
      {
        float pos = getPositionMm();
        Serial.printf("[MOVE] Ziel erreicht, pos=%.2fmm\n", pos);
      }
    }
    break;
  case Mode::HOMING:
    updateHoming_();
    break;
  case Mode::IDLE:
  default:
    break;
  }

  if (mode_ != Mode::HOMING)
    enforceSoftLimits_();

  // Bewegungsprotokolle mit Drosselung
  if (debug_)
  {
    uint32_t now = millis();
    if (now - lastDebugPosMs_ >= debugMoveLogIntervalMs_)
    {
      lastDebugPosMs_ = now;
      if (mode_ == Mode::GOTO)
      {
        float pos = getPositionMm();
        Serial.printf("[MOVE] pos=%.2fmm running=%d\n", pos, fas_ ? fas_->isRunning() : 0);
      }
      else if (mode_ == Mode::CONTINUOUS)
      {
        float pos = getPositionMm();
        Serial.printf("[JOG] pos=%.2fmm running=%d\n", pos, fas_ ? fas_->isRunning() : 0);
      }
      else if (mode_ == Mode::HOMING)
      {
        float pos = getPositionMm();
        Serial.printf("[HOME] pos=%.2fmm sg(avg)=%.1f base=%.1f\n", pos, sgAvg_, sgBaseline_);
      }
    }
  }
}

/* ---------------- Homing ---------------- */

bool StepperCtrl::startHoming()
{
  if (homingFinished_)
    return true; // bereits abgeschlossen
  if (mode_ == Mode::HOMING)
    return false; // läuft

  // 1) PRE_BACKOFF: zuerst 3 mm von der Referenz weg (Gegenrichtung zum Homing)
  homingBackoffActive_ = false;
  hState_ = HomeState::PRE_BACKOFF;
  mode_ = Mode::HOMING;
  lastOpDone_ = false;
  // negative mmLogical => Gegenrichtung (nach unten) bei homingDirLogical_ = +1
  homeMoveRelativeMM_(-homingDirLogical_ * h_.backoff_mm, h_.speed_slow_hz);
  if (debug_)
    Serial.println("[HOME] Pre-backoff start");

  return false;
}

void StepperCtrl::resetHoming()
{
  homingFinished_ = false;
  hState_ = HomeState::IDLE;
  mode_ = Mode::IDLE;
  lastOpDone_ = false;
  sgCalibMs_ = 0;
  sgSamples_ = 0;
  sgBaseline_ = 0.0f;
  sgAvg_ = 0.0f;
  diagTriggered_ = false;
  homingBackoffActive_ = false;
  if (debug_)
    Serial.println("[HOME] Reset");
}

std::pair<float, bool> StepperCtrl::goTop(uint8_t speedLevel)
{
  return goToEnd(+1, speedLevel);
}
std::pair<float, bool> StepperCtrl::goBottom(uint8_t speedLevel)
{
  return goToEnd(-1, speedLevel);
}
std::pair<float, bool> StepperCtrl::moveUp(float mm, uint8_t speedLevel)
{
  return moveBy(+1, mm, speedLevel);
}
std::pair<float, bool> StepperCtrl::moveDown(float mm, uint8_t speedLevel)
{
  return moveBy(-1, mm, speedLevel);
}

/* ---------------- Bewegungen ---------------- */

std::pair<float, bool> StepperCtrl::goToEnd(int logicalDir, uint8_t speedLevel)
{
  const long target = (logicalDir >= 0) ? softMaxSteps_ : softMinSteps_;
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(stepsToMm(target), s_.hz[lvl]);
  if (debug_)
    Serial.printf("[CMD] goToEnd(%s) lvl=%u\n", (logicalDir >= 0) ? "TOP" : "BOTTOM", lvl);
  return {getPositionMm(), (mode_ == Mode::IDLE)};
}

std::pair<float, bool> StepperCtrl::moveBy(int logicalDir, float mm, uint8_t speedLevel)
{
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  // Koordinatenlogik: unten = +mm, oben = -mm
  const float mmLogical = (logicalDir >= 0) ? -fabsf(mm) : +fabsf(mm);
  moveRelativeLogicalMM_(mmLogical, s_.hz[lvl]);
  if (debug_)
    Serial.printf("[CMD] moveBy(mm=%.2f, %s) lvl=%u\n",
                  fabsf(mm), (logicalDir >= 0) ? "UP" : "DOWN", lvl);
  return {getPositionMm(), (mode_ == Mode::IDLE)};
}

std::pair<float, bool> StepperCtrl::moveTo(float pos_mm, uint8_t speedLevel)
{
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(pos_mm, s_.hz[lvl]);
  if (debug_)
    Serial.printf("[CMD] moveTo(%.2fmm) lvl=%u\n", pos_mm, lvl);
  return {getPositionMm(), (mode_ == Mode::IDLE)};
}

void StepperCtrl::jogUp(uint8_t speedLevel)
{
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  startContinuousLogical_(s_.hz[lvl], +1);
  if (debug_)
    Serial.printf("[CMD] jogUp lvl=%u\n", lvl);
}
void StepperCtrl::jogDown(uint8_t speedLevel)
{
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  startContinuousLogical_(s_.hz[lvl], -1);
  if (debug_)
    Serial.printf("[CMD] jogDown lvl=%u\n", lvl);
}
void StepperCtrl::jogStop()
{
  stop();
  if (debug_)
    Serial.println("[CMD] jogStop");
}

void StepperCtrl::stop()
{
  if (fas_)
  {
    fas_->stopMove();
  }
  mode_ = Mode::IDLE;
  lastOpDone_ = true;
}

StepperStatus StepperCtrl::status() const
{
  StepperStatus s;
  s.position_mm = getPositionMm();
  s.diag = digitalRead(p_.pin_diag);
  s.isHoming = (mode_ == Mode::HOMING);
  s.isMoving = (fas_ && fas_->isRunning()) || (mode_ == Mode::HOMING);
  s.lastOpDone = lastOpDone_;
  return s;
}

float StepperCtrl::getPositionMm() const
{
  long pos = fas_ ? fas_->getCurrentPosition() : 0;
  return stepsToMm(pos);
}

void StepperCtrl::enableDebug(bool on) { debug_ = on; }
void StepperCtrl::setDebugMoveLogInterval(uint16_t ms) { debugMoveLogIntervalMs_ = ms; }

void StepperCtrl::setCurrents(uint16_t irun_mA, uint16_t ihold_mA)
{
  c_.irun_mA = irun_mA;
  c_.ihold_mA = ihold_mA;
  float mult = (c_.irun_mA == 0) ? 0.0f : (float)c_.ihold_mA / (float)c_.irun_mA;
  if (mult < 0.0f)
    mult = 0.0f;
  driver_.rms_current(c_.irun_mA, mult);
  driver_.hold_multiplier(mult);
}

void StepperCtrl::setMicrosteps(uint8_t microsteps)
{
  k_.microsteps = microsteps;
  driver_.microsteps(k_.microsteps);
}

void StepperCtrl::setAxisUpDir(int axis_up_dir)
{
  AXIS_UP_DIR_ = (axis_up_dir >= 0) ? +1 : -1;
  recomputeSoftLimits_();
  // aktuelle Position in Softlimits halten
  if (fas_)
  {
    long pos = fas_->getCurrentPosition();
    if (pos < softMinSteps_)
      fas_->forceStopAndNewPosition(softMinSteps_);
    else if (pos > softMaxSteps_)
      fas_->forceStopAndNewPosition(softMaxSteps_);
  }
}

void StepperCtrl::setMaxTravelMm(float max_travel_mm)
{
  l_.max_travel_mm = fmaxf(0.0f, max_travel_mm);
  recomputeSoftLimits_();
}

/* ---------- Interna ---------- */
inline long StepperCtrl::mmToSteps(float mm) const
{
  const float steps_per_mm = (k_.steps_per_rev * k_.microsteps) / k_.lead_mm;
  return lroundf(mm * steps_per_mm);
}
inline float StepperCtrl::stepsToMm(long st) const
{
  const float steps_per_mm = (k_.steps_per_rev * k_.microsteps) / k_.lead_mm;
  return (float)st / steps_per_mm;
}
inline int StepperCtrl::logicalToMotorDir(int logicalDir) const
{
  return (logicalDir >= 0) ? AXIS_UP_DIR_ : -AXIS_UP_DIR_;
}
inline float StepperCtrl::stepsToLogicalMm(long deltaSteps) const
{
  return stepsToMm(deltaSteps) * (float)((AXIS_UP_DIR_ >= 0) ? +1 : -1);
}
inline float StepperCtrl::ema_update(float prev, float sample, float alpha)
{
  return alpha * sample + (1.0f - alpha) * prev;
}

void StepperCtrl::beginHomePhase_(HomeState next, float speedHz, int logicalDir)
{
  hState_ = next;
  homePhaseMs_ = millis();
  sgCalibMs_ = homePhaseMs_;
  sgSamples_ = 0;
  sgBaseline_ = 0.0f;
  sgAvg_ = 0.0f;
  sgLastPoll_ = 0;

  if (next == HomeState::FAST)
  {
    // Referenz für Mindestweg
    homingStartSteps_ = fas_ ? fas_->getCurrentPosition() : 0;

    // *** FAST-Phase als ENDLICHEN Move planen – OHNE mode_ zu ändern! ***
    const float target_mm = (logicalDir >= 0) ? l_.max_travel_mm : 0.0f;

    if (fas_)
    {
      float v = fabsf(speedHz);
      fas_->setSpeedInHz(v);
      fas_->setAcceleration(v * 2.0f);
      long tgt = mmToSteps(target_mm) * ((AXIS_UP_DIR_ >= 0) ? +1 : -1);
      fas_->moveTo(tgt);
    }
  }

  // Für andere Phasen (PRE_BACKOFF, BACKOFF, DONE) wird hier keine Bewegung geplant
  diagTriggered_ = false;

  if (debug_)
  {
    const char *name = (next == HomeState::FAST) ? "FAST" : (next == HomeState::BACKOFF)   ? "BACKOFF"
                                                        : (next == HomeState::PRE_BACKOFF) ? "PRE_BACKOFF"
                                                        : (next == HomeState::DONE)        ? "DONE"
                                                                                           : "?";
    Serial.printf("[HOME] Phase %s start @ %.0f Hz, logDir=%d\n", name, speedHz, logicalDir);
  }
}

void StepperCtrl::updateHoming_()
{
  if (!fas_)
    return;
  uint32_t now = millis();
  bool sgWindowOver = (now - homePhaseMs_) > h_.sg_ignore_ms;

  // PRE_BACKOFF: warte auf Ende der Rücknahme, dann FAST-Phase einleiten
  if (hState_ == HomeState::PRE_BACKOFF)
  {
    if (!fas_->isRunning())
    {
      // Jetzt mit voller Geschwindigkeit in Homing-Richtung (nach oben)
      beginHomePhase_(HomeState::FAST, h_.speed_fast_hz, homingDirLogical_);
      if (debug_)
        Serial.println("[HOME] Pre-backoff done -> FAST");
    }
    return;
  }

  // SG_RESULT pollen
  if (now - sgLastPoll_ >= h_.sg_poll_ms)
  {
    sgLastPoll_ = now;
    uint16_t sg_raw = driver_.SG_RESULT();

    if ((now - sgCalibMs_) <= h_.sg_baseline_ms)
    {
      sgSamples_++;
      sgBaseline_ = (sgBaseline_ * (sgSamples_ - 1) + (float)sg_raw) / (float)sgSamples_;
      if (sgSamples_ < 3)
        sgAvg_ = sgBaseline_;
    }
    else
    {
      sgAvg_ = ema_update(sgAvg_, (float)sg_raw, 0.25f);
    }
  }

  // Stall-Entscheidung
  bool stallDetected = false;
  if (sgWindowOver && (sgSamples_ > 5))
  {
    float dropThreshold = fmaxf(sgBaseline_ * (1.0f - h_.sg_drop_pct), (float)h_.sg_abs_thr);
    if (sgAvg_ <= dropThreshold)
      stallDetected = true;
  }
  if (stallDetected)
  {
    long delta = labs((long)(fas_->getCurrentPosition() - homingStartSteps_));
    if (delta < mmToSteps(h_.min_stall_mm))
      stallDetected = false;
  }

  if (hState_ == HomeState::FAST && stallDetected)
  {
    // *** Harter Stopp aller Bewegungen ***
    if (fas_)
    {
      long cur = fas_->getCurrentPosition();
      fas_->forceStopAndNewPosition(cur);

      // Null exakt setzen (0 mm bzw. home_offset_mm)
      fas_->setCurrentPosition(mmToSteps(l_.home_offset_mm));

      // ABSOLUTER Backoff: exakt 3.00 mm weg von 0 (langsam!)
      const float backoff_target_mm = l_.home_offset_mm - homingDirLogical_ * h_.backoff_mm;
      float v = fabsf(h_.speed_slow_hz);
      fas_->setSpeedInHz(v);
      fas_->setAcceleration(v * 2.0f);
      long tgt = mmToSteps(backoff_target_mm) * ((AXIS_UP_DIR_ >= 0) ? +1 : -1);
      fas_->moveTo(tgt);
    }

    hState_ = HomeState::BACKOFF;
    homingBackoffActive_ = true;
    if (debug_)
      Serial.println("[HOME] STALL -> HARD STOP -> ZERO -> BACKOFF(abs, slow)");
    return;
  }

  if (hState_ == HomeState::BACKOFF)
  {
    if (!fas_->isRunning())
    {
      // BACKOFF beendet => Arbeits-Nullpunkt = aktuelle Backoff-Position
      fas_->forceStopAndNewPosition(0);

      // Softlimits neu auf [-max .. 0] setzen (0 ist oberer Arbeitsrand)
      recomputeSoftLimits_();

      hState_ = HomeState::DONE;
      mode_ = Mode::IDLE;
      lastOpDone_ = true;
      homingFinished_ = true;
      homingBackoffActive_ = false;

      if (debug_)
      {
        Serial.println("[HOME] Done (after backoff), zero set to 0.00mm (at backoff)");
      }
    }
  }
}

void StepperCtrl::enforceSoftLimits_()
{
  if (!fas_)
    return;
  long pos = fas_->getCurrentPosition();
  if (pos < softMinSteps_)
  {
    fas_->forceStopAndNewPosition(softMinSteps_);
    mode_ = Mode::IDLE;
    lastOpDone_ = true;
    if (debug_)
      Serial.printf("[SAFE] Softlimit (unten) → auf %.2f mm begrenzen\n", stepsToMm(softMinSteps_));
  }
  else if (pos > softMaxSteps_)
  {
    fas_->forceStopAndNewPosition(softMaxSteps_);
    mode_ = Mode::IDLE;
    lastOpDone_ = true;
    if (debug_)
      Serial.printf("[SAFE] Softlimit (oben) → auf %.2f mm begrenzen\n", stepsToMm(softMaxSteps_));
  }
}

// === stepper_ctrl.cpp – NEU: Debug-Helfer ===
void StepperCtrl::debugPrintLimits() const
{
  float min_mm = stepsToMm(softMinSteps_);
  float max_mm = stepsToMm(softMaxSteps_);
  Serial.printf("[LIM] softMin=%.2f mm (%ld st), softMax=%.2f mm (%ld st), axisUpDir=%d\n",
                min_mm, softMinSteps_, max_mm, softMaxSteps_, AXIS_UP_DIR_);
}

void StepperCtrl::startContinuousLogical_(float speedHz, int logicalDir, bool keepMode)
{
  if (!fas_)
    return;
  // Wenn keepMode false ist, setze Mode auf CONTINUOUS. Während Homing bleibt Mode::HOMING.
  if (!keepMode)
  {
    mode_ = Mode::CONTINUOUS;
  }
  const int motorSign = logicalToMotorDir(logicalDir);
  fas_->setSpeedInHz(fabsf(speedHz));
  fas_->setAcceleration(fabsf(speedHz) * 2.0f);
  lastOpDone_ = false;
  if (motorSign >= 0)
  {
    fas_->runForward();
  }
  else
  {
    fas_->runBackward();
  }
}

void StepperCtrl::moveRelativeLogicalMM_(float mmLogical, float maxHz)
{
  if (!fas_)
    return;
  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1; // +1: forward=UP
  const long stepsMotor = mmToSteps(mmLogical) * (-axisSign);
  const long cur = fas_->getCurrentPosition();
  long tgt = cur + stepsMotor;
  if (tgt < softMinSteps_)
    tgt = softMinSteps_;
  if (tgt > softMaxSteps_)
    tgt = softMaxSteps_;

  mode_ = Mode::GOTO;
  fas_->setSpeedInHz(fabsf(maxHz));
  fas_->setAcceleration(fabsf(maxHz) * 2.0f);
  fas_->move(tgt - cur);
  lastOpDone_ = false;
}

void StepperCtrl::moveRelativeMM_motor_(float mm, float maxHz)
{
  if (!fas_)
    return;
  mode_ = Mode::GOTO;
  fas_->setSpeedInHz(fabsf(maxHz));
  fas_->setAcceleration(fabsf(maxHz) * 2.0f);
  fas_->move(mmToSteps(mm));
  lastOpDone_ = false;
}

void StepperCtrl::homeMoveRelativeMM_(float mmLogical, float maxHz)
{
  if (!fas_)
    return;
  fas_->setSpeedInHz(fabsf(maxHz));
  fas_->setAcceleration(fabsf(maxHz) * 2.0f);
  long cur = fas_->getCurrentPosition();
  long stepsMotor = mmToSteps(mmLogical) * ((AXIS_UP_DIR_ >= 0) ? +1 : -1);
  long target = cur + stepsMotor;
  if (target < softMinSteps_)
    target = softMinSteps_;
  if (target > softMaxSteps_)
    target = softMaxSteps_;
  long delta = target - cur;
  fas_->move(delta);
}

void StepperCtrl::moveToAbsMM_(float mm, float maxHz)
{
  if (!fas_)
    return;
  if (mm < 0)
    mm = 0;
  if (mm > l_.max_travel_mm)
    mm = l_.max_travel_mm;

  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1;    // +1: forward=UP
  const long target_steps = mmToSteps(mm) * (-axisSign); // invertiert

  mode_ = Mode::GOTO;
  fas_->setSpeedInHz(fabsf(maxHz));
  fas_->setAcceleration(fabsf(maxHz) * 2.0f);

  long tgt = target_steps;
  if (tgt < softMinSteps_)
    tgt = softMinSteps_;
  if (tgt > softMaxSteps_)
    tgt = softMaxSteps_;
  fas_->moveTo(tgt);
  lastOpDone_ = false;
}
