#include "stepper_ctrl.h"

// DIAG-Flag (Fehlerindikator, nicht Stall)
volatile bool StepperCtrl::diagTriggered_ = false;
void IRAM_ATTR StepperCtrl::onDiagRiseISR_() { StepperCtrl::diagTriggered_ = true; }

StepperCtrl::StepperCtrl(const StepperPins& pins,
                         const StepperKinematics& kin,
                         const StepperLimits& lim,
                         const TMC2209Config& tmc,
                         const HomingParams& hp,
                         const SpeedTable& spd)
: p_(pins), k_(kin), l_(lim), c_(tmc), h_(hp), s_(spd),
  driver_(p_.uart, c_.r_sense, c_.driver_addr),
  stepper_(AccelStepper::DRIVER, p_.pin_step, p_.pin_dir)
{}

void StepperCtrl::begin() {
  pinMode(p_.pin_step, OUTPUT);
  pinMode(p_.pin_dir,  OUTPUT);
  pinMode(p_.pin_en,   OUTPUT);
  pinMode(p_.pin_diag, INPUT_PULLUP);

  p_.uart->begin(p_.uart_baud, SERIAL_8N1, p_.uart_rx, p_.uart_tx);

  setupDriver_();

  stepper_.setMaxSpeed(2500);
  stepper_.setAcceleration(5000);

  softMinSteps_ = 0;
  softMaxSteps_ = mmToSteps(l_.max_travel_mm);

  homingFinished_ = false;

  attachInterrupt(digitalPinToInterrupt(p_.pin_diag), StepperCtrl::onDiagRiseISR_, RISING);

  if (debug_) {
    Serial.printf("[INIT] microsteps=%u lead=%.2fmm Irun=%umA Ihold=%umA max=%.1fmm\n",
      k_.microsteps, k_.lead_mm, c_.irun_mA, c_.ihold_mA, l_.max_travel_mm);
  }
}

void StepperCtrl::setupDriver_() {
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

  // Simple CoolStep defaults
  driver_.semin(5); driver_.semax(2); driver_.sedn(0b01); driver_.seimin(false);

  setCurrents(c_.irun_mA, c_.ihold_mA);

  digitalWrite(p_.pin_en, LOW); // enable driver
}

void StepperCtrl::tick() {
  switch (mode_) {
    case Mode::CONTINUOUS:
      stepper_.runSpeed();
      break;
    case Mode::GOTO:
      stepper_.run();
      if (stepper_.distanceToGo() == 0) {
        mode_ = Mode::IDLE;
        lastOpDone_ = true;
        if (debug_) {
          float pos = getPositionMm();
          Serial.printf("[MOVE] Reached target, pos=%.2fmm\n", pos);
        }
      }
      break;
    case Mode::HOMING:
      stepper_.runSpeed();
      updateHoming_();
      break;
    case Mode::IDLE:
    default:
      break;
  }

  if (mode_ != Mode::HOMING) enforceSoftLimits_();

  // Throttled move logs
  if (debug_) {
    uint32_t now = millis();
    if (now - lastDebugPosMs_ >= debugMoveLogIntervalMs_) {
      lastDebugPosMs_ = now;
      if (mode_ == Mode::GOTO) {
        long dist = const_cast<AccelStepper&>(stepper_).distanceToGo();
        float pos = getPositionMm();
        float tgt = stepsToMm(const_cast<AccelStepper&>(stepper_).targetPosition());
        Serial.printf("[MOVE] pos=%.2fmm -> target=%.2fmm, d=%ld\n", pos, tgt, dist);
      } else if (mode_ == Mode::CONTINUOUS) {
        float pos = getPositionMm();
        float sp  = const_cast<AccelStepper&>(stepper_).speed();
        Serial.printf("[JOG] pos=%.2fmm speed=%.0f sps\n", pos, sp);
      } else if (mode_ == Mode::HOMING) {
        float pos = getPositionMm();
        Serial.printf("[HOME] pos=%.2fmm sg(avg)=%.1f base=%.1f\n", pos, sgAvg_, sgBaseline_);
      }
    }
  }
}

/* ---------------- Homing ---------------- */

bool StepperCtrl::startHoming() {
  if (homingFinished_) return true;         // bereits abgeschlossen
  if (mode_ == Mode::HOMING) return false;  // läuft noch

  // kurzer Backoff (blockierend, wenige mm)
  moveRelativeLogicalMM_(-h_.backoff_mm * (float)homingDirLogical_, h_.speed_slow_hz);
  while (stepper_.distanceToGo() != 0) stepper_.run();

  beginHomePhase_(HomeState::FAST, h_.speed_fast_hz, homingDirLogical_);
  mode_ = Mode::HOMING;
  lastOpDone_ = false;

  if (debug_) Serial.println("[HOME] Start (FAST)");

  return false;
}

void StepperCtrl::resetHoming() {
  homingFinished_ = false;
  hState_ = HomeState::IDLE;
  mode_   = Mode::IDLE;
  lastOpDone_ = false;
  sgCalibMs_  = 0;
  sgSamples_  = 0;
  sgBaseline_ = 0.0f;
  sgAvg_      = 0.0f;
  diagTriggered_ = false;
  if (debug_) Serial.println("[HOME] Reset");
}

std::pair<float,bool> StepperCtrl::goTop(uint8_t speedLevel) {
  return goToEnd(+1, speedLevel);
}
std::pair<float,bool> StepperCtrl::goBottom(uint8_t speedLevel) {
  return goToEnd(-1, speedLevel);
}
std::pair<float,bool> StepperCtrl::moveUp(float mm, uint8_t speedLevel) {
  return moveBy(+1, mm, speedLevel);
}
std::pair<float,bool> StepperCtrl::moveDown(float mm, uint8_t speedLevel) {
  return moveBy(-1, mm, speedLevel);
}

/* ---------------- Bewegungen ---------------- */

std::pair<float,bool> StepperCtrl::goToEnd(int logicalDir, uint8_t speedLevel) {
  const long target = (logicalDir >= 0) ? softMaxSteps_ : softMinSteps_;
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(stepsToMm(target), s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] goToEnd(%s) lvl=%u\n", (logicalDir>=0)?"TOP":"BOTTOM", lvl);
  return { getPositionMm(), (mode_==Mode::IDLE) };
}

std::pair<float,bool> StepperCtrl::moveBy(int logicalDir, float mm, uint8_t speedLevel) {
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  const float mmLogical = (logicalDir >= 0) ? fabsf(mm) : -fabsf(mm);
  moveRelativeLogicalMM_(mmLogical, s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] moveBy(mm=%.2f, %s) lvl=%u\n",
                            fabsf(mm), (logicalDir>=0)?"UP":"DOWN", lvl);
  return { getPositionMm(), (mode_==Mode::IDLE) };
}

std::pair<float,bool> StepperCtrl::moveTo(float pos_mm, uint8_t speedLevel) {
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(pos_mm, s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] moveTo(%.2fmm) lvl=%u\n", pos_mm, lvl);
  return { getPositionMm(), (mode_==Mode::IDLE) };
}

void StepperCtrl::jogUp(uint8_t speedLevel) {
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  startContinuousLogical_(s_.hz[lvl], +1);
  if (debug_) Serial.printf("[CMD] jogUp lvl=%u\n", lvl);
}
void StepperCtrl::jogDown(uint8_t speedLevel) {
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  startContinuousLogical_(s_.hz[lvl], -1);
  if (debug_) Serial.printf("[CMD] jogDown lvl=%u\n", lvl);
}
void StepperCtrl::jogStop() {
  stop();
  if (debug_) Serial.println("[CMD] jogStop");
}

void StepperCtrl::stop() {
  mode_ = Mode::IDLE;
  stepper_.stop();
  stepper_.setSpeed(0);
}

StepperStatus StepperCtrl::status() const {
  StepperStatus s;
  s.position_mm = getPositionMm();
  s.diag        = digitalRead(p_.pin_diag);
  s.isHoming    = (mode_ == Mode::HOMING);
  s.isMoving    = (mode_ == Mode::HOMING) || (mode_ == Mode::CONTINUOUS) || (mode_ == Mode::GOTO);
  s.lastOpDone  = lastOpDone_;
  return s;
}

float StepperCtrl::getPositionMm() const {
  long pos = const_cast<AccelStepper&>(stepper_).currentPosition();
  return stepsToMm(pos);
}

void StepperCtrl::enableDebug(bool on) { debug_ = on; }
void StepperCtrl::setDebugMoveLogInterval(uint16_t ms) { debugMoveLogIntervalMs_ = ms; }

void StepperCtrl::setCurrents(uint16_t irun_mA, uint16_t ihold_mA) {
  c_.irun_mA  = irun_mA;
  c_.ihold_mA = ihold_mA;
  float mult = (c_.irun_mA == 0) ? 0.0f : (float)c_.ihold_mA / (float)c_.irun_mA;
  if (mult < 0.0f) mult = 0.0f;
  driver_.rms_current(c_.irun_mA, mult);
  driver_.hold_multiplier(mult);
}

void StepperCtrl::setMicrosteps(uint8_t microsteps) {
  k_.microsteps = microsteps;
  driver_.microsteps(k_.microsteps);
}

void StepperCtrl::setAxisUpDir(int axis_up_dir) {
  AXIS_UP_DIR_ = (axis_up_dir >= 0) ? +1 : -1;
}

void StepperCtrl::setMaxTravelMm(float max_travel_mm) {
  l_.max_travel_mm = fmaxf(0.0f, max_travel_mm);
  softMaxSteps_ = mmToSteps(l_.max_travel_mm);
}

/* ---------- internals ---------- */
inline long  StepperCtrl::mmToSteps(float mm) const {
  const float steps_per_mm = (k_.steps_per_rev * k_.microsteps) / k_.lead_mm;
  return lroundf(mm * steps_per_mm);
}
inline float StepperCtrl::stepsToMm(long st) const {
  const float steps_per_mm = (k_.steps_per_rev * k_.microsteps) / k_.lead_mm;
  return (float)st / steps_per_mm;
}
inline int StepperCtrl::logicalToMotorDir(int logicalDir) const {
  return (logicalDir >= 0) ? AXIS_UP_DIR_ : -AXIS_UP_DIR_;
}
inline float StepperCtrl::stepsToLogicalMm(long deltaSteps) const {
  return stepsToMm(deltaSteps) * (float)((AXIS_UP_DIR_ >= 0) ? +1 : -1);
}
inline float StepperCtrl::ema_update(float prev, float sample, float alpha) {
  return alpha * sample + (1.0f - alpha) * prev;
}

void StepperCtrl::beginHomePhase_(HomeState next, float speedHz, int logicalDir) {
  hState_ = next;
  homePhaseMs_ = millis();
  sgCalibMs_   = homePhaseMs_;
  sgSamples_   = 0;
  sgBaseline_  = 0.0f;
  sgAvg_       = 0.0f;
  sgLastPoll_  = 0;

  if (next == HomeState::FAST) {
    homingStartSteps_ = stepper_.currentPosition();
  }
  diagTriggered_ = false;

  startContinuousLogical_(speedHz, logicalDir);

  if (debug_) {
    const char* name = (next == HomeState::FAST) ? "FAST" :
                       (next == HomeState::BACKOFF) ? "BACKOFF" :
                       (next == HomeState::DONE) ? "DONE" : "?";
    Serial.printf("[HOME] Phase %s start @ %.0f Hz, logDir=%d\n", name, speedHz, logicalDir);
  }
}

void StepperCtrl::updateHoming_() {
  uint32_t now = millis();
  bool sgWindowOver = (now - homePhaseMs_) > h_.sg_ignore_ms;

  // SG_RESULT pollen
  if (now - sgLastPoll_ >= h_.sg_poll_ms) {
    sgLastPoll_ = now;
    uint16_t sg_raw = driver_.SG_RESULT();

    if ((now - sgCalibMs_) <= h_.sg_baseline_ms) {
      sgSamples_++;
      sgBaseline_ = (sgBaseline_ * (sgSamples_ - 1) + (float)sg_raw) / (float)sgSamples_;
      if (sgSamples_ < 3) sgAvg_ = sgBaseline_;
    } else {
      sgAvg_ = ema_update(sgAvg_, (float)sg_raw, 0.25f);
    }

    // 100ms-prints kommen aus tick(), hier nur Berechnung
  }

  // Stall-Entscheidung
  bool stallDetected = false;
  if (sgWindowOver && (sgSamples_ > 5)) {
    float dropThreshold = fmaxf(sgBaseline_ * (1.0f - h_.sg_drop_pct), (float)h_.sg_abs_thr);
    if (sgAvg_ <= dropThreshold) stallDetected = true;
  }
  if (stallDetected) {
    long delta = labs(stepper_.currentPosition() - homingStartSteps_);
    if (delta < mmToSteps(h_.min_stall_mm)) stallDetected = false;
  }

  if (hState_ == HomeState::FAST && stallDetected) {
    // Stop & kurzer Backoff
    stop();
    moveRelativeLogicalMM_(-h_.backoff_mm * (float)homingDirLogical_, h_.speed_fast_hz);
    while (stepper_.distanceToGo() != 0) stepper_.run();

    // Nullpunkt setzen
    stepper_.setCurrentPosition(mmToSteps(l_.home_offset_mm));
    hState_     = HomeState::DONE;
    mode_       = Mode::IDLE;
    lastOpDone_ = true;
    homingFinished_ = true;

    if (debug_) {
      float pos = getPositionMm();
      Serial.printf("[HOME] Done: zero set @ %.2fmm (expected %.2fmm)\n",
                    pos, l_.home_offset_mm);
    }
  }
}

void StepperCtrl::enforceSoftLimits_() {
  long pos = stepper_.currentPosition();
  if (pos < softMinSteps_) {
    stepper_.setCurrentPosition(softMinSteps_);
    mode_ = Mode::IDLE;
    stepper_.stop();
    if (debug_) Serial.println("[SAFE] Softlimit (bottom) → clamp to 0mm");
  } else if (pos > softMaxSteps_) {
    stepper_.setCurrentPosition(softMaxSteps_);
    mode_ = Mode::IDLE;
    stepper_.stop();
    if (debug_) Serial.printf("[SAFE] Softlimit (top) → clamp to %.2fmm\n", l_.max_travel_mm);
  }
}

void StepperCtrl::startContinuousLogical_(float speedHz, int logicalDir) {
  mode_ = Mode::CONTINUOUS;
  const int motorSign = logicalToMotorDir(logicalDir);
  const float sps = speedHz * (float)motorSign;
  stepper_.setSpeed(sps);
  lastOpDone_ = false;
}

void StepperCtrl::moveRelativeLogicalMM_(float mmLogical, float maxHz) {
  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(maxHz);
  stepper_.setAcceleration(maxHz * 2.0f);
  long stepsMotor = mmToSteps(mmLogical) * ((AXIS_UP_DIR_ >= 0) ? +1 : -1);
  stepper_.move(stepsMotor);
  lastOpDone_ = false;
}

void StepperCtrl::moveRelativeMM_motor_(float mm, float maxHz) {
  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(maxHz);
  stepper_.setAcceleration(maxHz * 2.0f);
  stepper_.move(mmToSteps(mm));
  lastOpDone_ = false;
}

void StepperCtrl::moveToAbsMM_(float mm, float maxHz) {
  if (mm < 0) mm = 0;
  if (mm > l_.max_travel_mm) mm = l_.max_travel_mm;
  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(maxHz);
  stepper_.setAcceleration(maxHz * 2.0f);
  stepper_.moveTo(mmToSteps(mm));
  lastOpDone_ = false;
}
