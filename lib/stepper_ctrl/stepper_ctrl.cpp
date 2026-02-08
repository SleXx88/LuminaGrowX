#include "stepper_ctrl.h"

volatile bool StepperCtrl::diagTriggered_ = false;
void IRAM_ATTR StepperCtrl::onDiagRiseISR_() { StepperCtrl::diagTriggered_ = true; }

StepperCtrl::StepperCtrl(const StepperPins &pins,
                         const StepperKinematics &kin,
                         const StepperLimits &lim,
                         const TMC2209Config &tmc,
                         const HomingParams &hp,
                         const SpeedTable &spd)
    : p_(pins), k_(kin), l_(lim), c_(tmc), h_(hp), s_(spd),
      stepper_(p_.pin_step, p_.pin_dir, p_.pin_en, p_.uart, c_.r_sense, c_.driver_addr)
{
}

void StepperCtrl::recomputeSoftLimits_()
{
  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1; 
  const long bottomSigned = mmToSteps(l_.max_travel_mm) * (-axisSign);
  const long top = 0; 

  softMinSteps_ = (bottomSigned < top) ? bottomSigned : top;
  softMaxSteps_ = (bottomSigned > top) ? bottomSigned : top;
}

void StepperCtrl::begin()
{
  mutex_ = xSemaphoreCreateMutex();

  pinMode(p_.pin_diag, INPUT_PULLUP);

  // Initialize TMCTiny Stepper
  stepper_.begin(p_.uart_baud, p_.uart_rx, p_.uart_tx);
  
  // Initialize TMC Registers
  setupDriver_();

  stepper_.setMaxSpeed(2500.0f);
  stepper_.setAcceleration(5000.0f);
  stepper_.setCurrentPosition(0);

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
  // Start disabled
  stepper_.enable(false);
  delay(100); // Wait for Driver Power-Up / UART Stability
  
  // 1. Address Scan (Auto-detect TMC2209 Address 0-3)
  // Sometimes MS1/MS2 pins are pulled high internally or by board layout.
  uint8_t foundAddr = c_.driver_addr;
  bool found = false;
  uint32_t testVal = 0;

  Serial.println("[INIT] Scanning for TMC2209...");
  for (uint8_t i = 0; i < 4; i++) {
      stepper_.setAddress(i);
      // Try to read GCONF or IFCNT. GCONF is good.
      if (stepper_.readRegister(TMCTiny::GCONF, testVal)) {
          foundAddr = i;
          found = true;
          Serial.printf("[INIT] TMC2209 found at Address %d (GCONF=0x%08X)\n", i, testVal);
          break;
      } else {
          // Debug why it failed
          if (i==0) Serial.printf("[INIT] Addr 0 fail. Available bytes on UART: %d (Need 8+)\n", Serial1.available());
      }
  }

  if (!found) {
      Serial.println("[INIT] ERROR: TMC2209 not found on addr 0-3 (UART Read Failed). Assuming Addr 0 and attempting BLIND WRITE.");
      stepper_.setAddress(c_.driver_addr); // Fallback to config
  }

  // 2. Configuration
  stepper_.setMicrosteps(k_.microsteps);
  
  // Enable StealthChop (required for StallGuard4)
  // We do a blind write to GCONF to ensure StealthChop is ON even if Read failed.
  // Default GCONF is often 0 (StealthChop on). But we explicitly set it.
  // bit 2 = en_spreadCycle (1=Disabled/Spread, 0=Enabled/Stealth)
  // We want Bit 2 = 0.
  // We can't do Read-Modify-Write safely if Read fails, but we can try a sane default.
  // If Read succeeded (found==true), we use the real value.
  uint32_t gconf = 0;
  if (found) {
     stepper_.readRegister(TMCTiny::GCONF, gconf);
  } else {
     // Guessing default flags: internal_Rsense=0 (if ext), pdn_disable=1 (UART use), mstep_reg_select=1
     // Safer: Just assume 0x00000000 works for basics or try to just clear the bit if we could.
     // Without reading, writing GCONF is risky as we might overwrite other settings.
     // However, TMCTiny::setStealthChop does a read first. Let's rely on it only if found.
  }
  
  // Force StealthChop by using the lib function (which reads first). 
  // If read fails, it aborts. That's bad for blind write.
  // Let's force a write if not found.
  if (!found) {
      // Emergency config: Set GCONF to enable StealthChop (Bit 2 = 0)
      // and assuming we want to use UART (pdn_disable=1 -> Bit 6) ??
      // Actually TMC2209 PDN_UART is default.
      // Let's just try to write SGTHRS.
      Serial.println("[INIT] BLIND WRITE: Setting SGTHRS...");
  } else {
      stepper_.setStealthChop(true); 
  }
  
  delay(10); 
  
  // 3. Set StallGuard Threshold (CRITICAL)
  stepper_.setStallGuardThreshold(c_.sg_thrs_diag);
  stepper_.setCoolStep(0xFFFFF); // Enable CoolStep output
  
  // 4. Set Current
  setCurrents(c_.irun_mA, c_.ihold_mA);
  
  if (debug_ && found) {
      uint32_t gconf_verify = 0;
      stepper_.readRegister(TMCTiny::GCONF, gconf_verify);
      Serial.printf("[INIT] Verify GCONF=0x%08X SG_RESULT=%u\n", gconf_verify, (uint16_t)stepper_.readRegister(TMCTiny::SG_RESULT, testVal));
  }
}

void StepperCtrl::tick()
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  // --- Auto Enable/Disable Logic ---
  if (stepper_.isRunning()) {
      lastMotorActivityMs_ = millis();
  } else {
      // Disable after 500ms idle
      if (millis() - lastMotorActivityMs_ > 500) {
          stepper_.enable(false);
      }
  }

  switch (mode_)
  {
  case Mode::CONTINUOUS:
    break;
  case Mode::GOTO:
    if (!stepper_.isRunning())
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

  if (debug_)
  {
    uint32_t now = millis();
    if (now - lastDebugPosMs_ >= debugMoveLogIntervalMs_)
    {
      lastDebugPosMs_ = now;
      if (mode_ == Mode::GOTO)
      {
        float pos = getPositionMm();
        Serial.printf("[MOVE] pos=%.2fmm running=%d\n", pos, stepper_.isRunning());
      }
      else if (mode_ == Mode::CONTINUOUS)
      {
        float pos = getPositionMm();
        Serial.printf("[JOG] pos=%.2fmm running=%d\n", pos, stepper_.isRunning());
      }
      else if (mode_ == Mode::HOMING)
      {
        float pos = getPositionMm();
        Serial.printf("[HOME] pos=%.2fmm sg(avg)=%.1f base=%.1f\n", pos, sgAvg_, sgBaseline_);
      }
    }
  }

  if (mutex_) xSemaphoreGive(mutex_);
}

/* ---------------- Homing ---------------- */

bool StepperCtrl::startHoming(bool fullCalibration)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  
  if (homingFinished_) {
      if (mutex_) xSemaphoreGive(mutex_);
      return true; 
  }
  if (mode_ == Mode::HOMING) {
      if (mutex_) xSemaphoreGive(mutex_);
      return false; 
  }

  fullCalibration_ = fullCalibration;
  homingBackoffActive_ = false;
  newCalibrationAvailable_ = false; // Reset flag
  hState_ = HomeState::PRE_BACKOFF;
  mode_ = Mode::HOMING;
  lastOpDone_ = false;
  homeMoveRelativeMM_(-homingDirLogical_ * h_.backoff_mm, h_.speed_slow_hz);
  if (debug_) Serial.println("[HOME] Pre-backoff start");

  if (mutex_) xSemaphoreGive(mutex_);
  return true;
}

void StepperCtrl::resetHoming()
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

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
  newCalibrationAvailable_ = false;
  if (debug_) Serial.println("[HOME] Reset");

  if (mutex_) xSemaphoreGive(mutex_);
}

std::pair<float, bool> StepperCtrl::goTop(uint8_t speedLevel)
{
  return goToEnd(-1, speedLevel);
}
std::pair<float, bool> StepperCtrl::goBottom(uint8_t speedLevel)
{
  return goToEnd(+1, speedLevel);
}
std::pair<float, bool> StepperCtrl::moveUp(float mm, uint8_t speedLevel)
{
  return moveBy(-1, mm, speedLevel);
}
std::pair<float, bool> StepperCtrl::moveDown(float mm, uint8_t speedLevel)
{
  return moveBy(+1, mm, speedLevel);
}

/* ---------------- Bewegungen ---------------- */

std::pair<float, bool> StepperCtrl::goToEnd(int logicalDir, uint8_t speedLevel)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  // logicalDir: -1 = TOP (softMin), +1 = BOTTOM (softMax)
  const long target = (logicalDir >= 0) ? softMaxSteps_ : softMinSteps_; 
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(stepsToMm(target), s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] goToEnd(%s) lvl=%u\n", (logicalDir >= 0) ? "BOTTOM" : "TOP", lvl);
  
  std::pair<float, bool> ret = {getPositionMm(), (mode_ == Mode::IDLE)};
  
  if (mutex_) xSemaphoreGive(mutex_);
  return ret;
}

std::pair<float, bool> StepperCtrl::moveBy(int logicalDir, float mm, uint8_t speedLevel)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  // logicalDir: -1=Up (negative mm), +1=Down (positive mm)
  const float mmLogical = (logicalDir >= 0) ? +fabsf(mm) : -fabsf(mm);
  
  moveRelativeLogicalMM_(mmLogical, s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] moveBy(mm=%.2f, %s) lvl=%u\n", fabsf(mm), (logicalDir >= 0) ? "DOWN" : "UP", lvl);
  
  std::pair<float, bool> ret = {getPositionMm(), (mode_ == Mode::IDLE)};
  
  if (mutex_) xSemaphoreGive(mutex_);
  return ret;
}

std::pair<float, bool> StepperCtrl::moveTo(float pos_mm, uint8_t speedLevel)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  moveToAbsMM_(pos_mm, s_.hz[lvl]);
  if (debug_) Serial.printf("[CMD] moveTo(%.2fmm) lvl=%u\n", pos_mm, lvl);
  
  std::pair<float, bool> ret = {getPositionMm(), (mode_ == Mode::IDLE)};

  if (mutex_) xSemaphoreGive(mutex_);
  return ret;
}

void StepperCtrl::jogUp(uint8_t speedLevel)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  
  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  // Up is logical -1
  startContinuousLogical_(s_.hz[lvl], -1);
  if (debug_) Serial.printf("[CMD] jogUp lvl=%u\n", lvl);

  if (mutex_) xSemaphoreGive(mutex_);
}
void StepperCtrl::jogDown(uint8_t speedLevel)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  const uint8_t lvl = (speedLevel > 5) ? 5 : speedLevel;
  // Down is logical +1
  startContinuousLogical_(s_.hz[lvl], +1);
  if (debug_) Serial.printf("[CMD] jogDown lvl=%u\n", lvl);

  if (mutex_) xSemaphoreGive(mutex_);
}
void StepperCtrl::jogStop()
{
  stop(); // stop() handles locking
  if (debug_) Serial.println("[CMD] jogStop");
}

void StepperCtrl::stop()
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);

  stepper_.stop();
  mode_ = Mode::IDLE;
  lastOpDone_ = true;

  if (mutex_) xSemaphoreGive(mutex_);
}

bool StepperCtrl::testConnection() {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    uint32_t val = 0;
    bool ok = stepper_.readRegister(TMCTiny::GCONF, val);
    if (mutex_) xSemaphoreGive(mutex_);
    return ok;
}

StepperStatus StepperCtrl::status() const
{
  // Const-cast to use mutex
  StepperCtrl* self = const_cast<StepperCtrl*>(this);
  if (self->mutex_) xSemaphoreTake(self->mutex_, portMAX_DELAY);

  StepperStatus s;
  s.position_mm = getPositionMm();
  s.diag = digitalRead(p_.pin_diag);
  s.isHoming = (mode_ == Mode::HOMING);
  s.isMoving = stepper_.isRunning() || (mode_ == Mode::HOMING);
  s.lastOpDone = lastOpDone_;
  
  // We cannot call testConnection() inside here recursively if mutex is non-recursive.
  // But we have the lock. We can call internal helpers or just replicate logic?
  // testConnection() calls public locking. 
  // Let's just do it raw here since we have the lock.
  // Warning: stepper_ is mutable, so we can access it.
  uint32_t val = 0;
  // This UART read is blocking/slow. Ideally avoid in frequent status(), but status() IS frequent.
  // For now we do it.
  s.uart_ok = self->stepper_.readRegister(TMCTiny::GCONF, val);

  if (self->mutex_) xSemaphoreGive(self->mutex_);
  return s;
}

float StepperCtrl::getPositionMm() const
{
  // getPositionMm is called by status() (already locked) AND internal helpers (called by tick/commands - locked or not?)
  // Internal helpers (tick, moveBy) call it. They HOLD the lock.
  // So getPositionMm MUST NOT lock.
  // But status() calls it, and status() locks.
  // So getPositionMm is "internal-like".
  // Is it called by public API? Yes.
  // If public API calls getPositionMm(), it is unprotected!
  // BUT: getPositionMm() reads atomic volatile (stepper_.getCurrentPosition()).
  // So locking is not strictly required for just reading position.
  long pos = stepper_.getCurrentPosition();
  return stepsToMm(pos);
}

void StepperCtrl::enableDebug(bool on) { debug_ = on; }
void StepperCtrl::setDebugMoveLogInterval(uint16_t ms) { debugMoveLogIntervalMs_ = ms; }

void StepperCtrl::setCurrents(uint16_t irun_mA, uint16_t ihold_mA)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  c_.irun_mA = irun_mA;
  c_.ihold_mA = ihold_mA;
  stepper_.setCurrent(irun_mA, (float)ihold_mA / (float)irun_mA);
  if (mutex_) xSemaphoreGive(mutex_);
}

void StepperCtrl::setMicrosteps(uint8_t microsteps)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  k_.microsteps = microsteps;
  stepper_.setMicrosteps(k_.microsteps);
  if (mutex_) xSemaphoreGive(mutex_);
}

void StepperCtrl::setAxisUpDir(int axis_up_dir)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  AXIS_UP_DIR_ = (axis_up_dir >= 0) ? +1 : -1;
  recomputeSoftLimits_();
  
  long pos = stepper_.getCurrentPosition();
  if (pos < softMinSteps_)
    stepper_.forceStopAndNewPosition(softMinSteps_);
  else if (pos > softMaxSteps_)
    stepper_.forceStopAndNewPosition(softMaxSteps_);
  if (mutex_) xSemaphoreGive(mutex_);
}

void StepperCtrl::setMaxTravelMm(float max_travel_mm)
{
  if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
  l_.max_travel_mm = fmaxf(0.0f, max_travel_mm);
  recomputeSoftLimits_();
  if (mutex_) xSemaphoreGive(mutex_);
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
  // Internal helper, called with lock held
  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  hState_ = next;
  homePhaseMs_ = millis();
  sgCalibMs_ = homePhaseMs_;
  sgSamples_ = 0;
  sgBaseline_ = 0.0f;
  sgAvg_ = 0.0f;
  sgLastPoll_ = 0;

  if (next == HomeState::FAST_UP || next == HomeState::FAST_DOWN)
  {
    homingStartSteps_ = stepper_.getCurrentPosition();
    
    // Target calculation: Allow over-travel to ensure we hit the limit switch
    float target_mm;
    // Note: homingDirLogical is -1 (TOP/UP).
    // If logicalDir < 0 (UP): Target -1.5*max
    // If logicalDir > 0 (DOWN): Target +1.5*max
    if (logicalDir >= 0) target_mm = l_.max_travel_mm * 1.5f; 
    else target_mm = -l_.max_travel_mm * 1.5f; // Was 0.5, increased to 1.5 for safety if offset

    float v = fabsf(speedHz);
    stepper_.setMaxSpeed(v);
    stepper_.setAcceleration(v * 2.0f);
    
    // Use consistent coordinate transform (same as moveToAbsMM_)
    const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1;
    long tgt = mmToSteps(target_mm) * (-axisSign);
    
    stepper_.moveTo(tgt);
    Serial.printf("[HOMING] Start Fast Move (State: %d, Target: %.1f mm)\n", (int)next, target_mm);
  }

  diagTriggered_ = false;
  if (debug_)
  {
    Serial.printf("[HOME] Phase %d start @ %.0f Hz\n", (int)next, speedHz);
  }
}

void StepperCtrl::updateHoming_()
{
  // Internal helper, called with lock held
  uint32_t now = millis();
  bool sgWindowOver = (now - homePhaseMs_) > h_.sg_ignore_ms;

  // 0. Pre-Backoff Completion
  if (hState_ == HomeState::PRE_BACKOFF)
  {
    if (!stepper_.isRunning())
    {
      // Move Fast Up (towards Top, Logical -1)
      beginHomePhase_(HomeState::FAST_UP, h_.speed_fast_hz, homingDirLogical_); // -1
      if (debug_) Serial.println("[HOME] Pre-backoff done -> FAST_UP");
    }
    return;
  }

  // Monitor StallGuard during FAST moves
  bool stallDetected = false;
  if (hState_ == HomeState::FAST_UP || hState_ == HomeState::FAST_DOWN)
  {
      if (now - sgLastPoll_ >= h_.sg_poll_ms)
      {
        sgLastPoll_ = now;
        uint32_t sg_val = 0;
        if (stepper_.readRegister(TMCTiny::SG_RESULT, sg_val)) {
            uint16_t sg_raw = (uint16_t)sg_val;
            if ((now - sgCalibMs_) <= h_.sg_baseline_ms)
            {
              sgSamples_++;
              sgBaseline_ = (sgBaseline_ * (sgSamples_ - 1) + (float)sg_raw) / (float)sgSamples_;
              if (sgSamples_ < 3) sgAvg_ = sgBaseline_;
            }
            else
            {
              sgAvg_ = ema_update(sgAvg_, (float)sg_raw, 0.25f);
            }
        }
      }

      // 1. Check Hardware DIAG Pin
      if (diagTriggered_) {
          stallDetected = true;
          diagTriggered_ = false; // Ack
          Serial.println("[HOMING] Hardware Stop (DIAG Pin)");
      }

      // 2. Check Software Polling
      if (!stallDetected && sgWindowOver && (sgSamples_ > 5))
      {
        float dropThreshold = fmaxf(sgBaseline_ * (1.0f - h_.sg_drop_pct), (float)h_.sg_abs_thr);
        if (sgAvg_ <= dropThreshold) {
            stallDetected = true;
            Serial.printf("[HOMING] Soft Stop (SG: %.1f < %.1f)\n", sgAvg_, dropThreshold);
        }
      }

      // 3. Min distance check
      if (stallDetected)
      {
        long delta = labs((long)(stepper_.getCurrentPosition() - homingStartSteps_));
        if (delta < mmToSteps(h_.min_stall_mm)) {
            stallDetected = false;
        }
      }
  }

  // 1. Handle Stall in FAST_UP (Top detected)
  if (hState_ == HomeState::FAST_UP && stallDetected)
  {
    long cur = stepper_.getCurrentPosition();
    stepper_.forceStopAndNewPosition(cur);
    // Set temporary 0 + offset (we are at Top)
    // Actually, we are at the physical limit.
    // We backoff now.
    
    // Move Down (Logical +1)
    const float backoff_mm = h_.backoff_mm;
    const int backoffDir = (homingDirLogical_ == -1) ? +1 : -1; // -1 is Up, so Backoff is +1 (Down)

    float v = fabsf(h_.speed_slow_hz);
    stepper_.setMaxSpeed(v);
    stepper_.setAcceleration(v * 2.0f);
    
    // Relative move logic
    // We can't use moveRelativeLogicalMM_ because it respects soft limits which might be wrong.
    // Use raw move:
    // Dir +1 (Down) -> Motor +1 (if AxisUp -1).
    // Let's use homeMoveRelativeMM_ which clamps but we are at limit so clamp should be fine if limits are wide enough?
    // But we haven't set limits yet.
    // Use raw stepper move.
    long steps = mmToSteps(backoff_mm);
    // If backoffDir is +1 (Down), and Down is +Steps (since 0 is Top).
    // Yes.
    stepper_.enable(true);
    stepper_.move(steps); // +steps = Down

    hState_ = HomeState::BACKOFF_TOP;
    homingBackoffActive_ = true;
    Serial.printf("[HOMING] Top found. Backing off %.2f mm\n", backoff_mm);
    return;
  }

  // 2. Handle BACKOFF_TOP completion
  if (hState_ == HomeState::BACKOFF_TOP)
  {
    if (!stepper_.isRunning())
    {
      stepper_.forceStopAndNewPosition(0); // Set Zero at Top (after backoff)
      
      if (!fullCalibration_) {
          // Finish here for simple homing
          hState_ = HomeState::DONE;
          mode_ = Mode::IDLE;
          lastOpDone_ = true;
          homingFinished_ = true;
          homingBackoffActive_ = false;
          Serial.println("[HOMING] Simple Homing Complete (Top Zero).");
          return;
      }

      // Now start Phase 2: Find Bottom
      hState_ = HomeState::ZERO_TOP;
      // Fallthrough to next if immediately
    }
  }
  
  if (hState_ == HomeState::ZERO_TOP)
  {
      // Move Fast Down (Logical +1)
      beginHomePhase_(HomeState::FAST_DOWN, h_.speed_fast_hz, +1);
      if (debug_) Serial.println("[HOMING] Zero set. Searching Bottom...");
      return;
  }

  // 3. Handle Stall in FAST_DOWN (Bottom detected)
  if (hState_ == HomeState::FAST_DOWN && stallDetected)
  {
    long cur = stepper_.getCurrentPosition();
    stepper_.forceStopAndNewPosition(cur);
    
    // We are at Bottom.
    // Backoff Up (Logical -1).
    const float backoff_mm = h_.backoff_mm;
    // Up is Negative steps.
    long steps = -mmToSteps(backoff_mm);

    float v = fabsf(h_.speed_slow_hz);
    stepper_.setMaxSpeed(v);
    stepper_.setAcceleration(v * 2.0f);
    
    stepper_.enable(true);
    stepper_.move(steps); 

    hState_ = HomeState::BACKOFF_BOTTOM;
    Serial.printf("[HOMING] Bottom found. Backing off %.2f mm\n", backoff_mm);
    return;
  }

  // 4. Handle BACKOFF_BOTTOM completion
  if (hState_ == HomeState::BACKOFF_BOTTOM)
  {
      if (!stepper_.isRunning())
      {
          hState_ = HomeState::CALC_SPAN;
      }
  }

  // 5. Calculate Span
  if (hState_ == HomeState::CALC_SPAN)
  {
      long pos = stepper_.getCurrentPosition();
      // We are at Bottom Backoff.
      // Position is Steps from Top Zero.
      // Measured Travel = Steps -> mm.
      // Since we defined 0 as Top, Bottom is positive.
      float measured_mm = stepsToMm(pos);
      
      Serial.printf("[HOMING] Calibration Done. Steps=%ld, Measured=%.2f mm\n", pos, measured_mm);
      
      // Update Limit
      // DEADLOCK FIX: Do not call setMaxTravelMm(measured_mm) because it takes the mutex, 
      // and we already hold it in updateHoming_ (called by tick).
      l_.max_travel_mm = measured_mm;
      recomputeSoftLimits_();
      newCalibrationAvailable_ = true;

      // Return to Zero
      moveToAbsMM_(0.0f, h_.speed_fast_hz);
      hState_ = HomeState::RETURN_TO_ZERO;
  }

  // 6. Handle RETURN_TO_ZERO completion
  if (hState_ == HomeState::RETURN_TO_ZERO)
  {
      if (!stepper_.isRunning())
      {
          hState_ = HomeState::DONE;
          mode_ = Mode::IDLE;
          lastOpDone_ = true;
          homingFinished_ = true;
          homingBackoffActive_ = false;
          Serial.println("[HOMING] Sequence Complete. At Top Zero.");
      }
  }
}

void StepperCtrl::enforceSoftLimits_()
{
  long pos = stepper_.getCurrentPosition();
  if (pos < softMinSteps_)
  {
    stepper_.forceStopAndNewPosition(softMinSteps_);
    mode_ = Mode::IDLE;
    lastOpDone_ = true;
  }
  else if (pos > softMaxSteps_)
  {
    stepper_.forceStopAndNewPosition(softMaxSteps_);
    mode_ = Mode::IDLE;
    lastOpDone_ = true;
  }
}

void StepperCtrl::debugPrintLimits() const
{
  Serial.printf("[LIM] softMin=%ld, softMax=%ld\n", softMinSteps_, softMaxSteps_);
}

void StepperCtrl::startContinuousLogical_(float speedHz, int logicalDir, bool keepMode)
{
  // Internal, called with lock
  if (!keepMode) mode_ = Mode::CONTINUOUS;
  
  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  const int motorSign = logicalToMotorDir(logicalDir);
  stepper_.setMaxSpeed(fabsf(speedHz));
  stepper_.setAcceleration(fabsf(speedHz) * 2.0f);
  lastOpDone_ = false;
  
  if (motorSign >= 0) stepper_.runForward();
  else stepper_.runBackward();
}

void StepperCtrl::moveRelativeLogicalMM_(float mmLogical, float maxHz)
{
  // Internal, called with lock
  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1;
  const long stepsMotor = mmToSteps(mmLogical) * (-axisSign);
  const long cur = stepper_.getCurrentPosition();
  long tgt = cur + stepsMotor;
  
  if (tgt < softMinSteps_) tgt = softMinSteps_;
  if (tgt > softMaxSteps_) tgt = softMaxSteps_;

  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(fabsf(maxHz));
  stepper_.setAcceleration(fabsf(maxHz) * 2.0f);
  stepper_.moveTo(tgt);
  lastOpDone_ = false;
}

void StepperCtrl::moveRelativeMM_motor_(float mm, float maxHz)
{
  // Internal, called with lock
  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(fabsf(maxHz));
  stepper_.setAcceleration(fabsf(maxHz) * 2.0f);
  stepper_.move(mmToSteps(mm));
  lastOpDone_ = false;
}

void StepperCtrl::homeMoveRelativeMM_(float mmLogical, float maxHz)
{
  // Internal, called with lock
  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  stepper_.setMaxSpeed(fabsf(maxHz));
  stepper_.setAcceleration(fabsf(maxHz) * 2.0f);
  long cur = stepper_.getCurrentPosition();
  long stepsMotor = mmToSteps(mmLogical) * ((AXIS_UP_DIR_ >= 0) ? +1 : -1);
  long target = cur + stepsMotor;
  
  if (target < softMinSteps_) target = softMinSteps_;
  if (target > softMaxSteps_) target = softMaxSteps_;
  
  stepper_.moveTo(target);
}

void StepperCtrl::moveToAbsMM_(float mm, float maxHz)
{
  // Internal, called with lock
  if (mm < 0) mm = 0;
  if (mm > l_.max_travel_mm) mm = l_.max_travel_mm;

  stepper_.enable(true);
  lastMotorActivityMs_ = millis();

  const int axisSign = (AXIS_UP_DIR_ >= 0) ? +1 : -1;
  const long target_steps = mmToSteps(mm) * (-axisSign);

  mode_ = Mode::GOTO;
  stepper_.setMaxSpeed(fabsf(maxHz));
  stepper_.setAcceleration(fabsf(maxHz) * 2.0f);

  long tgt = target_steps;
  if (tgt < softMinSteps_) tgt = softMinSteps_;
  if (tgt > softMaxSteps_) tgt = softMaxSteps_;
  
  stepper_.moveTo(tgt);
  lastOpDone_ = false;
}
