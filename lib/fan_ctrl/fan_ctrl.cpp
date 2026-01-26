#include "fan_ctrl.h"

void IRAM_ATTR FanCtrl::isr(void* arg) {
  FanCtrl* instance = static_cast<FanCtrl*>(arg);
  if (instance) {
    instance->pulseCount_++;
  }
}

void FanCtrl::begin(const Config& cfg) {
  cfg_ = cfg;

  ledcSetup(cfg_.ledcChannel, cfg_.pwmFreq, cfg_.resolutionBits);
  ledcAttachPin(cfg_.pwmPin, cfg_.ledcChannel);

  // Tacho Setup
  if (cfg_.tachoPin >= 0) {
    pinMode(cfg_.tachoPin, INPUT_PULLUP);
    // Debug: Ensure pin is in input mode
    digitalWrite(cfg_.tachoPin, HIGH); 
    attachInterruptArg(digitalPinToInterrupt(cfg_.tachoPin), isr, this, FALLING);
    lastRpmCalcMs_ = millis();
    pulseCount_ = 0;
    lastPulseCount_ = 0;
    Serial.printf("[FAN] Init Tacho on Pin %d, Channel %d\n", cfg_.tachoPin, cfg_.ledcChannel);
  } else {
    Serial.printf("[FAN] Init PWM on Pin %d (no Tacho), Channel %d\n", cfg_.pwmPin, cfg_.ledcChannel);
  }

  // Start mit 0 %
  setPercent(0);
  started_ = true;
}

void FanCtrl::begin(uint8_t pwmPin, uint8_t ledcChannel,
                    bool invert, uint32_t pwmFreq,
                    uint8_t resolutionBits, int8_t tachoPin) {
  Config c;
  c.pwmPin = pwmPin;
  c.ledcChannel = ledcChannel;
  c.invert = invert;
  c.pwmFreq = pwmFreq;
  c.resolutionBits = resolutionBits;
  c.tachoPin = tachoPin;
  begin(c);
}

void FanCtrl::setPercent(float percent) {
  if (!started_) return;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  // Store the user-requested percent (0..100)
  currentPercent_ = percent;

  // Remap percent to account for fan's minimum effective PWM from datasheet:
  // - 0 stays 0 (fan off)
  // - 1..100 maps linearly to cfg_.minPercent .. 100
  float effectivePercent = 0.0f;
  if (percent == 0.0f) {
    effectivePercent = 0.0f;
  } else {
    // Clamp minPercent to valid range
    float minP = cfg_.minPercent;
    if (minP < 0.0f) minP = 0.0f;
    if (minP > 100.0f) minP = 100.0f;
    // Map [1..100] -> [minP..100]
    effectivePercent = minP + (percent - 1.0f) * ((100.0f - minP) / 99.0f);
  }

  uint32_t duty = (uint32_t)lroundf((effectivePercent / 100.0f) * (float)maxDuty());
  if (cfg_.invert) {
    duty = maxDuty() - duty;
  }
  ledcWrite(cfg_.ledcChannel, duty);
}

int FanCtrl::getRPM() {
  if (cfg_.tachoPin < 0) return 0;
  
  unsigned long now = millis();
  unsigned long dt = now - lastRpmCalcMs_;
  
  // Berechnung alle z.B. 1000ms
  if (dt >= 1000) {
    // Snapshot holen
    noInterrupts();
    uint32_t count = pulseCount_;
    pulseCount_ = 0; // Reset für nächstes Intervall
    interrupts();
    
    // Pulse pro Umdrehung = 2
    // RPM = (Pulses * 30000) / dt
    if (dt > 0) {
      currentRpm_ = (count * 30000) / dt;
    } else {
      currentRpm_ = 0;
    }
    
    // Debug output for Tacho
    if (currentRpm_ > 0 || count > 0) {
       Serial.printf("[FAN] Tacho Pin %d: %u pulses in %lu ms -> %d RPM\n", 
                     cfg_.tachoPin, count, dt, currentRpm_);
    }

    lastRpmCalcMs_ = now;
  }
  return currentRpm_;
}

void FanCtrl::setRaw(uint32_t duty) {
  if (!started_) return;
  uint32_t md = maxDuty();
  if (duty > md) duty = md;
  if (cfg_.invert) {
    duty = md - duty;
  }
  ledcWrite(cfg_.ledcChannel, duty);
}
