#include "fan_ctrl.h"

void FanCtrl::begin(const Config& cfg) {
  cfg_ = cfg;

  ledcSetup(cfg_.ledcChannel, cfg_.pwmFreq, cfg_.resolutionBits);
  ledcAttachPin(cfg_.pwmPin, cfg_.ledcChannel);

  // Start mit 0 %
  setPercent(0);
  started_ = true;
}

void FanCtrl::begin(uint8_t pwmPin, uint8_t ledcChannel,
                    bool invert, uint32_t pwmFreq,
                    uint8_t resolutionBits) {
  Config c;
  c.pwmPin = pwmPin;
  c.ledcChannel = ledcChannel;
  c.invert = invert;
  c.pwmFreq = pwmFreq;
  c.resolutionBits = resolutionBits;
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

void FanCtrl::setRaw(uint32_t duty) {
  if (!started_) return;
  uint32_t md = maxDuty();
  if (duty > md) duty = md;
  if (cfg_.invert) {
    duty = md - duty;
  }
  ledcWrite(cfg_.ledcChannel, duty);
}
