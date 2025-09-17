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

  currentPercent_ = percent;

  uint32_t duty = (uint32_t)lroundf((percent / 100.0f) * (float)maxDuty());
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
