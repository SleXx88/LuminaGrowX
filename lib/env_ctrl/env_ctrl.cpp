/*
 * env_ctrl.cpp
 *
 * Implementierung der EnvCtrl-Klasse zur Steuerung der Umgebung einer Growbox.
 * Siehe env_ctrl.h für Details und Verwendung. Der Controller verwendet eine
 * proportionale Anpassung der LED- und Lüfterausgänge basierend auf dem
 * Unterschied zwischen dem gemessenen Dampfdruckdefizit (VPD) und dem
 * Mittelpunkt des konfigurierten Ziel-VPD-Bereichs. Die Standardeinstellungen
 * spiegeln typische gärtnerische Empfehlungen wider, aber alle Parameter
 * können zur Laufzeit überschrieben werden.
 */

#include "env_ctrl.h"
#include <math.h>

using namespace env_ctrl;
using vpd_calc::computeVpd;

EnvCtrl::EnvCtrl()
    : stage_(vpd_calc::GrowthStage::Seedling),
      mode_(DayMode::Day),
      kpFan_(20.0f),
      kpLed_(10.0f),
      sensor_(nullptr),
      ledDriver_(nullptr),
      fan_(nullptr),
      ledPercent_(0.0f),
      fanPercent_(0.0f),
      vpd_(0.0),
      temp_(0.0),
      rh_(0.0)
{
    // Initialise default phase/mode settings.
    // See env_ctrl.h for explanation of indices.
    // Seedling (index 0)
    phaseSettings_[0][0] = { /*Day*/
        30.0f, 50.0f,   // LED min/max ------------------------- 30.0f, 50.0f,
        10.0f, 15.0f,   // Fan min/max  ------------------------- 20.0f, 80.0f,
        0.4f, 0.8f      // VPD min/max (kPa)
    };
    phaseSettings_[0][1] = { /*Night*/
        0.0f,  0.0f,
        20.0f, 60.0f,
        0.4f, 0.8f
    };
    phaseSettings_[0][2] = { /*NightSilent*/
        0.0f,  0.0f,
        10.0f, 40.0f,
        0.4f, 0.8f
    };

    // Vegetative (index 1)
    phaseSettings_[1][0] = { /*Day*/
        50.0f, 80.0f,
        20.0f, 100.0f,
        0.8f, 1.2f
    };
    phaseSettings_[1][1] = { /*Night*/
        0.0f, 0.0f,
        20.0f, 60.0f,
        0.8f, 1.2f
    };
    phaseSettings_[1][2] = { /*NightSilent*/
        0.0f, 0.0f,
        10.0f, 40.0f,
        0.8f, 1.2f
    };

    // Flowering (index 2)
    phaseSettings_[2][0] = { /*Day*/
        80.0f, 100.0f,
        30.0f, 100.0f,
        1.2f, 1.6f
    };
    phaseSettings_[2][1] = { /*Night*/
        0.0f, 0.0f,
        20.0f, 70.0f,
        1.2f, 1.6f
    };
    phaseSettings_[2][2] = { /*NightSilent*/
        0.0f, 0.0f,
        10.0f, 50.0f,
        1.2f, 1.6f
    };
}

void EnvCtrl::begin(SHT41Ctrl& sensor, GP8211Ctrl& ledDriver, FanCtrl& fan) {
    sensor_ = &sensor;
    ledDriver_ = &ledDriver;
    fan_ = &fan;
}

void EnvCtrl::setStage(vpd_calc::GrowthStage stage) {
    stage_ = stage;
}

void EnvCtrl::setMode(DayMode mode) {
    mode_ = mode;
}

void EnvCtrl::setStageModeLimits(vpd_calc::GrowthStage stage, DayMode mode,
                                 float ledMin, float ledMax,
                                 float fanMin, float fanMax,
                                 float vpdMin, float vpdMax) {
    int si = stageIndex(stage);
    int mi = modeIndex(mode);

    // Clamp values to safe ranges
    if (ledMin < 0.0f) ledMin = 0.0f;
    if (ledMax > 100.0f) ledMax = 100.0f;
    if (fanMin < 0.0f) fanMin = 0.0f;
    if (fanMax > 100.0f) fanMax = 100.0f;
    if (ledMax < ledMin) ledMax = ledMin;
    if (fanMax < fanMin) fanMax = fanMin;
    if (vpdMin < 0.0f) vpdMin = 0.0f;
    if (vpdMax < vpdMin) vpdMax = vpdMin;

    phaseSettings_[si][mi].ledMin = ledMin;
    phaseSettings_[si][mi].ledMax = ledMax;
    phaseSettings_[si][mi].fanMin = fanMin;
    phaseSettings_[si][mi].fanMax = fanMax;
    phaseSettings_[si][mi].vpdMin = vpdMin;
    phaseSettings_[si][mi].vpdMax = vpdMax;
}

void EnvCtrl::setKpFan(float kp) {
    kpFan_ = kp;
}

void EnvCtrl::setKpLed(float kp) {
    kpLed_ = kp;
}

bool EnvCtrl::update() {
    if (!sensor_ || !ledDriver_ || !fan_) {
        return false;
    }

    // Read sensor; if it fails, return false to indicate no update
    float t, h;
    if (!sensor_->read(t, h)) {
        return false;
    }
    temp_ = static_cast<double>(t);
    rh_   = static_cast<double>(h);

    // Compute VPD using the vpd_calc library
    vpd_ = computeVpd(temp_, rh_);

    // Fetch current limits and target range
    PhaseModeSettings& ps = currentSettings();

    float target = (ps.vpdMin + ps.vpdMax) * 0.5f;
    float error = static_cast<float>(vpd_) - target;

    // Compute base setpoints as midpoints of the output ranges
    float baseLed = (ps.ledMin + ps.ledMax) * 0.5f;
    float baseFan = (ps.fanMin + ps.fanMax) * 0.5f;

    // Proportional control: decrease LED when VPD is high (error>0),
    // increase LED when VPD is low (error<0).  For fan it is
    // inverted: increase fan when VPD is low (more humidity),
    // decrease fan when VPD is high (dry conditions).
    float ledOut = baseLed - kpLed_ * error;
    float fanOut = baseFan - kpFan_ * error;

    // Clamp outputs to configured limits
    ledPercent_ = clamp(ledOut, ps.ledMin, ps.ledMax);
    fanPercent_ = clamp(fanOut, ps.fanMin, ps.fanMax);

    // Apply outputs to hardware
    ledDriver_->setPercent(ledPercent_);
    fan_->setPercent(fanPercent_);

    return true;
}