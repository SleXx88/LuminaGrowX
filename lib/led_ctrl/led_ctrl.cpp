#include "led_ctrl.h"

LedCtrl::LedCtrl(int pin, int numLeds) 
    : _pin(pin), _numLeds(numLeds), 
      _pixels(numLeds, pin, NEO_GRB + NEO_KHZ800) 
{
}

void LedCtrl::begin() {
    _pixels.begin();
    _pixels.setBrightness(_globalBrightness);
    _pixels.show(); // Initialize all pixels to 'off'
    _targetColorPacked = 0; // Off
}

void LedCtrl::setColor(Color color) {
    _targetColorPacked = getColorPacked(color);
    // If solid, update immediately
    if (_currentMode == Mode::SOLID) {
        showColor(_targetColorPacked);
    }
}

void LedCtrl::setCustomColor(uint8_t r, uint8_t g, uint8_t b) {
    _targetColorPacked = _pixels.Color(r, g, b);
    if (_currentMode == Mode::SOLID) {
        showColor(_targetColorPacked);
    }
}

void LedCtrl::setBrightness(uint8_t brightness) {
    _globalBrightness = brightness;
    _pixels.setBrightness(_globalBrightness);
    // Re-show current state
    if (_currentMode == Mode::SOLID) {
        showColor(_targetColorPacked);
    }
}

void LedCtrl::setMode(Mode mode) {
    _currentMode = mode;
    // Reset timing or state if needed
    if (mode == Mode::SOLID) {
        showColor(_targetColorPacked);
    }
}

void LedCtrl::setCycleTime(unsigned long ms) {
    if (ms == 0) ms = 100; // safety
    _cycleDuration = ms;
}

void LedCtrl::update() {
    unsigned long now = millis();

    switch (_currentMode) {
        case Mode::SOLID:
            // Do nothing, already set
            break;

        case Mode::BLINK: {
            // Simple square wave: 50% duty cycle
            // Use modulo of cycle duration
            unsigned long phase = now % _cycleDuration;
            if (phase < (_cycleDuration / 2)) {
                // ON
                showColor(_targetColorPacked);
            } else {
                // OFF
                showColor(0);
            }
            break;
        }

        case Mode::BREATHE: {
            // Smooth sine wave
            // angle goes from 0 to 2*PI over _cycleDuration
            float angle = (float)(now % _cycleDuration) / (float)_cycleDuration * 2.0f * PI;
            // sin goes -1 to 1. map to 0.05 to 1.0 (min brightness to max)
            // (sin(angle) + 1) / 2 goes 0 to 1
            float factor = (sin(angle - PI/2.0f) + 1.0f) / 2.0f; 
            
            // To prevent total darkness, maybe clamp min to 5%
            if (factor < 0.05f) factor = 0.05f;

            // We apply this factor to the components of _targetColorPacked
            // Note: _pixels.setBrightness handles global scaling, 
            // so we scale the color values themselves here relative to max.
            
            uint8_t r = (uint8_t)((_targetColorPacked >> 16) & 0xFF);
            uint8_t g = (uint8_t)((_targetColorPacked >> 8) & 0xFF);
            uint8_t b = (uint8_t)(_targetColorPacked & 0xFF);

            r = (uint8_t)(r * factor);
            g = (uint8_t)(g * factor);
            b = (uint8_t)(b * factor);

            _pixels.setPixelColor(0, _pixels.Color(r, g, b));
            _pixels.show();
            break;
        }

        case Mode::RAINBOW: {
            // Cycle hue over time
            // full hue cycle is 65536
            // we want to complete one cycle in _cycleDuration
            // This update needs to be fast enough
            unsigned long progress = now % _cycleDuration;
            uint16_t hue = (uint16_t)(65536UL * progress / _cycleDuration);
            
            _pixels.setPixelColor(0, _pixels.ColorHSV(hue, 255, 255));
            _pixels.show();
            break;
        }
    }
}

void LedCtrl::showColor(uint32_t color) {
    for(int i=0; i<_numLeds; i++) {
        _pixels.setPixelColor(i, color);
    }
    _pixels.show();
}

uint32_t LedCtrl::getColorPacked(Color c) {
    switch (c) {
        case Color::OFF:        return _pixels.Color(0, 0, 0);
        case Color::RED:        return _pixels.Color(255, 0, 0);
        case Color::GREEN:      return _pixels.Color(0, 255, 0);
        case Color::BLUE:       return _pixels.Color(0, 0, 255);
        case Color::WHITE:      return _pixels.Color(255, 255, 255);
        case Color::ORANGE:     return _pixels.Color(255, 165, 0);
        case Color::PURPLE:     return _pixels.Color(128, 0, 128);
        case Color::CYAN:       return _pixels.Color(0, 255, 255);
        case Color::YELLOW:     return _pixels.Color(255, 255, 0);
        case Color::MAGENTA:    return _pixels.Color(255, 0, 255);
        case Color::WARM_WHITE: return _pixels.Color(255, 244, 229);
        default:                return _pixels.Color(0, 0, 0);
    }
}
