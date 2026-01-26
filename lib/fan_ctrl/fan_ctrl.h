#pragma once
#include <Arduino.h>

/**
 * FanCtrl – einfache 4-Pin Lüftersteuerung (ESP32/ESP32-S3)
 * - PWM 25 kHz (LEDC)
 * - Prozentvorgabe 0..100 %
 *
 * Hinweis:
 * - PC-Lüfter erwarten ein Open-Drain PWM-Signal (active low).
 *   => Transistor/MOSFET zwischen GPIO und Lüfter-PWM-Leitung.
 */
class FanCtrl {
public:
  struct Config {
    uint8_t pwmPin = 2;          // GPIO (in main.cpp setzen)
    uint8_t ledcChannel = 0;     // 0..7
    uint32_t pwmFreq = 25000;    // 25 kHz laut Intel-Spec
    uint8_t resolutionBits = 8;  // 8 Bit = 0..255 Duty
    bool invert = false;          // active-low (meist TRUE)
    float minPercent = 10.0f;     // Minimum effective percent from datasheet (0..100).
    int8_t tachoPin = -1;         // Tacho-Pin (optional), < 0 deaktiviert
  };

  FanCtrl() = default;

  void begin(const Config& cfg);
  void begin(uint8_t pwmPin, uint8_t ledcChannel = 0,
             bool invert = true, uint32_t pwmFreq = 25000,
             uint8_t resolutionBits = 8, int8_t tachoPin = -1);

  // Setzen in %
  void setPercent(float percent);    // 0..100
  float getPercent() const { return currentPercent_; }
  
  // RPM lesen (falls Tacho-Pin konfiguriert)
  int getRPM();

  // Raw Duty (0..maxDuty)
  void setRaw(uint32_t duty);
  uint32_t maxDuty() const { return (1UL << cfg_.resolutionBits) - 1; }

private:
  Config cfg_;
  bool started_ = false;
  float currentPercent_ = 0.0f;
  
  // RPM-Messung
  volatile uint32_t pulseCount_ = 0;
  uint32_t lastPulseCount_ = 0;
  unsigned long lastRpmCalcMs_ = 0;
  int currentRpm_ = 0;

  static void IRAM_ATTR isr(void* arg);
};