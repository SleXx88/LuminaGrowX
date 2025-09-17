#pragma once
#include <Arduino.h>
#include <Wire.h>

class SHT41Ctrl {
public:
  // SHT41 hat fixe Adresse 0x44, Parameter bleibt trotzdem offen.
  explicit SHT41Ctrl(uint8_t i2c_addr = 0x44) : _addr(i2c_addr) {}

  // Wire-Instanz wird von außen übergeben (Pins in main.cpp via Wire.begin(...)!).
  bool begin(TwoWire& wire, bool doSoftReset = true);

  // Liest eine Messung (High-Precision, ohne Heater).
  // Gibt true bei Erfolg zurück; temp_c in °C, rh in %RH (0..100).
  bool read(float& temp_c, float& rh);

  // Optional nutzbar, z.B. bei Bus-Glitches.
  bool softReset();

  uint8_t address() const { return _addr; }

private:
  TwoWire* _wire = nullptr;
  uint8_t  _addr  = 0x44;

  static uint8_t crc8(const uint8_t* data, uint8_t len);
  static float convertTemp(uint16_t raw) {
    // T[°C] = -45 + 175 * raw / 65535
    return -45.0f + 175.0f * (static_cast<float>(raw) / 65535.0f);
  }
  static float convertRH(uint16_t raw) {
    // RH[%] = -6 + 125 * raw / 65535, auf 0..100 begrenzen
    float rh = -6.0f + 125.0f * (static_cast<float>(raw) / 65535.0f);
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    return rh;
  }
};
