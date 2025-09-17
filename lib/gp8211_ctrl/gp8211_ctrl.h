#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * Minimaler Treiber für den GP8211S (DFR1071).
 * - Prozent (0–100 %) -> 0–10 V
 * - Kein eigenes Wire.begin(); der I²C-Bus wird in main.cpp konfiguriert.
 * - Schreibt 15-Bit DAC-Wert: zuerst Low-Byte, dann High-Byte (siehe Datasheet).
 */
class GP8211Ctrl {
public:
  // 7-bit I²C-Adresse des Moduls (fix)
  static constexpr uint8_t I2C_ADDR = 0x58;

  GP8211Ctrl() : _wire(&Wire) {}

  // Übergib den bereits initialisierten I²C-Bus
  bool begin(TwoWire& wire = Wire) {
    _wire = &wire;
    // 10V Range setzen (Reg 0x01 <- 0x77), danach kurzen Tick warten
    if (!writeReg1Byte(0x01, 0x77)) return false;
    delay(2);
    // kleiner Ping: schreibe 0% (=0V)
    return setPercent(0.0f);
  }

  // Prozent 0..100 -> 0..10V
  bool setPercent(float percent) {
    if (!isfinite(percent)) percent = 0;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    const uint16_t code = static_cast<uint16_t>(lroundf(percent * 32767.0f / 100.0f));
    return setRaw(code);
  }

  // Direkter Spannungswert in Volt (0..10)
  bool setVoltage(float volts) {
    if (!isfinite(volts)) volts = 0;
    if (volts < 0) volts = 0;
    if (volts > 10) volts = 10;
    const uint16_t code = static_cast<uint16_t>(lroundf((volts / 10.0f) * 32767.0f));
    return setRaw(code);
  }

  // Direkter 15-Bit DAC-Code (0..0x7FFF). Achtung: keine Prüfung der Range!
  bool setRaw(uint16_t code15) {
    code15 &= 0x7FFF;
    // Daten laut Datasheet 3.3.3:
    // [START] [ADDR W] [0x02] [DATA_L] [DATA_H] [STOP]
    _wire->beginTransmission(I2C_ADDR);
    _wire->write(uint8_t(0x02));                // "Daten"-Befehl
    _wire->write(uint8_t(code15 & 0xFF));       // DATA Low
    _wire->write(uint8_t((code15 >> 8) & 0x7F)); // DATA High (Bit14..8)
    const uint8_t err = _wire->endTransmission();
    return (err == 0);
  }

  // Optional: 0–5V-Bereich setzen
  bool setRange5V()  { return writeReg1Byte(0x01, 0x55); }
  // Optional: 0–10V-Bereich setzen
  bool setRange10V() { return writeReg1Byte(0x01, 0x77); }

private:
  TwoWire* _wire;

  bool writeReg1Byte(uint8_t reg, uint8_t val) {
    // Datasheet 3.3.4 Range-Set:
    // [START] [ADDR W] [0x01] [0x55|0x77] [STOP]
    _wire->beginTransmission(I2C_ADDR);
    _wire->write(reg);
    _wire->write(val);
    const uint8_t err = _wire->endTransmission();
    return (err == 0);
  }
};
