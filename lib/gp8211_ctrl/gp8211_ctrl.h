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
  // Mapping entsprechend Anforderung:
  //   - 0%  -> 0V (AUS)
  //   - 1..100% -> Vmin..10V linear (1% == Vmin, 100% == 10V)
  //     Vmin ist konfigurierbar (default 1.0V, z.B. 0.6V möglich)
  bool setPercent(float percent) {
    if (!isfinite(percent)) percent = 0;
    if (percent <= 0.0f) {
      lastPercent_ = 0.0f;
      return setRaw(0); // 0V
    }
    if (percent > 100.0f) percent = 100.0f;
    lastPercent_ = percent;
    // Erzwinge Mindestwert 1% => Vmin
    float p = percent;
    if (p < 1.0f) p = 1.0f;
    const float vmin = clampVmin_(minVoltAt1Pct_);
    // Linear 1..100% -> Vmin..10V
    const float volts = vmin + (p - 1.0f) * ((10.0f - vmin) / 99.0f);
    const uint16_t code = static_cast<uint16_t>(lroundf((volts / 10.0f) * 32767.0f));
    return setRaw(code);
  }

  float getPercent() const { return lastPercent_; }

  // Effektiver Prozentwert (Anzeige):
  //  - 0% bleibt 0%
  //  - 0% < x < 1% wird als 1% angezeigt (weil min. Vmin ausgegeben wird)
  //  - 1..100% unverändert
  float effectivePercent(float requestedPercent) const {
    if (!isfinite(requestedPercent) || requestedPercent <= 0.0f) return 0.0f;
    if (requestedPercent < 1.0f) return 1.0f;
    if (requestedPercent > 100.0f) return 100.0f;
    return requestedPercent;
  }

  // Konfiguriere die Spannung, die bei 1% ausgegeben werden soll (Vmin)
  // Bereich: 0.0V .. <10V (sinnvoll 0.4V..2.0V). Wird intern geklemmt.
  void setMinVoltAt1Percent(float volts) {
    if (!isfinite(volts)) return;
    minVoltAt1Pct_ = clampVmin_(volts);
  }

  float getMinVoltAt1Percent() const { return minVoltAt1Pct_; }

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
  float    minVoltAt1Pct_ = 1.0f; // Default: 1.0V bei 1%
  float    lastPercent_ = 0.0f;

  static float clampVmin_(float v) {
    if (!isfinite(v)) return 1.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 9.9f) v = 9.9f; // <10V
    return v;
  }

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
