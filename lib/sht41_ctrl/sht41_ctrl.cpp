#include "sht41_ctrl.h"

bool SHT41Ctrl::begin(TwoWire& wire, bool doSoftReset) {
  _wire = &wire;

  // Ping: leere Transaktion zur Adresse -> 0 == OK
  _wire->beginTransmission(_addr);
  uint8_t err = _wire->endTransmission();
  if (err != 0) {
    return false;
  }

  if (doSoftReset) {
    if (!softReset()) return false;
    delay(2); // tSR ≤ 1 ms laut Datenblatt, 2 ms Reserve
  }
  return true;
}

bool SHT41Ctrl::softReset() {
  if (_wire == nullptr) return false;
  // Soft-Reset-Kommando 0x94
  _wire->beginTransmission(_addr);
  _wire->write(0x94);
  uint8_t err = _wire->endTransmission();
  // Sensor benötigt bis zu 1 ms, wir geben ihm ~2 ms
  delay(2);
  return (err == 0);
}

bool SHT41Ctrl::read(float& temp_c, float& rh) {
  if (_wire == nullptr) return false;

  // Messung "high precision, no heater": 0xFD
  _wire->beginTransmission(_addr);
  _wire->write(0xFD);
  if (_wire->endTransmission() != 0) {
    return false;
  }

  // High-precision Messzeit ~9 ms -> 10 ms warten
  delay(10);

  // 6 Bytes lesen: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC
  const uint8_t toRead = 6;
  uint8_t buf[toRead] = {0};
  uint8_t n = _wire->requestFrom(_addr, toRead);
  if (n != toRead) {
    return false;
  }
  for (uint8_t i = 0; i < toRead; ++i) {
    buf[i] = _wire->read();
  }

  // CRC prüfen (Sensirion CRC-8, poly 0x31, init 0xFF)
  if (crc8(buf, 2) != buf[2]) return false;
  if (crc8(buf + 3, 2) != buf[5]) return false;

  uint16_t rawT  = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
  uint16_t rawRH = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];

  temp_c = convertTemp(rawT);
  rh     = convertRH(rawRH);
  return true;
}

uint8_t SHT41Ctrl::crc8(const uint8_t* data, uint8_t len) {
  // CRC-8: Poly 0x31, Init 0xFF, no reflect, XOR out 0x00
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x31;
      else            crc <<= 1;
    }
  }
  return crc;
}
