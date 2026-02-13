// tof_ctrl.cpp
//
// Implementierung der ToFCtrl‑Klasse für den VL53L0X‑Sensor. Diese Datei
// enthält die Low‑Level‑Initialisierung und Messlogik, die weitgehend aus
// der bereitgestellten C‑Bibliothek portiert und auf das Arduino‑Framework
// (Wire) angepasst wurde. Die Struktur spiegelt die Funktionen der
// stepper_ctrl‑Bibliothek wider: klar getrennte Init‑Routine, einfache
// Abfragefunktionen und optionaler XSHUT‑Support.

#include "tof_ctrl.h"
// LittleFS für Kalibrierungs-Persistenz
#include <LittleFS.h>

// Persistenzpfad für Offset-Kalibrierung
const char* ToFCtrl::kOffsetPath = "/tof_offset.dat";
// #include "tof_calib_config.h" // nicht mehr benötigt

// --- statische Hilfsfunktionen ---
// Berechnet die Makroperiode (in ns) für die VCSEL‑Periode.
static inline uint32_t calcMacroPeriod(uint8_t vcsel_period_pclks) {
  return ((uint32_t)2304 * vcsel_period_pclks * 1655 + 500) / 1000;
}
// Kodiert die VCSEL‑Periode (siehe Datenblatt).
static inline uint8_t encodeVcselPeriod(uint8_t period_pclks) {
  return (period_pclks >> 1) - 1;
}

// --- Registerinit-Listen (PROGMEM spart RAM) ---
static const uint8_t I2CMode[]  PROGMEM = {4, 0x88,0x00, 0x80,0x01, 0xFF,0x01, 0x00,0x00};
static const uint8_t I2CMode2[] PROGMEM = {3, 0x00,0x01, 0xFF,0x00, 0x80,0x00};
static const uint8_t SPAD0[]    PROGMEM = {4, 0x80,0x01, 0xFF,0x01, 0x00,0x00, 0xFF,0x06};
static const uint8_t SPAD1[]    PROGMEM = {5, 0xFF,0x07, 0x81,0x01, 0x80,0x01, 0x94,0x6b, 0x83,0x00};
static const uint8_t SPAD2[]    PROGMEM = {4, 0xFF,0x01, 0x00,0x01, 0xFF,0x00, 0x80,0x00};
static const uint8_t SPAD[]     PROGMEM = {5, 0xFF,0x01, 0x4F,0x00, 0x4E,0x2C, 0xFF,0x00, 0xB6,0xB4};
static const uint8_t DefTuning[] PROGMEM = {
  80, 0xFF,0x01, 0x00,0x00, 0xFF,0x00, 0x09,0x00,
  0x10,0x00, 0x11,0x00, 0x24,0x01, 0x25,0xFF, 0x75,0x00, 0xFF,0x01, 0x4E,0x2C,
  0x48,0x00, 0x30,0x20, 0xFF,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03,
  0x40,0x83, 0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
  0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xA0,
  0xFF,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xFF, 0x4A,0x00, 0xFF,0x00, 0x7A,0x0A,
  0x7B,0x00, 0x78,0x21, 0xFF,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xFF, 0x45,0x26,
  0x46,0x05, 0x40,0x40, 0x0E,0x06, 0x20,0x1A, 0x43,0x40, 0xFF,0x00, 0x34,0x03,
  0x35,0x44, 0xFF,0x01, 0x31,0x04, 0x4B,0x09, 0x4C,0x05, 0x4D,0x04, 0xFF,0x00,
  0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
  0x72,0xFE, 0x76,0x00, 0x77,0x00, 0xFF,0x01, 0x0D,0x01, 0xFF,0x00, 0x80,0x01,
  0x01,0xF8, 0xFF,0x01, 0x8E,0x01, 0x00,0x01, 0xFF,0x00, 0x80,0x00
};

// Konstruktor: setzt Basiswerte, allerdings ohne Hardware zu initialisieren
ToFCtrl::ToFCtrl()
  : _w(&Wire), _addr(0x29), _xshut(-1), _ok(false), _debug(false),
    _maxCm(TOF_MAX_CM), _minCm(TOF_MIN_CM),
    _stopVar(0), _timingBudgetUs(0),
    _offsetMm(0), _lastMm(-1), _lastSuccessMs(0), _errorCount(0), _i2cErr(false) {}

// (Legacy) Kalibrier-Dateipfad entfernt – es wird nur noch Offset genutzt

// Initialisiert den Sensor. Optional XSHUT-Pin wird vor dem Init aktiviert.
bool ToFCtrl::begin(TwoWire &wire, uint8_t i2cAddr, int8_t xshutPin) {
  _w = &wire;
  _addr = i2cAddr;
  _xshut = xshutPin;
  // Falls XSHUT definiert ist, aktiviere den Sensor
  if (_xshut >= 0) {
    pinMode(_xshut, OUTPUT);
    digitalWrite(_xshut, HIGH);
    delay(3);
  }
  _ok = initSensor(false);
  return _ok;
}

bool ToFCtrl::reinit() {
  _ok = initSensor(false);
  return _ok;
}

void ToFCtrl::enable() {
  if (_xshut >= 0) {
    digitalWrite(_xshut, HIGH);
    delay(3);
  }
  reinit();
}

void ToFCtrl::disable() {
  if (_xshut >= 0) {
    digitalWrite(_xshut, LOW);
  }
  _ok = false;
}

bool ToFCtrl::getModelInfo(uint8_t &modelId, uint8_t &revisionId) {
  if (!_ok) return false;
  modelId    = r8(REG_IDENTIFICATION_MODEL_ID);
  revisionId = r8(REG_IDENTIFICATION_REVISION_ID);
  return true;
}

// --- Low-Level I2C Helferfunktionen ---
uint8_t ToFCtrl::r8(uint8_t reg) {
  if (_i2cErr) return 0; // Abbrechen bei vorherigem Fehler
  _w->beginTransmission(_addr);
  _w->write(reg);
  uint8_t err = _w->endTransmission(false);
  if (err != 0) {
    _i2cErr = true;
    _errorCount++;
    if (_debug) Serial.printf("[ToF] I2C r8 Error %d at reg 0x%02X\n", err, reg);
    return 0;
  }
  _w->requestFrom((int)_addr, 1);
  if (_w->available()) {
    _errorCount = 0;
    return _w->read();
  }
  _i2cErr = true; // Request failed
  _errorCount++;
  return 0;
}

uint16_t ToFCtrl::r16(uint8_t reg) {
  if (_i2cErr) return 0;
  _w->beginTransmission(_addr);
  _w->write(reg);
  uint8_t err = _w->endTransmission(false);
  if (err != 0) {
    _i2cErr = true;
    _errorCount++;
    if (_debug) Serial.printf("[ToF] I2C r16 Error %d at reg 0x%02X\n", err, reg);
    return 0;
  }
  _w->requestFrom((int)_addr, 2);
  uint16_t v = 0;
  if (_w->available() >= 2) {
    _errorCount = 0;
    v = ((uint16_t)_w->read() << 8) | _w->read();
  } else {
    _i2cErr = true;
    _errorCount++;
  }
  return v;
}

void ToFCtrl::rMulti(uint8_t reg, uint8_t *buf, int len) {
  if (_i2cErr) return;
  _w->beginTransmission(_addr);
  _w->write(reg);
  if (_w->endTransmission(false) != 0) { _i2cErr = true; return; }
  _w->requestFrom((int)_addr, len);
  for (int i = 0; i < len && _w->available(); ++i) {
    buf[i] = _w->read();
  }
}

void ToFCtrl::w8(uint8_t reg, uint8_t val) {
  if (_i2cErr) return;
  _w->beginTransmission(_addr);
  _w->write(reg);
  _w->write(val);
  if (_w->endTransmission() != 0) _i2cErr = true;
}

void ToFCtrl::w16(uint8_t reg, uint16_t val) {
  if (_i2cErr) return;
  _w->beginTransmission(_addr);
  _w->write(reg);
  _w->write((uint8_t)(val >> 8));
  _w->write((uint8_t)val);
  if (_w->endTransmission() != 0) _i2cErr = true;
}

void ToFCtrl::wMulti(uint8_t reg, const uint8_t *buf, int len) {
  if (_i2cErr) return;
  _w->beginTransmission(_addr);
  _w->write(reg);
  for (int i = 0; i < len; ++i) {
    _w->write(buf[i]);
  }
  if (_w->endTransmission() != 0) _i2cErr = true;
}

void ToFCtrl::wList(const uint8_t *list) {
  if (_i2cErr) return;
  uint8_t count = pgm_read_byte(list++);
  while (count--) {
    uint8_t reg = pgm_read_byte(list++);
    uint8_t val = pgm_read_byte(list++);
    w8(reg, val);
    if (_i2cErr) return;
  }
}

// --- Mathematische Hilfsfunktionen für Timeout-Umrechnung ---
uint16_t ToFCtrl::decodeTimeout(uint16_t reg_val) {
  return (uint16_t)((reg_val & 0xFF) << ((reg_val >> 8) & 0xFF)) + 1;
}

uint32_t ToFCtrl::timeoutMclksToUs(uint16_t mclks, uint8_t vcsel_pclks) {
  uint32_t macro_ns = calcMacroPeriod(vcsel_pclks);
  return ((uint32_t)mclks * macro_ns + macro_ns / 2) / 1000;
}

uint32_t ToFCtrl::timeoutUsToMclks(uint32_t us, uint8_t vcsel_pclks) {
  uint32_t macro_ns = calcMacroPeriod(vcsel_pclks);
  return ((us * 1000UL) + macro_ns / 2) / macro_ns;
}

uint16_t ToFCtrl::encodeTimeout(uint16_t mclks) {
  if (mclks == 0) return 0;
  uint32_t ls = mclks - 1;
  uint16_t ms = 0;
  while (ls & 0xFFFFFF00UL) {
    ls >>= 1;
    ms++;
  }
  return (ms << 8) | (ls & 0xFF);
}

// --- Sequenzzeitberechnung ---
void ToFCtrl::getSeqTimeouts(uint8_t enables, SeqTimeouts *t) {
  t->pre_range_vcsel_period_pclks = (r8(PRE_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1;
  t->msrc_dss_tcc_mclks = r8(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  t->msrc_dss_tcc_us    = timeoutMclksToUs(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_period_pclks);
  t->pre_range_mclks    = decodeTimeout(r16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  t->pre_range_us       = timeoutMclksToUs(t->pre_range_mclks, t->pre_range_vcsel_period_pclks);
  t->final_range_vcsel_period_pclks = (r8(FINAL_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1;
  t->final_range_mclks  = decodeTimeout(r16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (enables & 0x40 /* SEQUENCE_ENABLE_PRE_RANGE */) {
    t->final_range_mclks -= t->pre_range_mclks;
  }
  t->final_range_us = timeoutMclksToUs(t->final_range_mclks, t->final_range_vcsel_period_pclks);
}

bool ToFCtrl::setVcselPulsePeriod(VcselType type, uint8_t period_pclks) {
  uint8_t vcsel_reg = encodeVcselPeriod(period_pclks);
  uint8_t enables = r8(SYSTEM_SEQUENCE_CONFIG);
  SeqTimeouts t; getSeqTimeouts(enables, &t);
  if (type == VcselPreRange) {
    switch (period_pclks) {
      case 12: w8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
      case 14: w8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
      case 16: w8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
      case 18: w8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
      default: return false;
    }
    w8(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    w8(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);
    uint16_t new_pre_range_timeout_mclks = timeoutUsToMclks(t.pre_range_us, period_pclks);
    w16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
    uint16_t new_msrc_timeout_mclks = timeoutUsToMclks(t.msrc_dss_tcc_us, period_pclks);
    w8(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (uint8_t)(new_msrc_timeout_mclks - 1));
  } else {
    switch (period_pclks) {
      case 8:
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        w8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        w8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        w8(0xFF, 0x01); w8(ALGO_PHASECAL_LIM, 0x30); w8(0xFF, 0x00);
        break;
      case 10:
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        w8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        w8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        w8(0xFF, 0x01); w8(ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
        break;
      case 12:
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        w8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        w8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        w8(0xFF, 0x01); w8(ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
        break;
      case 14:
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        w8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        w8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        w8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        w8(0xFF, 0x01); w8(ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
        break;
      default: return false;
    }
    w8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_reg);
    uint16_t new_final_range_timeout_mclks = timeoutUsToMclks(t.final_range_us, period_pclks);
    if (enables & 0x40) new_final_range_timeout_mclks += t.pre_range_mclks;
    w16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
  }
  setMeasurementTimingBudget(_timingBudgetUs);
  uint8_t seq = r8(SYSTEM_SEQUENCE_CONFIG);
  w8(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x00);
  w8(SYSTEM_SEQUENCE_CONFIG, seq);
  return true;
}

bool ToFCtrl::getSpadInfo(uint8_t *count, uint8_t *typeIsAperture) {
  wList(SPAD0);
  w8(0x83, r8(0x83) | 0x04);
  wList(SPAD1);
  uint32_t tStart = millis();
  while (r8(0x83) == 0x00) {
    if (millis() - tStart > 250) return false;
    delay(5);
  }
  w8(0x83, 0x01);
  uint8_t t = r8(0x92);
  *count = (t & 0x7F);
  *typeIsAperture = (t & 0x80);
  w8(0x81, 0x00);
  w8(0xFF, 0x06);
  w8(0x83, (uint8_t)(r8(0x83) & ~0x04));
  wList(SPAD2);
  return true;
}

// --- Hauptinitialisierung ---
bool ToFCtrl::initSensor(bool longRangeMode) {
  _i2cErr = false; // Reset error flag before init

  // Externe Stromquelle 2.8V aktivieren
  w8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, (uint8_t)(r8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01));
  if (_i2cErr) return false; // Early exit on first I2C error

  // I2C‑Modus programmieren
  wList(I2CMode);
  _stopVar = r8(0x91);
  wList(I2CMode2);
  if (_i2cErr) return false;

  // MSRC/Pre-Range Limits deaktivieren, Min Count Rate setzen
  w8(REG_MSRC_CONFIG_CONTROL, (uint8_t)(r8(REG_MSRC_CONFIG_CONTROL) | 0x12));
  w16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32); // 0.25 in Q9.7
  w8(SYSTEM_SEQUENCE_CONFIG, 0xFF);
  
  if (_i2cErr) return false;

  // SPAD-Konfiguration
  uint8_t spadCount = 0, isAp = 0;
  if (!getSpadInfo(&spadCount, &isAp)) return false;
  if (_i2cErr) return false;

  uint8_t spadMap[6]; rMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spadMap, 6);
  wList(SPAD);
  uint8_t first = isAp ? 12 : 0;
  uint8_t enabled = 0;
  for (int i = 0; i < 48; ++i) {
    if (i < first || enabled == spadCount) {
      spadMap[i >> 3] &= ~(1 << (i & 7));
    } else if (spadMap[i >> 3] & (1 << (i & 7))) {
      enabled++;
    }
  }
  wMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spadMap, 6);
  // Standard-Tuning laden
  wList(DefTuning);
  
  if (_i2cErr) return false;

  // optional: Long-Range-Modus
  if (longRangeMode) {
    w16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13);
    setVcselPulsePeriod(VcselPreRange, 18);
    setVcselPulsePeriod(VcselFinalRange, 14);
  }
  // Interrupt-Konfiguration
  w8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  w8(GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(r8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10));
  w8(SYSTEM_INTERRUPT_CLEAR, 0x01);
  
  if (_i2cErr) return false;

  // Default-Messbudget für bessere Genauigkeit im Nahbereich erhöhen
  // (z. B. 50 ms → weniger Rauschen als Standard)
  // UPDATE: High Accuracy Mode = 200 ms (200000 us)
  (void)getMeasurementTimingBudget();
  const uint32_t HighAccuracyBudgetUs = 200000; // 200 ms für hohe Genauigkeit
  w8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  setMeasurementTimingBudget(HighAccuracyBudgetUs);
  w8(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) return false;
  w8(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) return false;
  w8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
  
  if (_i2cErr) return false;

  // Starte kontinuierliche Messung im Hintergrund (Non-Blocking)
  startContinuous();
  return !_i2cErr;
}

bool ToFCtrl::performSingleRefCalibration(uint8_t vhv_init_byte) {
  w8(SYSRANGE_START, (uint8_t)(0x01 | vhv_init_byte));
  uint32_t tStart = millis();
  while ((r8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (millis() - tStart > 500) return false;
    delay(5);
  }
  w8(SYSTEM_INTERRUPT_CLEAR, 0x01);
  w8(SYSRANGE_START, 0x00);
  return true;
}

bool ToFCtrl::setMeasurementTimingBudget(uint32_t budget_us) {
  const uint16_t StartOverhead = 1320, EndOverhead = 960, MsrcOverhead = 660,
                 TccOverhead = 590, DssOverhead = 690, PreOverhead = 660,
                 FinalOverhead = 550;
  const uint32_t MinBudget = 20000;
  if (budget_us < MinBudget) return false;
  uint32_t used = StartOverhead + EndOverhead;
  uint8_t enables = r8(SYSTEM_SEQUENCE_CONFIG);
  SeqTimeouts t; getSeqTimeouts(enables, &t);
  if (enables & 0x10) used += t.msrc_dss_tcc_us + TccOverhead;
  if (enables & 0x08) used += 2 * (t.msrc_dss_tcc_us + DssOverhead);
  else if (enables & 0x04) used += t.msrc_dss_tcc_us + MsrcOverhead;
  if (enables & 0x40) used += t.pre_range_us + PreOverhead;
  if (enables & 0x80) {
    used += FinalOverhead;
    if (used > budget_us) return false;
    uint32_t final_us = budget_us - used;
    uint16_t final_mclks = timeoutUsToMclks(final_us, t.final_range_vcsel_period_pclks);
    if (enables & 0x40) final_mclks += t.pre_range_mclks;
    w16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_mclks));
    _timingBudgetUs = budget_us;
  }
  return true;
}

uint32_t ToFCtrl::getMeasurementTimingBudget() {
  const uint16_t StartOverhead = 1910, EndOverhead = 960, MsrcOverhead = 660,
                 TccOverhead = 590, DssOverhead = 690, PreOverhead = 660,
                 FinalOverhead = 550;
  uint32_t budget = StartOverhead + EndOverhead;
  uint8_t enables = r8(SYSTEM_SEQUENCE_CONFIG);
  SeqTimeouts t; getSeqTimeouts(enables, &t);
  if (enables & 0x10) budget += t.msrc_dss_tcc_us + TccOverhead;
  if (enables & 0x08) budget += 2 * (t.msrc_dss_tcc_us + DssOverhead);
  else if (enables & 0x04) budget += t.msrc_dss_tcc_us + MsrcOverhead;
  if (enables & 0x40) budget += t.pre_range_us + PreOverhead;
  if (enables & 0x80) budget += t.final_range_us + FinalOverhead;
  _timingBudgetUs = budget;
  return budget;
}

// Startet den Continuous Back-to-Back Modus
void ToFCtrl::startContinuous() {
  w8(0x80, 0x01);
  w8(0xFF, 0x01);
  w8(0x00, 0x00);
  w8(0x91, _stopVar);
  w8(0x00, 0x01);
  w8(0xFF, 0x00);
  w8(0x80, 0x00);
  w8(SYSRANGE_START, 0x02); // 0x02 = VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
}

// Abfrage des aktuellen Messwerts (Non-Blocking)
// Prüft, ob der Sensor fertig ist. Wenn ja, Update _lastMm.
// Wenn nein, gib den letzten bekannten Wert zurück.
int ToFCtrl::readRawMm() {
  if (!_ok) {
    // Falls nicht OK, versuche alle 5 Sekunden ein Reinit
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      if (_debug) Serial.println("[ToF] Versuche Reinit da _ok=false...");
      reinit();
    }
    return -1;
  }

  // Recovery-Logik: Falls zu viele I2C-Fehler oder Sensor-Timeout (> 15s)
  if (_errorCount > 10 || (millis() - _lastSuccessMs > 15000 && _lastSuccessMs > 0)) {
    if (_debug) Serial.printf("[ToF] Recovery! Errors: %d, LastSuccess: %lu ms ago\n", _errorCount, millis() - _lastSuccessMs);
    _errorCount = 0;
    _lastSuccessMs = millis(); // Reset Timer für nächsten Versuch
    reinit();
    return -1;
  }

  uint8_t status = r8(RESULT_INTERRUPT_STATUS);
  
  // Prüfen auf Error-Bit (Bit 3) im Status
  if (status & 0x08) {
    if (_debug) Serial.println("[ToF] Sensor meldet Hardware-Error Bit!");
    w8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    return -1;
  }

  // Prüfen, ob neue Daten bereitstehen (Bit 0-2)
  if (status & 0x07) {
    // Wert lesen
    uint16_t range = r16(RESULT_RANGE_STATUS + 10);
    // Interrupt clearen, damit nächste Messung starten kann
    w8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Rohwert speichern (noch ohne Offset)
    int mm = (int)range;
    
    // 8190/8191 mm ist beim VL53L0X oft "out of range" oder "signal fail"
    if (range > 8000) {
       _lastMm = (int)_maxCm * 10 + 100; // Als out of range markieren
    } else {
      // Software-Offset anwenden
      mm -= _offsetMm;
      _lastMm = mm;
      _lastSuccessMs = millis(); // Erfolg markieren
    }
  }

  // Mit dem letzten bekannten Wert arbeiten
  int mm = _lastMm;

  // Initialwert abfangen
  if (mm < 0 && _lastSuccessMs == 0) return -1;

  // Prüfen auf Unterschreitung der minimalen Messdistanz.
  if (_minCm > 0) {
    int minMm = (int)_minCm * 10;
    if (mm < minMm) return -2; // zu nah
  }

  // Prüfen auf Überschreitung der maximalen Messdistanz
  int maxMm = (int)_maxCm * 10;
  if (mm > maxMm) return -1;

  return mm;
}

// Durchschnitt mehrerer Messungen (optimiert für Geschwindigkeit)
int ToFCtrl::readAvgMm(uint8_t samples) {
  if (samples == 0) return readRawMm();
  // Im High Accuracy Continuous Mode ist samples=1 empfohlen (via Config).
  // Falls samples > 1, wird hier schnell der gleiche Wert gelesen, was okay ist.
  // Robustere Mittelung: Trimmed Mean (schneidet Ausreißer ab)
  const uint8_t N = (samples > 32) ? 32 : samples;
  int vals[32];
  uint8_t n = 0;
  for (uint8_t i = 0; i < N; ++i) {
    int v = readRawMm();
    if (v >= 0) {
      vals[n++] = v;
    }
    delay(2);
  }
  if (n == 0) return -1;
  // Sortieren (kleines n)
  for (uint8_t i = 1; i < n; ++i) {
    int key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) { vals[j+1] = vals[j]; j--; }
    vals[j+1] = key;
  }
  uint8_t trim = (n >= 10) ? (n / 5) : 0; // 20% trim ab n>=10
  uint8_t start = trim;
  uint8_t end = (n > trim) ? (uint8_t)(n - trim) : n;
  if (end <= start) { start = 0; end = n; }
  long sum = 0; uint8_t cnt = 0;
  for (uint8_t i = start; i < end; ++i) { sum += vals[i]; cnt++; }
  if (cnt == 0) return vals[n/2];
  return (int)((sum + (cnt/2)) / cnt);
}
