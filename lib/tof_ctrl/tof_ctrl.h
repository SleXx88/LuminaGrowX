// tof_ctrl.h
//
// Eine einfache Arduino-kompatible Bibliothek zur Ansteuerung des VL53L0X‑Time‑of‑Flight‑Sensors.
// Die Schnittstelle orientiert sich am Stil der stepper_ctrl‑Bibliothek: eine kleine Klasse
// kapselt die Initialisierung, Messung und optional die Steuerung über den XSHUT‑Pin.
//
// Konfiguration:
//   – Standard‑Adresse: 0x29 (definiert im Beispiel-Code in main.cpp)
//   – Maximale Messweite in Zentimetern kann via TOF_MAX_CM vorcompiliert oder zur Laufzeit
//     mit setMaxRangeCm() gesetzt werden. Werte oberhalb des sinnvollen Bereichs werden als
//     „out of range“ gemeldet (Rückgabewert = -1).
//   – Für durchschnittliche Messungen wird eine definierbare Anzahl Samples gemittelt.
//
// Verwendung:
//   #include "tof_ctrl.h"
//   ToFCtrl tof;
//   bool ok = tof.begin(Wire, 0x29, xshutPin);
//   int distanceMm = tof.readRawMm();
//   int avg = tof.readAvgMm();
//   …

#pragma once

#include <Arduino.h>
#include <Wire.h>

// ===== Konfiguration =====
// Maximale sinnvolle Messreichweite in Zentimetern. Der Sensor kann zwar weiter messen,
// allerdings werden die Werte jenseits dieses Bereichs als unzuverlässig behandelt.
#ifndef TOF_MAX_CM
#define TOF_MAX_CM 110
#endif

// Minimale sinnvolle Messdistanz in Zentimetern. Der VL53L0X liefert unterhalb
// etwa 3 cm (30 mm) keine verlässlichen Messwerte. Messungen kleiner als
// dieser Wert werden als „zu nah“ interpretiert. Über setMinRangeCm() kann
// der Bereich zur Laufzeit geändert werden (z.B. auf 0, um keine Unterschwelle
// zu prüfen).
#ifndef TOF_MIN_CM
#define TOF_MIN_CM 3
#endif

// Standardanzahl Samples für gemittelte Messung (readAvgMm)
#ifndef TOF_DEFAULT_AVG_SAMPLES
#define TOF_DEFAULT_AVG_SAMPLES 10
#endif

class ToFCtrl {
public:
  ToFCtrl();

  // Initialisiert den Sensor. Der optionale XSHUT‑Pin wird vor der Initialisierung
  // aktiviert (HIGH) und kann zum Ein-/Ausschalten des Sensors verwendet werden.
  // @wire: Referenz auf das I²C-Objekt (z.B. Wire)
  // @i2cAddr: I²C‑Adresse des VL53L0X (default 0x29)
  // @xshutPin: optionaler GPIO für XSHUT (LOW = aus, HIGH = an). -1, wenn nicht genutzt.
  // @return true, wenn die Initialisierung erfolgreich war.
  bool begin(TwoWire &wire = Wire, uint8_t i2cAddr = 0x29, int8_t xshutPin = -1);

  // Setzt den Sensor nach einem Hard-Reset neu auf (enthält die Init-Sequenz erneut).
  // Wird automatisch von enable() aufgerufen, wenn der Sensor über XSHUT wieder eingeschaltet wird.
  // @return true, wenn das Reinit erfolgreich war.
  bool reinit();

  // Schaltet den Sensor ein (setzt XSHUT HIGH) und initialisiert ihn.
  void enable();

  // Schaltet den Sensor aus (setzt XSHUT LOW). Nach disable() ist readRawMm() nicht funktionsfähig
  // und begin()/enable() muss erneut ausgeführt werden.
  void disable();

  // Prüft, ob der Sensor erfolgreich initialisiert wurde. Liefert false nach einem Fehler
  // oder nach disable().
  bool isOk() const { return _ok; }

  // Setzt die maximale Messreichweite in Zentimetern. Bei Messungen > maxCm wird -1 gemeldet.
  void setMaxRangeCm(uint16_t cm) { _maxCm = cm; }

  // Setzt die minimale Messdistanz in Zentimetern. Bei Messungen < minCm wird -2
  // zurückgegeben. Standard ist TOF_MIN_CM (3 cm). Wird dieser Wert 0 gesetzt,
  // entfällt die Unterschwellenprüfung.
  void setMinRangeCm(uint16_t cm) { _minCm = cm; }

  // Liest eine einzelne Messung in Millimetern. Rückgabe:
  //   >= 0  : Distanz in mm innerhalb des festgelegten Bereichs
  //    -1   : Sensorfehler oder Distanz jenseits der maximalen Reichweite
  //    -2   : Distanz liegt unterhalb der minimalen Messdistanz (zu nah)
  int readRawMm();

  // Liest mehrere Messungen und bildet den Durchschnitt. Samples = 0 nutzt readRawMm()
  // direkt. Wird ein Fehler oder Out-of-Range gemessen, werden diese Messungen
  // bei der Mittelwertbildung ignoriert. Wenn alle Messungen ungültig sind, wird -1 zurückgegeben.
  int readAvgMm(uint8_t samples = TOF_DEFAULT_AVG_SAMPLES);

  // Optional: liest Modell- und Revisions-ID des Sensors. Liefert false, wenn nicht initialisiert.
  bool getModelInfo(uint8_t &modelId, uint8_t &revisionId);

private:
  // --- Helferfunktionen zum Lesen/Schreiben über I²C ---
  uint8_t  r8(uint8_t reg);
  uint16_t r16(uint8_t reg);
  void     rMulti(uint8_t reg, uint8_t *buf, int len);
  void     w8(uint8_t reg, uint8_t val);
  void     w16(uint8_t reg, uint16_t val);
  void     wMulti(uint8_t reg, const uint8_t *buf, int len);
  void     wList(const uint8_t *list);

  // --- Portierung interner VL53L0X-Routinen ---
  bool initSensor(bool longRangeMode = false);
  bool performSingleRefCalibration(uint8_t vhv_init_byte);
  bool setMeasurementTimingBudget(uint32_t budget_us);
  uint32_t getMeasurementTimingBudget();
  int readRangeContinuousMillimeters();

  struct SeqTimeouts {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
  };
  enum VcselType { VcselPreRange, VcselFinalRange };
  bool setVcselPulsePeriod(VcselType type, uint8_t period_pclks);
  void getSeqTimeouts(uint8_t enables, SeqTimeouts *t);
  static uint16_t decodeTimeout(uint16_t reg_val);
  static uint32_t timeoutMclksToUs(uint16_t mclks, uint8_t vcsel_pclks);
  static uint32_t timeoutUsToMclks(uint32_t us, uint8_t vcsel_pclks);
  static uint16_t encodeTimeout(uint16_t mclks);
  bool getSpadInfo(uint8_t *count, uint8_t *typeIsAperture);

  // --- Registerkonstanten ---
  static constexpr uint8_t REG_IDENTIFICATION_MODEL_ID    = 0xC0;
  static constexpr uint8_t REG_IDENTIFICATION_REVISION_ID = 0xC2;
  static constexpr uint8_t REG_SYSRANGE_START             = 0x00;
  static constexpr uint8_t RESULT_INTERRUPT_STATUS        = 0x13;
  static constexpr uint8_t RESULT_RANGE_STATUS            = 0x14;
  static constexpr uint8_t ALGO_PHASECAL_LIM              = 0x30;
  static constexpr uint8_t ALGO_PHASECAL_CONFIG_TIMEOUT   = 0x30;
  static constexpr uint8_t GLOBAL_CONFIG_VCSEL_WIDTH      = 0x32;
  static constexpr uint8_t FINAL_RANGE_CONFIG_VALID_PHASE_LOW  = 0x47;
  static constexpr uint8_t FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
  static constexpr uint8_t PRE_RANGE_CONFIG_VCSEL_PERIOD  = 0x50;
  static constexpr uint8_t PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  = 0x51;
  static constexpr uint8_t PRE_RANGE_CONFIG_VALID_PHASE_LOW   = 0x56;
  static constexpr uint8_t PRE_RANGE_CONFIG_VALID_PHASE_HIGH  = 0x57;
  static constexpr uint8_t REG_MSRC_CONFIG_CONTROL        = 0x60;
  static constexpr uint8_t FINAL_RANGE_CONFIG_VCSEL_PERIOD     = 0x70;
  static constexpr uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI= 0x71;
  static constexpr uint8_t MSRC_CONFIG_TIMEOUT_MACROP     = 0x46;
  static constexpr uint8_t FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
  static constexpr uint8_t SYSRANGE_START                 = 0x00;
  static constexpr uint8_t SYSTEM_SEQUENCE_CONFIG         = 0x01;
  static constexpr uint8_t SYSTEM_INTERRUPT_CONFIG_GPIO   = 0x0A;
  static constexpr uint8_t VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
  static constexpr uint8_t GLOBAL_CONFIG_SPAD_ENABLES_REF_0  = 0xB0;
  static constexpr uint8_t GPIO_HV_MUX_ACTIVE_HIGH        = 0x84;
  static constexpr uint8_t SYSTEM_INTERRUPT_CLEAR         = 0x0B;

  // --- interner Zustand ---
  TwoWire* _w;         // Verweis auf das verwendete I²C-Objekt
  uint8_t  _addr;      // I²C‑Adresse des Sensors
  int8_t   _xshut;     // GPIO für XSHUT; -1, wenn unbenutzt
  bool     _ok;        // Status der Initialisierung
  uint16_t _maxCm;     // maximale Messdistanz in cm
  uint16_t _minCm;     // minimale Messdistanz in cm
  uint8_t  _stopVar;   // interner Wert aus Sensor-Init
  uint32_t _timingBudgetUs; // gespeichertes Timing-Budget
};