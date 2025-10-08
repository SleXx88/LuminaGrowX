#include <Arduino.h>
#include <Wire.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "stepper_ctrl.h"
#include "tof_ctrl.h"
#include "vpd_calc.h"
#include "env_ctrl.h"
#include "rtc_ctrl.h"

#include <LittleFS.h>

// ===================== Test-Stepper =====================
// Auf true setzen, um den vollständigen Stepper‑Selbsttest in setup() auszuführen
// #define RUN_STEPPER_SELFTEST true

// VPD
using namespace vpd_calc;
// Umweltsteuerung
using namespace env_ctrl;

// Definiere das aktuelle Wachstumsstadium für die Klassifizierung.
static GrowthStage CURRENT_STAGE = GrowthStage::Vegetative;

//  Fan-Pins definieren
constexpr uint8_t FAN_PWM_PIN = 2;

//  I2C-Pins definieren
constexpr int I2C_SDA_1 = 8;  // SDA des ESP32‑S3
constexpr int I2C_SCL_1 = 9;  // SCL des ESP32‑S3
constexpr int I2C_SDA_2 = 20;
constexpr int I2C_SCL_2 = 19;
constexpr uint32_t I2C_FREQ_2 = 400000; // 400 kHz

// ToF XSHUT-Pin definieren
constexpr int PIN_XSHUT = 4; // XSHUT‑Pin des VL53L0X (optional)

//  Sensoren und Aktoren initialisieren
SHT41Ctrl sht_in;      // Adresse 0x44 (fix)
SHT41Ctrl sht_out;  // Adresse 0x44 (fix)
GP8211Ctrl dac;     // Adresse 0x58 (fix)
FanCtrl fan;        // Lüftersteuerung
ToFCtrl tof;        // Time-of-Flight Distanzsensor
StepperCtrl step;   // Schrittmotorsteuerung
EnvCtrl controller; // Umweltsteuerung
RTC_Ctrl rtc;       // RTC

// ===== Hilfsfunktionen für den Stepper‑Selbsttest =====
static inline bool approxEq(float a, float b, float eps = 0.02f) {
  return fabsf(a - b) <= eps;
}

static void waitUntilStill(StepperCtrl& s, unsigned long stable_ms = 250, unsigned long timeout_ms = 60000) {
  const float eps = 0.01f; // 0.01 mm als Stillstands-Schwelle
  unsigned long t0 = millis();
  float lastPos = s.getPositionMm();
  unsigned long stableSince = millis();

  for (;;) {
    s.tick();
    float p = s.getPositionMm();
    if (fabsf(p - lastPos) <= eps) {
      if (millis() - stableSince >= stable_ms) break; // still
    } else {
      stableSince = millis();
      lastPos = p;
    }

    if (millis() - t0 > timeout_ms) {
      Serial.println(F("[TEST] Timeout beim Warten auf Stillstand!"));
      break;
    }
    delay(1);
  }
}

static void printPos(StepperCtrl& s, const char* tag) {
  Serial.printf("[POS] %s: %.2f mm\n", tag, s.getPositionMm());
}

static void runStepperSelfTest() {
  Serial.println();
  Serial.println(F("=== Stepper Selbsttest START ==="));

  // Optional: Achsensinn + Weglänge setzen (falls nicht schon in begin() festgelegt)
  step.setAxisUpDir(+1);         // forward = nach OBEN; +mm (logisch) = nach UNTEN
  step.setMaxTravelMm(440.0f);   // Arbeitsweg nach unten (an Mechanik anpassen)

  step.enableDebug(true);
  step.setDebugMoveLogInterval(100); // 100 ms Logintervall

  // HOMING (nach oben). Backoff setzt 0.00 mm als Arbeits-Nullpunkt.
  Serial.println(F("[TEST] HOMING start"));
  while (!step.home()) { step.tick(); }
  Serial.println(F("[TEST] HOMING done"));
  printPos(step, "nach Homing (erwartet ~0.00)");

  // --- Test 1: Abwärts auf 20.00 mm ---------------------------------------
  // Serial.println(F("[TEST] moveTo(20.00 mm) – abwärts"));
  // step.moveTo(20.0f, 5);   // Level 2 wie in deinen Logs
  // waitUntilStill(step);
  // printPos(step, "soll ~20.00 mm");

  // --- Test 2: Zurück nach oben auf 0.00 mm --------------------------------
  Serial.println(F("[TEST] moveTo(0.00 mm) – aufwärts"));
  step.moveTo(0.0f, 5);
  waitUntilStill(step);
  printPos(step, "soll ~0.00 mm");

  // --- Test 3: Softlimit OBERES Ende (über 0 hinaus) -----------------------
  Serial.println(F("[TEST] Softlimit oben: Versuch -5.00 mm (über 0 hinaus)"));
  step.moveTo(-5.0f, 5);        // ungültig, soll auf 0.00 mm klemmen
  waitUntilStill(step);
  printPos(step, "clamp oben, erwartet ~0.00 mm");

  // --- Test 4: Softlimit UNTERES Ende (unter softMin hinaus) ---------------
  Serial.println(F("[TEST] Softlimit unten: Versuch 1e6 mm (weit unter softMin)"));
  step.moveTo(1e6f, 5);         // absichtlich zu groß → Clamp auf softMin
  waitUntilStill(step);
  printPos(step, "clamp unten, erwartet ~softMin (negativ)");

  // Zurück in sicheren Bereich
  Serial.println(F("[TEST] zurück in sicheren Bereich: moveTo(10.00 mm)"));
  step.moveTo(10.0f, 5);
  waitUntilStill(step);
  printPos(step, "soll ~10.00 mm");

  Serial.println(F("=== Stepper Selbsttest ENDE ==="));
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Gemeinsamer I²C-Bus für mehrere ICs:
  Wire.begin(I2C_SDA_1, I2C_SCL_1, 400000);
  Wire1.begin(I2C_SDA_2, I2C_SCL_2, I2C_FREQ_2);

  //  Stepper initialisieren
  step.begin();
  // step.setAxisUpDir(-1);
  step.enableDebug(true);            // Debug an
  step.setDebugMoveLogInterval(100); // alle 100 ms Logs

  //  Homing beim Start:
  while (!step.home()) { step.tick(); }

  //  Sensor einschalten und initialisieren
  if (PIN_XSHUT >= 0) {
    pinMode(PIN_XSHUT, OUTPUT);
    digitalWrite(PIN_XSHUT, HIGH);
    delay(3);
  }
  bool ok = tof.begin(Wire, 0x29, PIN_XSHUT);
  Serial.println(ok ? F("[ToF] Init OK") : F("[ToF] Init FAILED"));
  if (ok) {
    uint8_t model = 0, rev = 0;
    if (tof.getModelInfo(model, rev)) {
      Serial.print(F("[ToF] Model ID: "));
      Serial.print(model);
      Serial.print(F("  Revision: "));
      Serial.println(rev);
    }
  }

  // LittleFS starten (für Kalibrierdaten)
  if (!LittleFS.begin(true)) {
    Serial.println(F("[FS] LittleFS mount/format FAILED"));
  } else {
    Serial.println(F("[FS] LittleFS bereit"));
  }

  // Offset laden oder interaktiv (100 mm) kalibrieren
  {
    File f = LittleFS.open("/tof_offset.dat", FILE_READ);
    if (f) {
      String s = f.readString();
      f.close();
      int val = 0; int n = 0;
      if (s.length() > 0) n = sscanf(s.c_str(), "%d", &val);
      if (n == 1) {
        tof.setOffsetMm((int16_t)val);
        Serial.printf("[CAL] Offset geladen: %d mm\n", val);
      }
    } else {
      Serial.println(F("[CAL] Kein Offset gefunden – Offset-Kalibrierung jetzt: 50 mm Target halten, dann 'o' senden."));
      while (!Serial.available()) { delay(5); }
      while (Serial.available()) { char c = Serial.read(); if (c=='o'||c=='O') break; }
      int measured = tof.readAvgMm(20);
      if (measured >= 0) {
        int target = 50; int off = measured - target; // mm -= off
        tof.setOffsetMm((int16_t)off);
        File fw = LittleFS.open("/tof_offset.dat", FILE_WRITE);
        if (fw) { fw.printf("%d\n", off); fw.close(); }
        Serial.printf("[CAL] Offset gesetzt: measured=%d target=%d offset=%d mm\n", measured, target, off);
      } else {
        Serial.println(F("[CAL] Messung fehlgeschlagen – Offset=0."));
        tof.setOffsetMm(0);
      }
    }
  }

  // RTC initialisieren
  if (!rtc.begin(Wire1)) {
    Serial.println("[RTC] Keine Verbindung zur DS3231 (0x57).");
  } else {
    Serial.println("[RTC] Init OK (DS3231 gefunden).");
    // bool okTime = rtc.writeTimeFromString("18-09-2025 22-30-00");
    // Serial.printf("[RTC] Setzen: %s\n", okTime ? "OK" : "FEHLER");
  }

  // Fan konfigurieren
  FanCtrl::Config cfg;
  cfg.pwmPin = FAN_PWM_PIN;
  cfg.ledcChannel = 0;
  cfg.pwmFreq = 25000; // 25 kHz
  cfg.resolutionBits = 8;
  cfg.invert = false;

  // Fan initialisieren
  fan.begin(cfg);
  fan.setPercent(50);

  // GP8211 initialisieren
  if (!dac.begin(Wire1)) {
    Serial.println("[GP8211] init FAIL (I2C addr 0x58)");
  } else {
    Serial.println("[GP8211] init OK (10V-range set)");
  }
  dac.setPercent(10); // 2 V bei 10 V-Range

  // SHT41 in initialisieren
  Serial.println(F("[SHT41_in] Init..."));
  if (!sht_in.begin(Wire1, /*softReset=*/true)) {
    Serial.println(F("[SHT41_in] Init FAILED (Sensor nicht erreichbar?)"));
  } else {
    Serial.println(F("[SHT41_in] Init OK"));
  }

  // SHT41 out initialisieren
  Serial.println(F("[SHT41_out] Init..."));
  if (!sht_out.begin(Wire, /*softReset=*/true)) {
    Serial.println(F("[SHT41_out] Init FAILED (Sensor nicht erreichbar?)"));
  } else {
    Serial.println(F("[SHT41_out] Init OK"));
  }

  // Hardware mit Controller verbinden
  controller.begin(sht_in, sht_out, dac, fan);
  controller.setStage(vpd_calc::GrowthStage::Vegetative);
  controller.setMode(DayMode::Day);

  // Schlanke Regel-Parameter (Defaults sind brauchbar)
  controller.setKpFan(18.0f);
  controller.setDeadband(0.06f);
  controller.setRateLimit(6.0f);
  controller.setSmoothingAlpha(0.25f);

  // Sicherheits-/Robustheits-Parameter
  controller.setMaxTemperature(30.0f);
  controller.setOverTempReduction(5.0f);
  controller.setTempHighFanBoost(31.0f, 15.0f);
  controller.setHumidityOverride(85.0f, 1.5f, true, 20.0f);
  controller.setDoorDetection(8.0f, 2.0f, 15000);
  controller.setModeChangeHold(8000);

  Serial.println(F("EnvCtrl demo startet"));

#if RUN_STEPPER_SELFTEST
  runStepperSelfTest();
#endif

  // Falls du ohne Selbsttest direkt fahren willst:
  // step.moveTo(20.0f, 5);
}

void loop()
{
  step.tick();

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  if (now - lastUpdate >= 5000) {
    lastUpdate = now;
    if (controller.update()) {
      String t = rtc.readTimeString();  // Lese die aktuelle Zeit
      Serial.print(t);                  // Ausgabe der Zeit

      float tC_in = NAN, tC_out = NAN, rh_in = NAN, rh_out = NAN;
      if (sht_in.read(tC_in, rh_in) && sht_out.read(tC_out, rh_out)) {
        // Indoor Werte
        Serial.print(F(" | [INDOOR] Temp: "));
        Serial.print(tC_in, 1);
        Serial.print(F(" C, RH: "));
        Serial.print(rh_in, 1);
        Serial.print(F(" %, Taupunkt: "));
        Serial.print(computeDewPoint(static_cast<double>(tC_in), static_cast<double>(rh_in)), 1);
        Serial.print(F(" C, VPD: "));
        Serial.print(computeVpd(static_cast<double>(tC_in), static_cast<double>(rh_in)), 3);
        Serial.print(F(" kPa"));

        // Outdoor Werte
        Serial.print(F(" | [OUTDOOR] Temp: "));
        Serial.print(tC_out, 1);
        Serial.print(F(" C, RH: "));
        Serial.print(rh_out, 1);
        Serial.print(F(" % | LED: "));
        Serial.print(controller.currentLedPercent(), 1);
        Serial.print(F(" % | Fan: "));
        Serial.print(controller.currentFanPercent(), 1);

        // ToF Entfernung ausgeben
        // Bevorzugt kalibr. Messung (fällt auf Rohwert zurück)
        int distance = tof.readAvgMm(10);
        Serial.print(F(" % | ToF: "));
        if (distance >= 0) {
          Serial.print(distance);
          Serial.print(F(" mm"));
        } else {
          Serial.print(F("out of range - "));
          Serial.print(distance);
          Serial.print(F(" mm"));
        }

        Serial.println();
      } else {
        Serial.println(F(" [SHT41] Read FAILED"));
      }
    } else {
      Serial.println(F("Sensor read failed"));
    }
  }
}


  //step.moveTo(20.0f, 2);
  // step.tick();
 
  // static uint32_t tPrev = 0;
  // uint32_t now = millis();
  // if (now - tPrev > 1000)
  // {
  //   tPrev = now;
  //   Serial.printf("[FAN] %5.1f %%\n", fan.getPercent());

  //   float tC_in = NAN, rh_in = NAN;
  //   if (sht_in.read(tC_in, rh_in))
  //   {
  //     Serial.print(F("[SHT41] T="));
  //     Serial.print(tC_in, 2);
  //     Serial.print(F(" °C  RH="));
  //     Serial.print(rh_in, 2);
  //     Serial.println(F(" %"));
  //   }
  //   else
  //   {
  //     Serial.println(F("[SHT41] Read FAILED"));
  //   }

  //   // Berechne VPD
  //   double vpd = computeVpd(static_cast<double>(tC_in), static_cast<double>(rh_in));
  //   // Klassifiziere VPD
  //   const char* status = classifyVpd(vpd, CURRENT_STAGE);

  //   // Berechne den Taupunkt zur Information
  //   double dewPoint = computeDewPoint(static_cast<double>(tC_in), static_cast<double>(rh_in));

  //   // Ausgabe der Ergebnisse
  //   Serial.print("Temp: ");
  //   Serial.print(tC_in);
  //   Serial.print(" °C, Luftfeuchtigkeit: ");
  //   Serial.print(rh_in);
  //   Serial.print(" %, VPD: ");
  //   Serial.print(vpd, 3);
  //   Serial.print(" kPa, Status: ");
  //   Serial.print(status);
  //   Serial.print(", Taupunkt: ");
  //   Serial.print(dewPoint, 2);
  //   Serial.println(" °C");
  // }

  // if (Serial.available())
  // {
  //   String s = Serial.readStringUntil('\n');
  //   s.trim();
  //   if (s.length() >= 2 && (s[0] == 'p' || s[0] == 'P'))
  //   {
  //     float val = s.substring(1).toFloat();
  //     fan.setPercent(val);
  //     Serial.printf("[FAN] -> %.1f %%\n", fan.getPercent());
  //   }
  // }
  // }

// ########################################################

// #include <Arduino.h>
// #include <Wire.h>
// #include "stepper_ctrl.h"
// #include "tof_ctrl.h"

// // ====== Pinbelegung anpassen ======
// static constexpr int I2C_SDA_1 = 8; // SDA des ESP32‑S3
// static constexpr int I2C_SCL_1 = 9; // SCL des ESP32‑S3
// static constexpr int PIN_XSHUT = 4;   // XSHUT‑Pin des VL53L0X (optional)

// // Instanz des Sensors
// ToFCtrl tof;

// StepperCtrl step;

// void setup()
// {
//   delay(200);
//   Serial.begin(115200);
//   delay(200);

//   // I²C starten
//   Wire.begin(I2C_SDA_1, I2C_SCL_1, 400000);
//   // Sensor einschalten und initialisieren
//   if (PIN_XSHUT >= 0)
//   {
//     pinMode(PIN_XSHUT, OUTPUT);
//     digitalWrite(PIN_XSHUT, HIGH);
//     delay(3);
//   }
//   bool ok = tof.begin(Wire, 0x29, PIN_XSHUT);
//   Serial.println(ok ? F("[ToF] Init OK") : F("[ToF] Init FAILED"));
//   if (ok)
//   {
//     uint8_t model = 0, rev = 0;
//     if (tof.getModelInfo(model, rev))
//     {
//       Serial.print(F("[ToF] Model ID: "));
//       Serial.print(model);
//       Serial.print(F("  Revision: "));
//       Serial.println(rev);
//     }
//   }

//   int mm_raw = tof.readRawMm();
//   int mm_avg = tof.readAvgMm(10);

//   // Auswertung des Rohwerts: negative Codes unterscheiden Bereichsverletzungen
//   if (mm_raw >= 0)
//   {
//     Serial.printf("RAW  : %4d mm\n", mm_raw);
//   }
//   else if (mm_raw == -2)
//   {
//     Serial.println("RAW  : too close (under min range)");
//   }
//   else
//   {
//     Serial.println("RAW  : out of range / error");
//   }

//   // Gemittelter Messwert (ignoriert zu-nahe/out-of-range Einzelmessungen)
//   if (mm_avg >= 0)
//   {
//     Serial.printf("AVG10: %4d mm\n", mm_avg);
//   }
//   else
//   {
//     Serial.println("AVG10: out of range / error");
//   }

//   // Stepper initialisieren
//   step.begin();
//   step.setAxisUpDir(-1);
//   step.enableDebug(true);            // Debug an
//   step.setDebugMoveLogInterval(100); // alle 100 ms Logs

//   // Homing beim Start:
//   while (!step.home())
//   {
//     step.tick();
//   }
//   delay(100);
//   Serial.println("Homing abgeschlossen!");
//   // while (1)
//   // {
//   //   /* code */
//   // }

// }

// void loop()
// {
//   // step.tick();

//   static unsigned long lastToFPrint = 0;
//   unsigned long now = millis();

//   // Alle 800ms Entfernung ausgeben
//   if (now - lastToFPrint >= 800) {
//     int mm = tof.readRawMm();
//     Serial.printf("[ToF] Entfernung: %d mm\n", mm);
//     lastToFPrint = now;
//   }

//   int mm_raw = tof.readRawMm();
//   const int target_mm = 100;
//   const int tolerance = 5; // mm Toleranzbereich

//   if (mm_raw >= 0) {
//     if (mm_raw > target_mm + tolerance) {
//       // Zu weit weg, Stepper nach unten bewegen
//       step.moveDown(mm_raw - target_mm, 5); // Geschwindigkeit 2 mm/s
//     } else if (mm_raw < target_mm - tolerance) {
//       // Zu nah dran, Stepper nach oben bewegen
//       step.moveUp(target_mm - mm_raw, 5); // Geschwindigkeit 2 mm/s
//     } else {
//       // Im Zielbereich, Stepper anhalten
//       step.stop();
//     }
//   }

//     // // dann auf 100mm
//   // step.moveTo(100.0f, 5);
//   // while (step.getPositionMm() != 100.0f)
//   // {
//   //   step.tick();
//   // }
//   // step.moveTo(20.0f, 5);
//   // // Beispiele:
//   // // step.goTop(4);
//   // // step.moveDown(20.0f, 2);
//   // // step.jogUp(3);   ... später: step.jogStop();
// }
