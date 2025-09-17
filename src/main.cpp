
// ########################################################

#include <Arduino.h>
#include <Wire.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "stepper_ctrl.h"
#include "tof_ctrl.h"
#include "vpd_calc.h"
#include "env_ctrl.h"

// VPD
using namespace vpd_calc;
// Umweltsteuerung
using namespace env_ctrl;

// Definiere das aktuelle Wachstumsstadium für die Klassifizierung.
// Du kannst dies basierend auf deinem Wachstumsplan ändern oder auf Kalenderwochen abbilden.
static GrowthStage CURRENT_STAGE = GrowthStage::Vegetative;

//  Fan-Pins definieren
constexpr uint8_t FAN_PWM_PIN = 2;

//  I2C-Pins definieren
constexpr int I2C_SDA_1 = 8; // SDA des ESP32‑S3
constexpr int I2C_SCL_1 = 9; // SCL des ESP32‑S3
constexpr int I2C_SDA_2 = 20;
constexpr int I2C_SCL_2 = 19;
constexpr uint32_t I2C_FREQ_2 = 400000; // 400 kHz

// ToF XSHUT-Pin definieren
constexpr int PIN_XSHUT = 4; // XSHUT‑Pin des VL53L0X (optional)

//  Sensoren und Aktoren initialisieren
SHT41Ctrl sht;      // Adresse 0x44 (fix)
GP8211Ctrl dac;     // Adresse 0x58 (fix)
FanCtrl fan;        // Lüftersteuerung
ToFCtrl tof;        // Time-of-Flight Distanzsensor
StepperCtrl step;   // Schrittmotorsteuerung
EnvCtrl controller; // Umweltsteuerung

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Gemeinsamer I²C-Bus für mehrere ICs:
  Wire.begin(I2C_SDA_1, I2C_SCL_1, 400000);
  Wire1.begin(I2C_SDA_2, I2C_SCL_2, I2C_FREQ_2);

  //  Stepper initialisieren
  step.begin();
  step.setAxisUpDir(-1);
  step.enableDebug(true);            // Debug an
  step.setDebugMoveLogInterval(100); // alle 100 ms Logs

  //  Homing beim Start:
  while (!step.home()) //  Homing beim Start:
  {
    step.tick();
  }

  //  Sensor einschalten und initialisieren
  if (PIN_XSHUT >= 0)
  {
    pinMode(PIN_XSHUT, OUTPUT);
    digitalWrite(PIN_XSHUT, HIGH);
    delay(3);
  }
  bool ok = tof.begin(Wire, 0x29, PIN_XSHUT);
  Serial.println(ok ? F("[ToF] Init OK") : F("[ToF] Init FAILED"));
  if (ok)
  {
    uint8_t model = 0, rev = 0;
    if (tof.getModelInfo(model, rev))
    {
      Serial.print(F("[ToF] Model ID: "));
      Serial.print(model);
      Serial.print(F("  Revision: "));
      Serial.println(rev);
    }
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
  //  Setze Fan-Geschwindigkeit
  fan.setPercent(50);

  // GP8211 initialisieren
  if (!dac.begin(Wire1))
  {
    Serial.println("[GP8211] init FAIL (I2C addr 0x58)");
    // Weiterlaufen erlaubt, aber keine Ausgabe
  }
  else
  {
    Serial.println("[GP8211] init OK (10V-range set)");
  }
  // Ausgang auf 2 V / 10%
  dac.setPercent(10);

  // SHT41 initialisieren
  Serial.println(F("[SHT41] Init..."));
  if (!sht.begin(Wire1, /*softReset=*/true))
  {
    Serial.println(F("[SHT41] Init FAILED (Sensor nicht erreichbar?)"));
    // Weiterlaufen ist optional; hier brechen wir nicht ab.
  }
  else
  {
    Serial.println(F("[SHT41] Init OK"));
  }

  // Bind hardware to controller
  controller.begin(sht, dac, fan);
  // Set desired growth stage and mode
  controller.setStage(vpd_calc::GrowthStage::Seedling); // Seedling, Vegetative, Flowering
  controller.setMode(DayMode::Day);
  // Optionally tweak gains
  controller.setKpFan(20.0f);
  controller.setKpLed(10.0f);

  Serial.println(F("EnvCtrl demo started"));
}

void loop()
{

  if (controller.update())
  {
    Serial.print(F("Temp: "));
    Serial.print(controller.currentTemp(), 2);
    Serial.print(F(" C, RH: "));
    Serial.print(controller.currentRh(), 1);
    Serial.print(F(" %, VPD: "));
    Serial.print(controller.currentVpd(), 3);
    Serial.print(F(" kPa, LED: "));
    Serial.print(controller.currentLedPercent(), 1);
    Serial.print(F(" %, Fan: "));
    Serial.print(controller.currentFanPercent(), 1);
    Serial.println(F(" %"));
  }
  else
  {
    Serial.println(F("Sensor read failed"));
  }
  delay(2000);
  // static uint32_t tPrev = 0;
  // uint32_t now = millis();
  // if (now - tPrev > 1000)
  // {
  //   tPrev = now;
  //   Serial.printf("[FAN] %5.1f %%\n", fan.getPercent());

  //   float tC = NAN, rh = NAN;
  //   if (sht.read(tC, rh))
  //   {
  //     Serial.print(F("[SHT41] T="));
  //     Serial.print(tC, 2);
  //     Serial.print(F(" °C  RH="));
  //     Serial.print(rh, 2);
  //     Serial.println(F(" %"));
  //   }
  //   else
  //   {
  //     Serial.println(F("[SHT41] Read FAILED"));
  //   }

  //   // Berechne VPD
  //   double vpd = computeVpd(static_cast<double>(tC), static_cast<double>(rh));
  //   // Klassifiziere VPD
  //   const char* status = classifyVpd(vpd, CURRENT_STAGE);

  //   // Berechne den Taupunkt zur Information
  //   double dewPoint = computeDewPoint(static_cast<double>(tC), static_cast<double>(rh));

  //   // Ausgabe der Ergebnisse
  //   Serial.print("Temp: ");
  //   Serial.print(tC);
  //   Serial.print(" °C, Luftfeuchtigkeit: ");
  //   Serial.print(rh);
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
}

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
