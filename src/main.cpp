// LuminaGrowX main – uses centralized lumina_config

#include <Arduino.h>

#include <WiFi.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <time.h>

#include <Wire.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "stepper_ctrl.h"
#include "tof_ctrl.h"
#include "vpd_calc.h"
#include "plant_ctrl.h"
#include "rtc_ctrl.h"
#include "lumina_config.h"
#include "net_ctrl.h"
#include "web_ctrl.h"

using namespace vpd_calc;
using namespace plant_ctrl;

static GrowthStage CURRENT_STAGE = GrowthStage::Flowering;

// Pins aus zentraler Config
constexpr uint8_t FAN_PWM_PIN = lumina::pins::FAN_PWM;
constexpr int I2C_SDA_1 = lumina::pins::I2C1_SDA;
constexpr int I2C_SCL_1 = lumina::pins::I2C1_SCL;
constexpr int I2C_SDA_2 = lumina::pins::I2C2_SDA;
constexpr int I2C_SCL_2 = lumina::pins::I2C2_SCL;
constexpr uint32_t I2C_FREQ_2 = lumina::pins::I2C2_FREQ;
constexpr int PIN_XSHUT = lumina::pins::TOF_XSHUT;

// Geräte
SHT41Ctrl sht_in;
SHT41Ctrl sht_out;
GP8211Ctrl dac;
FanCtrl fan;
ToFCtrl tof;

// Stepper aus zentraler Config
static StepperPins makeStepPins() {
  StepperPins p;
  p.pin_step  = lumina::pins::Stepper::STEP;
  p.pin_dir   = lumina::pins::Stepper::DIR;
  p.pin_en    = lumina::pins::Stepper::EN;
  p.pin_diag  = lumina::pins::Stepper::DIAG;
  p.uart_tx   = lumina::pins::Stepper::UART_TX;
  p.uart_rx   = lumina::pins::Stepper::UART_RX;
  p.uart      = &Serial1;
  p.uart_baud = lumina::pins::Stepper::UART_BAUD;
  return p;
}
static StepperCtrl step(makeStepPins());
PlantCtrl controller;
RTC_Ctrl rtc;
net_ctrl::NetCtrl net;
web_ctrl::WebCtrl web;

static inline bool approxEq(float a, float b, float eps = 0.02f) {
  return fabsf(a - b) <= eps;
}

static void waitUntilStill(StepperCtrl& s, unsigned long stable_ms = 250, unsigned long timeout_ms = 60000) {
  const float eps = 0.01f;
  unsigned long t0 = millis();
  float lastPos = s.getPositionMm();
  unsigned long stableSince = millis();
  for (;;) {
    s.tick();
    float p = s.getPositionMm();
    if (fabsf(p - lastPos) <= eps) {
      if (millis() - stableSince >= stable_ms) break;
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

void setup()
{
  Serial.begin(115200);
  delay(200);

  // Optional: Boot-Zurücksetzen in AP-Mode über GPIO6 (aktiv LOW für 5s, Pullup)
  net.configureResetPin(6, /*activeHigh=*/false);

  // I2C
  Wire.begin(I2C_SDA_1, I2C_SCL_1, 400000);
  Wire1.begin(I2C_SDA_2, I2C_SCL_2, I2C_FREQ_2);

  // Fan
  FanCtrl::Config cfg;
  cfg.pwmPin = FAN_PWM_PIN;
  cfg.ledcChannel = 0;
  cfg.pwmFreq = 25000;
  cfg.resolutionBits = 8;
  cfg.invert = false;
  fan.begin(cfg);
  fan.setPercent(0);

  // LED Initialisierung (GP8211)
  if (!dac.begin(Wire1)) {
    Serial.println("[GP8211] init FAIL (I2C addr 0x58)");
  } else {
    Serial.println("[GP8211] init OK (10V-range set)");
  }
  // LED/DAC-Minimalspannung anpassen (Anzeige: 1% entspricht dieser Spannung)
  dac.setMinVoltAt1Percent(lumina::ledcfg::MIN_V_AT_1PCT);
  dac.setPercent(10);

  // Stepper
  step.begin();
  step.enableDebug(true);
  step.setDebugMoveLogInterval(100);
  while (!step.home()) { step.tick(); }

  // step.moveTo(180.0f, 5); // --------------------------------------------------------------- Nur für den Anfang, damit die LED näher an der Pflanze ist
  // waitUntilStill(step);

  // ToF aktivieren und starten
  if (PIN_XSHUT >= 0) {
    pinMode(PIN_XSHUT, OUTPUT);
    digitalWrite(PIN_XSHUT, HIGH);
    delay(3);
  }
  bool ok = tof.begin(Wire, 0x29, PIN_XSHUT);
  Serial.println(ok ? F("[ToF] Init OK") : F("[ToF] Init FAILED"));

  // FS
  if (!LittleFS.begin(true)) {
    Serial.println(F("[FS] LittleFS mount/format FAILED"));
  } else {
    Serial.println(F("[FS] LittleFS bereit"));
  }

  // // ToF: Offset laden oder automatisch kalibrieren (Fixture mit definiertem Abstand)
  // {
  //   int offsetLoaded = 0;
  //   bool haveOffset = false;
  //   File f = LittleFS.open("/tof_offset.dat", FILE_READ);
  //   if (f) {
  //     String s = f.readString();
  //     f.close();
  //     int val = 0; int n = 0;
  //     if (s.length() > 0) n = sscanf(s.c_str(), "%d", &val);
  //     if (n == 1) { tof.setOffsetMm((int16_t)val); haveOffset = true; }
  //   }
  //   if (!haveOffset) {
  //     // Automatische Ein-Punkt-Offset-Kalibrierung
  //     int measured = tof.readAvgMm(lumina::calib::TOF_CAL_SAMPLES);
  //     if (measured >= 0) {
  //       int target = lumina::calib::TOF_CAL_TARGET_MM;
  //       int off = measured - target; // readRawMm() zieht diesen Offset ab
  //       tof.setOffsetMm((int16_t)off);
  //       File fw = LittleFS.open("/tof_offset.dat", FILE_WRITE);
  //       if (fw) { fw.printf("%d\n", off); fw.close(); }
  //       Serial.println(F("[CAL] ToF Offset kalibriert und gespeichert"));
  //     }
  //   }
  // }

  // RTC (vor Netzstart initialisieren, damit NetCtrl darauf zugreifen darf)
  if (!rtc.begin(Wire1)) {
    Serial.println("[RTC] Keine Verbindung zur DS3231 (0x57).");
  } else {
    Serial.println("[RTC] Init OK (DS3231 gefunden).");
  }

  // Netzwerkteil starten (AP immer, STA wenn konfiguriert); mDNS: luminagrowx.local
  {
    bool forceAP = net.shouldForceAPAtBoot(5000);
    net.begin(forceAP, "luminagrowx", &rtc);
  }

  // SHT41
  Serial.println(F("[SHT41_in] Init..."));
  if (!sht_in.begin(Wire1, true)) {
    Serial.println(F("[SHT41_in] Init FAILED (Sensor nicht erreichbar?)"));
  } else {
    Serial.println(F("[SHT41_in] Init OK"));
  }
  Serial.println(F("[SHT41_out] Init..."));
  if (!sht_out.begin(Wire, true)) {
    Serial.println(F("[SHT41_out] Init FAILED (Sensor nicht erreichbar?)"));
  } else {
    Serial.println(F("[SHT41_out] Init OK"));
  }

  // Zentrale Env-Defaults anwenden (aus lumina_config.h)
  {
    const GrowthStage stages[3] = { GrowthStage::Seedling, GrowthStage::Vegetative, GrowthStage::Flowering };
    const DayMode modes[3] = { DayMode::Day, DayMode::Night, DayMode::NightSilent };
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        const auto& s = lumina::defaults::PHASE_MODE[i][j];
        controller.setStageModeSettings(stages[i], modes[j], s.ledPercent, s.fanMin, s.fanMax, s.vpdMin, s.vpdMax);
      }
    }
  }

  // Controller verbinden und konfigurieren (inkl. Stepper/ToF/RTC/Tuer)
  controller.begin(sht_in, sht_out, dac, fan, step, tof, &rtc, lumina::plant::DOOR_SWITCH_PIN);
  controller.setStage(vpd_calc::GrowthStage::Flowering);
  controller.setMode(DayMode::Day);
  controller.applyLuminaConfig();

  // Nach Homing: blockierende Annäherung an Mindestabstand
  // controller.runStartupApproachBlocking();

  Serial.println(F("PlantCtrl demo startet"));

  // Webserver + API starten
  web.begin(&controller, &rtc, &net);
}

void loop()
{
  // Keep stepper ticking every frame
  step.tick();

  // Run controller at high rate every loop iteration
  controller.update();

  // Debug print every ~3s without throttling the control loop
  static unsigned long lastLogMs = 0;
  unsigned long now = millis();
  if (now - lastLogMs >= 3000) {
    lastLogMs = now;

    String t = rtc.readTimeString();
    Serial.print(t);

    float tC_in = NAN, tC_out = NAN, rh_in = NAN, rh_out = NAN;
    if (sht_in.read(tC_in, rh_in) && sht_out.read(tC_out, rh_out)) {
      Serial.print(F(" | [INDOOR] Temp: "));
      Serial.print(tC_in, 1);
      Serial.print(F(" C, RH: "));
      Serial.print(rh_in, 1);
      Serial.print(F(" %, Taupunkt: "));
      Serial.print(computeDewPoint(static_cast<double>(tC_in), static_cast<double>(rh_in)), 1);
      Serial.print(F(" C, VPD: "));
      Serial.print(computeVpd(static_cast<double>(tC_in), static_cast<double>(rh_in)), 3);
      Serial.print(F(" kPa"));

      Serial.print(F(" | [OUTDOOR] Temp: "));
      Serial.print(tC_out, 1);
      Serial.print(F(" C, RH: "));
      Serial.print(rh_out, 1);
      Serial.print(F(" % | LED: "));
      Serial.print(controller.currentLedPercentEffective(), 1);
      Serial.print(F(" % | Fan: "));
      Serial.print(controller.currentFanPercent(), 1);

      int distance = tof.readRawMm();
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
  }

  // Web/Net periodische Aufgaben
  web.loop();
}
