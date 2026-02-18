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
#include <sys/time.h>

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
#include "../lib/mqtt_ctrl/mqtt_ctrl.h"
#include "../include/health.h"
#include "../include/version.h"
#include "../include/setup_flag.h"
#include "led_ctrl.h"

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

// Geräte
SHT41Ctrl sht_in;
SHT41Ctrl sht_out;
GP8211Ctrl dac;
FanCtrl fan;
FanCtrl fan2;
FanCtrl fan3;
ToFCtrl tof;
LedCtrl statusLed(lumina::pins::LED_RGB);

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
MqttCtrl mqtt;
static bool g_setupMode = false;

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

// Versucht einen blockierten I2C-Bus zu befreien, indem SCL manuell getaktet wird.
// Hilft, wenn ein Slave (z.B. VL53L0X) SDA auf LOW hält.
static void recoverI2C(int sdaPin, int sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);
  delay(10); // Kurze Wartezeit

  // Wenn SCL oder SDA dauerhaft LOW sind, versuchen wir Recovery
  if (digitalRead(sclPin) == LOW || digitalRead(sdaPin) == LOW) {
    Serial.printf("[I2C-RECOVERY] Bus an Pin SDA=%d/SCL=%d scheint blockiert. Starte Reset-Sequenz...\n", sdaPin, sclPin);
    
    pinMode(sclPin, OUTPUT);
    pinMode(sdaPin, INPUT); // SDA floaten lassen, Slave treibt es ggf. LOW
    
    // Bis zu 9 Clock-Pulse senden, damit der Slave sein Bit fertig sendet und SDA freigibt
    for (int i = 0; i < 9; i++) {
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(10);
      if (digitalRead(sdaPin) == HIGH) {
        // SDA ist high -> Slave hat losgelassen
        break;
      }
      digitalWrite(sclPin, LOW);
      delayMicroseconds(10);
    }
    
    // STOP Condition senden
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(10);

    Serial.println(F("[I2C-RECOVERY] Sequenz beendet."));
  } else {
    // Bus scheint frei zu sein
    // Serial.println(F("[I2C-RECOVERY] Busleitungen sind HIGH (OK)."));
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.printf("[FW] Version %s (%s %s)\n", FW_VERSION, __DATE__, __TIME__);
  Serial.println(F("[INIT] Setup startet..."));

  // Optional: Boot-Zurücksetzen in AP-Mode über GPIO6 (aktiv LOW für 5s, Pullup)
  // AP-Mode-Reset-Pin aus lumina_config.h
  // net.configureResetPin moved to Network section

  // ===== 1) Basis: I2C-Busse =====
  // Vor Wire.begin() versuchen, den Bus freizumachen
  recoverI2C(I2C_SDA_1, I2C_SCL_1);
  recoverI2C(I2C_SDA_2, I2C_SCL_2);

  // Frequenz auf 100kHz reduziert für höhere Stabilität mit VL53L0X
  Wire.begin(I2C_SDA_1, I2C_SCL_1, 100000);
  Wire1.begin(I2C_SDA_2, I2C_SCL_2, I2C_FREQ_2);
  health::set_i2c0(true);
  health::set_i2c1(true);

  // Fan 1
  FanCtrl::Config cfg;
  cfg.pwmPin = lumina::pins::FAN_PWM;
  cfg.ledcChannel = 0;
  cfg.pwmFreq = 25000;
  cfg.resolutionBits = 8;
  cfg.invert = false;
  cfg.tachoPin = lumina::pins::FAN_TACHO; 
  fan.begin(cfg);
  fan.setPercent(0);

  // Fan 2 (zusätzlich)
  FanCtrl::Config cfg2;
  cfg2.pwmPin = lumina::pins::FAN2_PWM;
  cfg2.ledcChannel = 1; // Wichtig: Anderer Channel!
  cfg2.pwmFreq = 25000;
  cfg2.resolutionBits = 8;
  cfg2.invert = false;
  cfg2.tachoPin = lumina::pins::FAN2_TACHO;
  fan2.begin(cfg2);
  fan2.setPercent(0);

  // Fan 3 (LED-Lüfter, kein Tacho)
  FanCtrl::Config cfg3;
  cfg3.pwmPin = lumina::pins::FAN_PWM_LED;
  cfg3.ledcChannel = 2; // Channel 2
  cfg3.pwmFreq = 25000;
  cfg3.resolutionBits = 8;
  cfg3.invert = false;
  cfg3.tachoPin = -1; // Kein Tacho
  fan3.begin(cfg3);
  fan3.setPercent(0);

  // LED Initialisierung (GP8211)
  if (!dac.begin(Wire)) {
    Serial.println("[GP8211] init FAIL (I2C addr 0x58)");
    health::set_dac(false, F("GP8211 nicht erreichbar"));
  } else {
    Serial.println("[GP8211] init OK (10V-range set)");
    health::set_dac(true);
  }
  // LED/DAC-Minimalspannung anpassen (Anzeige: 1% entspricht dieser Spannung)
  dac.setMinVoltAt1Percent(lumina::ledcfg::MIN_V_AT_1PCT);
  // dac.setPercent(10);

  // ===== 2) Hardware-Sensoren =====
  // ToF aktivieren und starten
  // XSHUT nicht verwendet, direkt -1 übergeben
  bool ok = tof.begin(Wire1, 0x29, -1);
  tof.setDebug(true); // Debugging für ToF-Sensor aktivieren
  Serial.println(ok ? F("[ToF] Init OK") : F("[ToF] Init FAILED"));
  health::set_tof(ok, ok ? String("") : F("ToF Init fehlgeschlagen"));

  // SHT41 initialisieren
  Serial.println(F("[SHT41_in] Init..."));
  if (!sht_in.begin(Wire, true)) {
    Serial.println(F("[SHT41_in] Init FAILED (Sensor nicht erreichbar?)"));
    health::set_sht_in(false, F("SHT41 innen fehlt/fehlerhaft"));
  } else {
    Serial.println(F("[SHT41_in] Init OK"));
    health::set_sht_in(true);
  }
  Serial.println(F("[SHT41_out] Init..."));
  if (!sht_out.begin(Wire1, true)) {
    Serial.println(F("[SHT41_out] Init FAILED (Sensor nicht erreichbar?)"));
    health::set_sht_out(false, F("SHT41 außen fehlt/fehlerhaft"));
  } else {
    Serial.println(F("[SHT41_out] Init OK"));
    health::set_sht_out(true);
  }

  // ===== 3) Filesystem (mit explizitem Partition-Label "littlefs" und Mount-Pfad "/littlefs") =====
  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
    Serial.println(F("[FS] LittleFS mount/format FAILED"));
    health::set_fs(false, F("LittleFS Fehler"));
  } else {
    size_t total = LittleFS.totalBytes();
    size_t used  = LittleFS.usedBytes();
    Serial.printf("[FS] LittleFS bereit total=%u used=%u free=%u\n",
                  (unsigned)total, (unsigned)used, (unsigned)((total>used)?(total-used):0));
  health::set_fs(true);
  }
  // Setup-Flag lesen
  bool setupDone = setup_flag::is_done();
  g_setupMode = !setupDone;

  // Status LED logic
  statusLed.begin();
  statusLed.setBrightness(50); // Moderate brightness
  if (g_setupMode) {
      statusLed.setMode(LedCtrl::Mode::BREATHE);
      statusLed.setColor(LedCtrl::Color::RED);
      statusLed.setCycleTime(2000); // 2 seconds breathe cycle
  } else {
      statusLed.setColor(LedCtrl::Color::OFF);
  }

  // SHT41
  // SHT41 bereits initialisiert (früher im Setup)

  // ===== 4) Stepper + (optional) Homing/Kalibrierung =====
  step.begin();
  // Debugging aktivieren/deaktivieren
  step.enableDebug(false);
  // Log-Intervall für Debug-Moves setzen (ms)
  step.setDebugMoveLogInterval(100);

  // ToF: Kalibrierung erfolgt nur im Setup (NVS)
  // Offset wird später via web.setHardware() aus NVS geladen

  // RTC (vor Netzstart initialisieren, damit NetCtrl darauf zugreifen darf)
  if (!rtc.begin(Wire1)) {
    Serial.println("[RTC] Keine Verbindung zur DS3231 (0x68).");
    health::set_rtc(false, F("RTC nicht erreichbar"));
  } else {
    Serial.println("[RTC] Init OK (DS3231 gefunden).");
    health::set_rtc(true);

    // Systemzeit aus RTC initialisieren (für den Fall, dass kein NTP verfügbar ist)
    uint16_t y; uint8_t mo, d, hh, mi, ss;
    if (rtc.readUTCComponents(y, mo, d, hh, mi, ss)) {
      // Manuelle Konvertierung von UTC nach time_t (Unix-Timestamp)
      // Ersetzt timegm(), das nicht auf allen Systemen verfügbar ist.
      auto is_leap = [](int year) { return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)); };
      time_t t_now = 0;
      for (int i = 1970; i < y; ++i) t_now += is_leap(i) ? 366 : 365;
      static const int mdays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      for (int i = 0; i < mo - 1; ++i) {
        t_now += mdays[i];
        if (i == 1 && is_leap(y)) t_now++;
      }
      t_now += d - 1;
      t_now = (((t_now * 24) + hh) * 60 + mi) * 60 + ss;

      struct timeval tv = { .tv_sec = t_now, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      Serial.printf("[RTC] Systemzeit (UTC) synchronisiert: %04d-%02d-%02d %02d:%02d\n", y, mo, d, hh, mi);
    }
  }

  // Türeffner-Pin konfigurieren (nur für Setup-Tests relevant)
  if (lumina::plant::DOOR_SWITCH_PIN >= 0) {
    pinMode(lumina::plant::DOOR_SWITCH_PIN, INPUT_PULLUP);
  }
  
  // Pumpe
  if (lumina::pins::PUMP_EN >= 0) {
    pinMode(lumina::pins::PUMP_EN, OUTPUT);
    digitalWrite(lumina::pins::PUMP_EN, LOW);
  }

  // ===== 5) Weitere Aktoren + Controller =====
  // Controller verbinden und konfigurieren (inkl. Stepper/ToF/RTC/Tuer)
  if (!g_setupMode) {
    controller.begin(sht_in, sht_out, dac, fan, &fan2, &fan3, step, tof, &rtc, lumina::plant::DOOR_SWITCH_PIN);
    controller.applyLuminaConfig();
  }

  // Netzwerk + Web am Ende
  net.configureResetPin(lumina::netcfg::AP_RESET_PIN,
                        /*activeHigh=*/lumina::netcfg::AP_RESET_ACTIVE_HIGH);
  {
    bool forceAP = net.shouldForceAPAtBoot(lumina::netcfg::AP_RESET_HOLD_MS);
    // keepAP = g_setupMode (AP bleibt aktiv, auch wenn STA verbindet)
    net.begin(forceAP, "luminagrowx", &rtc, g_setupMode);
  }
  web.begin(&controller, &rtc, &net);
  // Hardware-Referenzen für Setup/Tests an WebCtrl geben (inkl. SHT41, Fan2, Fan3)
  web.setHardware(&dac, &fan, &fan2, &fan3, &step, &tof, &sht_in, &sht_out);
  web.setMqtt(&mqtt);

  // Homing-Routine (optional über lumina_config.h)
  // Hier NACH web.setHardware, damit die Kalibrierung nicht überschrieben wird
  if (!g_setupMode && lumina::startup::DO_STEPPER_HOME_ON_BOOT) {
    Serial.println(F("[INIT] Starte sicheres Boot-Homing (nur Nullpunkt oben)..."));
    step.resetHoming();
    if (step.home(false)) { // false = Nur zum oberen Nullpunkt fahren, keine Kalibrierfahrt nach unten
      while (step.status().isHoming) {
        step.tick();
        delay(1);
      }
      Serial.println(F("[INIT] Boot-Homing abgeschlossen."));
    } else {
      Serial.println(F("[INIT] Boot-Homing konnte nicht gestartet werden!"));
    }
  }

  // Nach Homing: blockierende Annäherung an Mindestabstand (optional über lumina_config.h)
  if (!g_setupMode && lumina::startup::DO_APPROACH_MIN_DISTANCE_BOOT) {
    controller.runStartupApproachBlocking();
  }

  // Drying-Status laden und in Controller setzen
  if (!g_setupMode) {
    web_ctrl::DryingState dryState;
    if (web.loadDrying(dryState)) {
      controller.setDryingMode(dryState.active);
      if (dryState.active) {
        Serial.println(F("[INIT] Trocknungsmodus aktiv"));
      }
    }
  }

  Serial.println(F("[INIT] Setup beendet!"));
}

void loop()
{
  unsigned long now = millis();
  // Keep stepper ticking every frame
  step.tick();
  
  // Update LED effects
  statusLed.update();

  // Setup-Modus: nur Setup-Flow + Netzwerk/Web aktiv halten
  if (g_setupMode) {
    web.loop();
    return;
  }

  // Regelung nur laufen lassen, wenn kritische Sensoren OK
  if (health::critical_ok()) {
    health::state().control_paused = false;
    
    // Regelung auf ~10 Hz drosseln für stabile Zeit-Deltas (dt_s)
    static uint32_t lastCtrlMs = 0;
    if (now - lastCtrlMs >= 100) {
      lastCtrlMs = now;
      bool okUpdate = controller.update();
      if (!okUpdate) {
        // Sensor-Lesefehler -> pausieren und markieren
        health::state().control_paused = true;
      }
    }
  } else {
    health::state().control_paused = true;
    // Keine Regelung wenn SHT fehlt; Stepper tickt weiter, Web läuft
  }

  // Debug print every ~3s; nur wenn Regelung aktiv
  static unsigned long lastLogMs = 0;
  if (now - lastLogMs >= 3000) {
    lastLogMs = now;
    if (!health::state().control_paused) {
      // System-Lokalzeit (SNTP + TZ) und RTC-Lokalzeit zum Vergleich ausgeben
      time_t ts = time(nullptr);
      char sysBuf[24] = {0};
      if (ts >= 0) {
        struct tm tmL; localtime_r(&ts, &tmL);
        snprintf(sysBuf, sizeof(sysBuf), "%02d-%02d-%04d %02d-%02d-%02d",
                 tmL.tm_mday, tmL.tm_mon+1, tmL.tm_year+1900,
                 tmL.tm_hour, tmL.tm_min, tmL.tm_sec);
      }
      String rtcStr = rtc.readTimeString();
      Serial.print("SYS:");
      Serial.print(sysBuf);
      Serial.print(" | RTC:");
      Serial.print(rtcStr);

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
        Serial.println(controller.currentFanPercent(), 1);
      }
    } else {
      Serial.println(F("Reglung unterbrochen"));
    }
  }

  // Web/Net periodische Aufgaben
  web.loop();
}
