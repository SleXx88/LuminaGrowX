// ==============================
// File: include/lumina_config.h
// ==============================
// LuminaGrowX - Zentrale Projektkonfiguration
// Pins, Phasen/Modi-Defaults, Licht-Zeitpläne, Regelparameter

#pragma once

#include <Arduino.h>
#include "vpd_calc.h"
#include "plant_ctrl.h"
// #include "env_ctrl.h" // war: nur für DayMode/PhaseModeSettings; ersetzt durch plant_ctrl

namespace lumina
{

  // --- Hardware-Pins ---
  namespace pins
  {
    // I2C-Bus 1 (Innen)
    constexpr int I2C1_SDA = 8;
    constexpr int I2C1_SCL = 9;

    // I2C-Bus 2 (Außen / weitere Geräte)
    constexpr int I2C2_SDA = 20;
    constexpr int I2C2_SCL = 19;
    constexpr uint32_t I2C2_FREQ = 400000; // 400 kHz

    // 4-Pin-Lüfter (PWM)
    constexpr uint8_t FAN_PWM = 2;

    // ToF (VL53L0X) – XSHUT (optional)
    constexpr int TOF_XSHUT = 4;

    // Stepper / TMC2209
    struct Stepper
    {
      static constexpr int STEP = 12;
      static constexpr int DIR = 11;
      static constexpr int EN = 10;
      static constexpr int DIAG = 13;
      static constexpr int UART_TX = 17; // MCU TX -> TMC PDN (RX)
      static constexpr int UART_RX = 18; // MCU RX <- TMC PDN (TX)
      static constexpr uint32_t UART_BAUD = 115200;
    };
  }

  // --- Phasen & Modi ---
  using PlantPhase = vpd_calc::GrowthStage; // Seedling, Vegetative, Flowering
  using DayMode = plant_ctrl::DayMode;      // Day, Night, NightSilent

  // --- Licht-Zeitpläne (optional) ---
  struct TimeHM
  {
    uint8_t hour;
    uint8_t minute;
  };
  struct LightSchedule
  {
    TimeHM on;                // Start (Licht an)
    TimeHM off;               // Ende  (Licht aus)
    uint16_t sunrise_minutes; // Ramp-up Dauer
    uint16_t sunset_minutes;  // Ramp-down Dauer
  };

  namespace schedule
  {
    // Hinweis: Beispiel – identisch für alle Phasen
    constexpr LightSchedule SEEDLING{{8, 0}, {18, 0}, 20, 20};
    constexpr LightSchedule VEGETATIVE{{8, 0}, {18, 0}, 20, 20};
    constexpr LightSchedule FLOWERING{{8, 0}, {18, 0}, 20, 20};
  }

  // --- Default-Parameter je Phase & Modus ---
  namespace defaults
  {
    // { LED%, FanMin%, FanMax%, VPDmin[kPa], VPDmax[kPa] }
    constexpr plant_ctrl::PhaseModeSettings PHASE_MODE[3][3] = {
        // SEEDLING
        {
            {40.0f, 20.0f, 80.0f, 0.40f, 0.80f}, // Day
            {0.0f, 20.0f, 60.0f, 0.40f, 0.80f},  // Night
            {0.0f, 10.0f, 40.0f, 0.40f, 0.80f}   // NightSilent
        },
        // VEGETATIVE
        {
            {65.0f, 20.0f, 100.0f, 0.80f, 1.20f}, // Day
            {0.0f, 20.0f, 60.0f, 0.80f, 1.20f},   // Night
            {0.0f, 10.0f, 40.0f, 0.80f, 1.20f}    // NightSilent
        },
        // FLOWERING (Ziel 1.2–1.4 kPa, passive Feuchtequellen)
        {
            {90.0f, 10.0f, 70.0f, 1.20f, 1.40f}, // Day  – FanMin reduziert, FanMax reduziert
            {0.0f, 20.0f, 70.0f, 1.20f, 1.40f},  // Night
            {0.0f, 10.0f, 50.0f, 1.20f, 1.40f}   // NightSilent
        }};
  }

  // --- Plant distance / door / ToF parameters ---
  namespace plant
  {
    // Türschalter-Pin (LOW = Tür zu, gegen GND). -1 deaktiviert und nimmt "immer zu" an.
    // Erklärung Türerkennung:
    // - door_dRh: Anteilige Veränderung der relativen Luftfeuchte [%] zwischen zwei Messungen,
    //   die als Hinweis auf ein Türereignis gewertet wird (z. B. plötzlicher Feuchtigkeitsanstieg).
    // - door_dT: Temperaturänderung [°C] zwischen zwei Messungen, die ein Türereignis bestätigt.
    // - door_hold_ms: Zeitdauer [ms], für die das System nach Erkennung eines Türereignisses in
    //   einem besonderen Haltezustand verbleibt (z. B. keine Moduswechsel, geänderte Reglereinstellungen).
    constexpr int DOOR_SWITCH_PIN = 5;

    // Minimale LED-zu-Pflanze Abstände pro Phase (mm)
    // Keimling=50mm, Vegetativ=80mm, Blüte=120mm
    constexpr float MIN_DIST_MM[3] = {50.0f, 80.0f, 120.0f};

    // ToF-Sensor sitzt 10 mm unter der LED -> diesen Offset zur ToF-Messung hinzurechnen
    constexpr int TOF_LED_OFFSET_MM = 10;

    // ToF-Leseintervall und Mittelung
    constexpr uint32_t TOF_READ_INTERVAL_MS = 50;   // Sicherheitsabstand für Sensor-Timing
    constexpr uint8_t  TOF_AVG_SAMPLES      = 10;   // Mittelung pro Messung

    // Verhalten der Distanzanpassung
    constexpr float    ADJUST_HYST_MM = 2.0f;       // Totband um Ziel
    constexpr float    ADJUST_STEP_MM = 1.0f;       // pro Anpassungsschritt
    constexpr uint8_t  SPEED_LEVEL    = 5;          // Index in der Steppertabelle
    // Zeitfenster für Nachregelung nach Events (ms)
    constexpr uint32_t ADJUST_WINDOW_MS = 30000;
    constexpr float    STARTUP_STEP_MM = 5.0f;      // größere Schritte beim ersten Heranfahren
  }

  // --- Zentrale Regelparameter (hier veränderst du ALLES) ---
  namespace ctrl
  {
    struct Params
    {
      // Regelung (VPD -> Fan)
      float Kp;               // [%/kPa] Proportional
      float Ki;               // [(%/s)/kPa] Integrator (sehr klein)
      float deadband_kPa;     // Totband um Soll-VPD
      float rate_limit_pct_s; // max. % pro Sekunde (sanft)
      float ema_alpha;        // 0..1 (0 = sehr träge, 1 = ungefiltert)

      // Außen feuchter -> Lüfter blockieren
      bool outside_humid_block; // true: blocken wenn dp_out > dp_in + Hyst
      float dp_hysteresis_C;    // °C

      // Overrides
      float rh_high_thr;          // %RH, ab der Feuchte-Override greift
      float dew_gap_min_C;        // °C, Mindestabstand Temp – Taupunkt innen (Kondensation)
      bool allow_silent_override; // NightSilent darf bei Notlage lauter werden
      float humid_boost_pct;      // %-Punkte über fanMin bei Feuchte-Override

      // Temperatur-Schutz
      float max_temp_C;          // LED-Reduktion ab
      float led_reduce_pct;      // %-Punkte LED runter
      float temp_high_fan_C;     // ab hier Fan-Boost
      float temp_high_fan_boost; // %-Punkte über fanMin

      // Tür / Transienten / Holds
      float door_dRh; // %RH Sprung, der als Türereignis interpretiert werden kann
      float door_dT;  // °C  Sprung, der Türereignis unterstützt
      uint32_t door_hold_ms; // Haltezeit nach Türöffnung [ms]
      uint32_t mode_hold_ms; // Haltezeit nach Moduswechsel [ms]
    };

    // Träge, stabile Defaults passend zu passiver Befeuchtung
    constexpr Params PARAMS{
        /*Kp*/ 6.0f,
        /*Ki*/ 0.050f,             // deutlich größer als vorher (0.010)
        /*deadband_kPa*/ 0.12f,    // größer = ruhiger, vermeidet Nachjagen
        /*rate_limit_pct_s*/ 1.5f, // sehr sanfte Stelländerungen
        /*ema_alpha*/ 0.20f,       // träge Messwertfilterung

        /*outside_block*/ true,         // Lüfter blockieren wenn außen feuchter
        /*dp_hyst_C*/ 0.5f,             // Hysterese für Taupunktvergleich

        /*rh_high_thr*/ 85.0f,          // Schwelle für Feuchte-Override
        /*dew_gap_min_C*/ 1.5f,         // Mindestabstand Temp-Taupunkt innen
        /*allow_silent_ov*/ true,       // NightSilent darf lauter werden
        /*humid_boost_pct*/ 20.0f,      // Zusatz-Lüfterleistung bei Override

        /*max_temp_C*/ 30.0f,           // LED-Reduktion ab dieser Temp
        /*led_reduce_pct*/ 5.0f,        // LED-Reduktion in %
        /*temp_high_fan_C*/ 31.0f,      // Fan-Boost ab dieser Temp
        /*temp_high_boost*/ 15.0f,      // Fan-Boost in %

        /*door_dRh*/ 8.0f,              // RH-Sprung für Türerkennung
        /*door_dT*/ 2.0f,               // T-Sprung für Türerkennung
        /*door_hold_ms*/ 15000,         // Haltezeit nach Türöffnung [ms]
        /*mode_hold_ms*/ 8000};         // Haltezeit nach Moduswechsel [ms]
  }

  // --- Sensor-/Kalibrier-Defaults ---
  namespace calib
  {
    constexpr int TOF_CAL_TARGET_MM = 100;
    constexpr uint8_t TOF_CAL_SAMPLES = 10;
  }

} // namespace lumina
