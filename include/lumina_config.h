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

  // --- LED/DAC-Konfiguration ---
  namespace ledcfg
  {
    // Spannung (Volt), die bei 1% LED ausgegeben wird.
    // 1.0f = Standard (1% -> 1V); kann z.B. auf 0.6f gestellt werden (1% -> 0.6V)
    constexpr float MIN_V_AT_1PCT = 1.0f;
  }

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
    constexpr LightSchedule SEEDLING{{6, 0}, {2, 0}, 20, 20};
    constexpr LightSchedule VEGETATIVE{{6, 0}, {0, 0}, 20, 20};
    constexpr LightSchedule FLOWERING{{6, 0}, {18, 0}, 20, 20};
  }

  // --- Default-Parameter je Phase & Modus ---
  namespace defaults
  {
    // { LED%, FanMin%, FanMax%, VPDmin[kPa], VPDmax[kPa] }
    constexpr plant_ctrl::PhaseModeSettings PHASE_MODE[3][3] = {
        // SEEDLING
        {
            {40.0f, 20.0f, 80.0f, 0.40f, 0.80f}, // Day (VPD 0.40–0.80)
            {0.0f, 20.0f, 60.0f, 0.40f, 0.80f},  // Night
            {0.0f, 10.0f, 40.0f, 0.40f, 0.80f}   // NightSilent
        },
        // VEGETATIVE
        {
            {65.0f, 20.0f, 100.0f, 0.80f, 1.10f}, // Day (VPD 0.80–1.10)
            {0.0f, 20.0f, 60.0f, 0.80f, 1.10f},   // Night
            {0.0f, 10.0f, 40.0f, 0.80f, 1.10f}    // NightSilent
        },
        // FLOWERING (Ziel 1.2–1.4 kPa, passive Feuchtequellen)
        {
            {90.0f, 20.0f, 70.0f, 1.10f, 1.40f}, // Day (VPD 1.10–1.40), FanMin 15, FanMax 70
            {0.0f, 20.0f, 70.0f, 1.10f, 1.40f},  // Night, Fan 20/70
            {0.0f, 10.0f, 50.0f, 1.10f, 1.40f}   // NightSilent, Fan 10/50
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

      // Schimmelprävention – RH-Caps je Phase (+ Hysterese) und Taupunktabstände
      float seedling_rh_cap;        // %RH
      float vegetative_rh_cap;      // %RH
      float flowering_rh_cap;       // %RH (späte Blüte optional 45)
      float rh_cap_hyst;            // %RH Hysterese (allgemein)
      float flowering_rh_cap_hyst;  // %RH Hysterese für Flowering (empf. 2)

      float seedling_dp_gap_min_C;  // °C Mindestabstand Temp – Taupunkt
      float vegetative_dp_gap_min_C;// °C
      float flowering_dp_gap_min_C; // °C

      // Mindesttemperaturen je Phase (Nacht-Floor) und FanMax-Deckel bei Untertemp
      float seedling_min_tempC;     // °C
      float vegetative_min_tempC;   // °C
      float flowering_min_tempC;    // °C
      float minTemp_fanMax_scale;   // 0..1, Deckel-Faktor auf FanMax bei Untertemp

      // Strikte Priorität: RH/Taupunkt überstimmt VPD
      bool humidity_priority_strict;

      // Optional: späte Blüte engerer RH-Cap
      bool  use_flowering_late_cap; // wenn true, setze flowering_rh_cap auf flowering_late_rh_cap
      float flowering_late_rh_cap;  // 45 % empfohlen

      // HP-Exit Cooldown
      uint32_t hp_cooldown_ms;     // Mindestverweilzeit nach HP-Exit
    };

    // Sanftere, schnellere Defaults für höhere Update-Rate, plus Schimmelpräventions-Parameter
    constexpr Params PARAMS{
        /*Kp*/ 8.0f,
        /*Ki*/ 0.030f,             // (%/s)/kPa
        /*deadband_kPa*/ 0.08f,
        /*rate_limit_pct_s*/ 3.0f,
        /*ema_alpha*/ 0.30f,

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
        /*mode_hold_ms*/ 8000,          // Haltezeit nach Moduswechsel [ms]

        /*seedling_rh_cap*/       80.0f,
        /*vegetative_rh_cap*/     65.0f,
        /*flowering_rh_cap*/      55.0f,
        /*rh_cap_hyst*/            3.0f,
        /*flowering_rh_cap_hyst*/  2.0f,

        /*seedling_dp_gap_min_C*/  2.0f,
        /*vegetative_dp_gap_min_C*/5.0f,
        /*flowering_dp_gap_min_C*/ 7.0f,

        /*seedling_min_tempC*/    20.0f,
        /*vegetative_min_tempC*/  20.5f,
        /*flowering_min_tempC*/   20.0f,
        /*minTemp_fanMax_scale*/   0.6f,

        /*humidity_priority_strict*/ true,

        /*use_flowering_late_cap*/   false,
        /*flowering_late_rh_cap*/    45.0f,
        /*hp_cooldown_ms*/           30000
    };
  }

  // Dokumentierte Klimaziele je Phase (nicht direkt in Regelung genutzt)
  namespace climate
  {
    struct TempRange { float minC; float maxC; };
    struct PhaseClimateTargets {
      TempRange day;
      TempRange night;
      float     minTempC;
      float     rh_target_min;
      float     rh_target_max;
      float     rh_cap;
      float     rh_hyst;
      float     vpd_min;
      float     vpd_max;
      float     dp_gap_min_C;
    };

    constexpr PhaseClimateTargets SEEDLING{
      /*day*/   {24.0f, 26.0f},
      /*night*/ {21.0f, 22.0f},
      /*minTempC*/ 20.0f,
      /*RH target*/ 70.0f, 80.0f,
      /*RH cap + hyst*/ 80.0f, 3.0f,
      /*VPD*/ 0.40f, 0.80f,
      /*DP gap*/ 2.0f
    };

    constexpr PhaseClimateTargets VEGETATIVE{
      {25.0f, 27.0f},
      {21.0f, 23.0f},
      20.5f,
      55.0f, 65.0f,
      65.0f, 3.0f,
      0.80f, 1.10f,
      5.0f
    };

    constexpr PhaseClimateTargets FLOWERING{
      {24.0f, 26.0f},
      {20.0f, 22.0f},
      20.0f,
      40.0f, 55.0f,
      55.0f, 2.0f,
      1.10f, 1.40f,
      7.0f
    };
  }

  // --- Sensor-/Kalibrier-Defaults ---
  namespace calib
  {
    constexpr int TOF_CAL_TARGET_MM = 100;
    constexpr uint8_t TOF_CAL_SAMPLES = 10;
  }

} // namespace lumina
