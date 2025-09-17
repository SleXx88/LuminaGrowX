#pragma once
#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

/*
  StepperCtrl – ESP32-S3 + TMC2209 (Adafruit 0,11 Ω) + FastAccelStepper (ISR/RMT)
  -------------------------------------------------------------------------------
  - Nicht-blockierende Zustandsmaschine (tick()-basiert) – Schrittimpulse über ISR
  - Sensorloses Homing über SG_RESULT (Baseline + EMA + Mindestweg + Backoff)
  - Softlimits (0 .. max_travel_mm), logisch ↔ motorisch zuordenbar
  - Komfort-API: home/rehome, goTop/goBottom, moveUp/moveDown, moveTo, jogUp/jogDown/jogStop
  - Debug-Layer mit aussagekräftigen Logs (Homing, Position, Ziel, Softlimits, Jog)
*/

struct StepperPins {
  int pin_step = 12;
  int pin_dir  = 11;
  int pin_en   = 10;
  int pin_diag = 13;
  int uart_tx  = 17; // MCU TX -> TMC PDN (RX)
  int uart_rx  = 18; // MCU RX <- TMC PDN (TX)
  HardwareSerial* uart = &Serial1;
  uint32_t uart_baud = 115200;
};

struct StepperKinematics {
  uint16_t steps_per_rev = 200;
  uint8_t  microsteps    = 16;
  float    lead_mm       = 8.0f; // TR8x8
};

struct StepperLimits {
  float max_travel_mm  = 440.0f;
  float home_offset_mm = 0.0f;
};

struct TMC2209Config {
  float   r_sense       = 0.11f; // Adafruit 0,11Ω
  uint8_t driver_addr   = 0b00;
  uint16_t irun_mA      = 500;
  uint16_t ihold_mA     = 100;
  uint8_t  sg_thrs_diag = 240;   // DIAG nur (Fehler), NICHT Stall
};

struct HomingParams {
  float    speed_fast_hz     = 2500.0f;
  float    speed_slow_hz     = 1200.0f;  // Rückfahrt
  float    backoff_mm        = 3.0f;
  uint32_t sg_ignore_ms      = 50;
  uint32_t sg_poll_ms        = 10;
  uint32_t sg_baseline_ms    = 200;
  uint16_t sg_abs_thr        = 80;        // 0=aus
  float    sg_drop_pct       = 0.30f;
  float    min_stall_mm      = 3.0f;
};

struct SpeedTable {
  float hz[6] = { 150.0f, 400.0f, 800.0f, 1200.0f, 1500.0f, 4000.0f };
};

struct StepperStatus {
  float position_mm = 0.0f;
  bool  diag        = false;  // Treiberfehler-Pin
  bool  isMoving    = false;
  bool  isHoming    = false;
  bool  lastOpDone  = false;  // letzte Ziel-/Jog-/Homing-Op abgeschlossen?
};

class StepperCtrl {
public:
  StepperCtrl(const StepperPins& pins = StepperPins(),
              const StepperKinematics& kin = StepperKinematics(),
              const StepperLimits& lim = StepperLimits(),
              const TMC2209Config& tmc = TMC2209Config(),
              const HomingParams& hp = HomingParams(),
              const SpeedTable& spd = SpeedTable());

  // Grundlaufzeit
  void begin();
  void tick();

  // --- Komfort-API (empfohlen) ---
  // Homing: Startet/überwacht Homing. Liefert true, sobald abgeschlossen.
  bool startHoming();
  inline bool home() { return startHoming(); }

  // Homing zurücksetzen (danach kann erneut gehomed werden)
  void resetHoming();
  inline void rehome() { resetHoming(); }

  // Endlagen (Softlimits)
  std::pair<float,bool> goTop(uint8_t speedLevel);    // -> max_travel_mm
  std::pair<float,bool> goBottom(uint8_t speedLevel); // -> 0 mm

  // Relative Fahrten
  std::pair<float,bool> moveUp(float mm, uint8_t speedLevel);
  std::pair<float,bool> moveDown(float mm, uint8_t speedLevel);

  // Absolute Fahrt
  std::pair<float,bool> moveTo(float pos_mm, uint8_t speedLevel);

  // Jog (Dauergeschwindigkeit, bis jogStop())
  void jogUp(uint8_t speedLevel);
  void jogDown(uint8_t speedLevel);
  void jogStop();

  // Sofort stoppen
  void stop();

  // Status/Config
  StepperStatus status() const;
  float getPositionMm() const;
  void  enableDebug(bool on);
  void  setDebugMoveLogInterval(uint16_t ms); // Standard 100 ms
  void  setCurrents(uint16_t irun_mA, uint16_t ihold_mA);
  void  setMicrosteps(uint8_t microsteps);
  void  setAxisUpDir(int axis_up_dir /* +1 oder -1 */);
  void  setMaxTravelMm(float max_travel_mm);

  // --- Technische API (weiter verfügbar) ---
  // (Bleibt für Kompatibilität, die Komfort-Wrapper sind einfacher.)
  std::pair<float,bool> goToEnd(int logicalDir, uint8_t speedLevel);
  std::pair<float,bool> moveBy(int logicalDir, float mm, uint8_t speedLevel);

private:
  enum class Mode { IDLE, CONTINUOUS, GOTO, HOMING };
  // Neuer PRE_BACKOFF Zustand: fahre 3mm runter vor schnellem Homing
  enum class HomeState { IDLE, PRE_BACKOFF, FAST, BACKOFF, DONE };

  inline long  mmToSteps(float mm) const;
  inline float stepsToMm(long st) const;
  inline int   logicalToMotorDir(int logicalDir) const;
  inline float stepsToLogicalMm(long deltaSteps) const;
  static inline float ema_update(float prev, float sample, float alpha);

  void setupDriver_();
  void beginHomePhase_(HomeState next, float speedHz, int logicalDir);
  void updateHoming_();
  void enforceSoftLimits_();

  // Starte eine kontinuierliche Bewegung in gegebener logischer Richtung mit gegebener Geschwindigkeit.
  // Wenn keepMode true ist, wird der mode_ nicht geändert (nützlich beim Homing),
  // andernfalls wird Mode::CONTINUOUS gesetzt.
  void startContinuousLogical_(float speedHz, int logicalDir, bool keepMode = false);
  void moveRelativeLogicalMM_(float mmLogical, float maxHz);
  void moveRelativeMM_motor_(float mm, float maxHz);
  void moveToAbsMM_(float mm, float maxHz);

private:
  // Konfig
  StepperPins       p_;
  StepperKinematics k_;
  StepperLimits     l_;
  TMC2209Config     c_;
  HomingParams      h_;
  SpeedTable        s_;

  // Laufzeit
  TMC2209Stepper          driver_;
  FastAccelStepperEngine  engine_;
  FastAccelStepper*       fas_ = nullptr;
  bool                    debug_ = false;

  Mode      mode_         = Mode::IDLE;
  HomeState hState_       = HomeState::IDLE;
  uint32_t  homePhaseMs_  = 0;
  uint32_t  sgLastPoll_   = 0;
  uint32_t  sgCalibMs_    = 0;
  uint32_t  sgSamples_    = 0;
  float     sgBaseline_   = 0.0f;
  float     sgAvg_        = 0.0f;
  long      softMinSteps_ = 0;
  long      softMaxSteps_ = 0;
  long      homingStartSteps_ = 0;

  int       AXIS_UP_DIR_  = -1; // Standard: oben ist Motor-
  int       homingDirLogical_ = +1; // Homing in logischer +1 Richtung
  bool      lastOpDone_   = false;
  bool      homingFinished_ = false;
  bool      homingBackoffActive_ = false;

  // Debug: Positions-Logs drosseln
  uint32_t  lastDebugPosMs_ = 0;
  uint16_t  debugMoveLogIntervalMs_ = 100;

  // DIAG (Fehler) – ISR
  static volatile bool diagTriggered_;
  static void IRAM_ATTR onDiagRiseISR_();

  // intern: Homing-Backoff ohne Mode-Wechsel
  void homeMoveRelativeMM_(float mmLogical, float maxHz);
};
