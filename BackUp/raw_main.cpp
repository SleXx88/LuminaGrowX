#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>

/* ============================================================
   PIN-BELEGUNG  (ANPASSEN!)
   ------------------------------------------------------------
   - PDN_UART vom TMC2209 an ESP32 UART (gekreuzt): TX->PDN, RX<-PDN
   ============================================================ */
#define PIN_STEP   12    // Step-Pin zum TMC2209
#define PIN_DIR    11    // Richtung
#define PIN_EN     10    // Enable (LOW = aktiv, je nach Board)
#define PIN_DIAG   13    // DIAG-Ausgang (Fehler), nicht für StallGuard!

#define TMC_UART_TX   17 // MCU TX -> TMC PDN_UART (RX)
#define TMC_UART_RX   18 // MCU RX <- TMC PDN_UART (TX)
#define TMC_UART_PORT Serial1
#define TMC_BAUD      115200

/* ============================================================
   MECHANIK / KINEMATIK
   Beispiel: TR8x8, NEMA17 (200 Schritte/U), 1/16 Microstepping
   ============================================================ */
static const uint16_t STEPS_PER_REV = 200;
static const uint8_t  MICROSTEPS    = 16;
static const float    LEAD_MM       = 8.0f; // Gewindesteigung (mm/Umdrehung)
static const float    STEPS_PER_MM  = (STEPS_PER_REV * MICROSTEPS) / LEAD_MM;

// Software-Fahrbereich (mm)
static const float    MAX_TRAVEL_MM  = 440.0f;  // an Hardware anpassen
static const float    HOME_OFFSET_MM = 0.0f;    // Offset nach Homing

/* ============================================================
   TMC2209-TREIBER
   ============================================================ */
#define R_SENSE        0.11f     // Shunt (Ohm), Adafruit i.d.R. 0.11
#define DRIVER_ADDR    0b00      // Einzelner Treiber
TMC2209Stepper driver(&TMC_UART_PORT, R_SENSE, DRIVER_ADDR);

// Standardströme konservativ, damit das Board kühl bleibt
static uint16_t IRUN_mA  = 500;  // Fahrstrom (mA, RMS)
static uint16_t IHOLD_mA = 100;  // Haltestrom (mA, RMS)
static uint8_t  sg_thrs  = 240;  // StallGuard-Empfindlichkeit 0..255 default = 120

/* ============================================================
   HOMING (UART-basiertes StallGuard)
   ============================================================ */
enum class HomeState { IDLE, FAST, BACKOFF, SLOW, DONE };
volatile bool diagTriggered = false;

// Geschwindigkeiten & Verhalten (deine aktuell funktionierenden Werte)
static const float HOME_SPEED_FAST_HZ = 2500.0f; // schnell (Schritte/s)
static const float HOME_SPEED_SLOW_HZ = 2000.0f; // langsam (Schritte/s)
static const float BACKOFF_MM         = 3.0f;    // Rückzug nach Stall
static const uint32_t SG_IGNORE_MS    = 50;     // Stall in ersten 100ms ignorieren

// UART-basierte StallGuard-Parameter
static const uint32_t SG_POLL_INTERVAL_MS = 10;    // Polling-Intervall
static const uint32_t SG_BASELINE_MS      = 200;   // Kalibrierzeit am Phasenanfang
static const uint16_t SG_ABS_THR          = 0;     // keine absolute Untergrenze
static const float    SG_DROP_PCT         = 0.35f; // 40% Einbruch vs. Baseline => Stall

// Mindestweg bis zur Auswertung (mm)
static const float MIN_STALL_DISTANCE_MM = 3.0f;

// Laufende Variablen für SG_RESULT-Auswertung
static uint32_t sgLastPollMs   = 0;
static uint32_t sgCalibStartMs = 0;
static uint32_t sgSamples      = 0;
static float    sgBaseline     = 0.0f;
static float    sgAvg          = 0.0f;

// Richtungs-Mapping:
// AXIS_UP_DIR = welcher Motor-Sign (+1/-1) ist physisch "oben"?
// Du hattest berichtet, dass "+" vorher nach unten fuhr -> -1 = oben.
static int AXIS_UP_DIR = -1;       // -1: Motor-Sign -1 ist "oben"; +1: Motor-Sign +1 ist "oben"
// Homing-Richtung (logisch): +1 = nach oben, -1 = nach unten
static int homingDirLogical = +1;  // bei dir Homing nach unten

/* ============================================================
   LAUFZEIT-OBJEKTE
   ============================================================ */
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

enum class Mode { IDLE, CONTINUOUS, GOTO, HOMING };
Mode mode = Mode::IDLE;
HomeState hState = HomeState::IDLE;
uint32_t homingPhaseStartMs = 0;

// Softlimits in Schritten
static long SOFT_MIN_STEPS = 0;
static long SOFT_MAX_STEPS = 0;

// Geschwindigkeitsstufen (Schritte/s) für '0'..'5'
static const float SPEED_PRESETS_HZ[] = { 150.0f, 400.0f, 800.0f, 1200.0f, 1500.0f, 4000.0f };
uint8_t speedLevel = 1; // Start: Stufe 1 = 400 Hz
float   contSpeedHz = SPEED_PRESETS_HZ[speedLevel];

// Für Mindestfahrstrecke im Homing
static long homingStartSteps = 0;

/* Reporting für manuelle Fahrten (+ / -) */
static bool     motionReportActive = false;
static long     motionStartSteps   = 0;
static uint32_t motionLastReportMs = 0;

// ------------------------------------------------------------
inline long  mmToSteps(float mm) { return lroundf(mm * STEPS_PER_MM); }
inline float stepsToMm(long st)  { return (float)st / STEPS_PER_MM; }

// Exponentieller gleitender Mittelwert (EMA)
static inline float ema_update(float prev, float sample, float alpha) {
  return alpha * sample + (1.0f - alpha) * prev;
}

static void IRAM_ATTR onDiagRise() { diagTriggered = true; }

// Ströme korrekt in mA setzen
void setCurrents_mA(uint16_t run_mA, uint16_t hold_mA) {
  IRUN_mA  = run_mA;
  IHOLD_mA = hold_mA;
  float mult = (IRUN_mA == 0) ? 0.0f : (float)IHOLD_mA / (float)IRUN_mA;
  if (mult < 0.0f) mult = 0.0f;
  driver.rms_current(IRUN_mA, mult);
  driver.hold_multiplier(mult);
}

/* Softlimits erzwingen: stoppt Motor und klemmt Position */
void enforceSoftLimits() {
  long pos = stepper.currentPosition();
  if (pos < SOFT_MIN_STEPS) {
    stepper.setCurrentPosition(SOFT_MIN_STEPS);
    mode = Mode::IDLE;
    stepper.stop();
  } else if (pos > SOFT_MAX_STEPS) {
    stepper.setCurrentPosition(SOFT_MAX_STEPS);
    mode = Mode::IDLE;
    stepper.stop();
  }
}

/* ============================================================
   TMC2209 INITIALISIERUNG
   ============================================================ */
void setupDriver() {
  TMC_UART_PORT.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

  driver.begin();
  driver.toff(5);
  driver.blank_time(24);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  // StallGuard im StealthChop
  driver.en_spreadCycle(false);
  driver.TPWMTHRS(0);          // kein Umschalten in SpreadCycle
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(sg_thrs);

  // CoolStep-Parameter
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.seimin(false);

  // Ströme (mA)
  setCurrents_mA(IRUN_mA, IHOLD_mA);

  // Enable/Treiber aktivieren
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);   // LOW = aktiv (je nach Breakout)

  // DIAG-Pin mit Pullup (Open-Drain am Breakout) – nur Fehlerdiagnose
  pinMode(PIN_DIAG, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_DIAG), onDiagRise, RISING);
}

/* ============================================================
   RICHTUNGS-MAPPING
   ============================================================ */
// LOGISCH (+1=oben, -1=unten) -> MOTOR-Sign (+1/-1)
inline int logicalToMotorDir(int logicalDir) {
  return (logicalDir >= 0) ? AXIS_UP_DIR : -AXIS_UP_DIR;
}
// Hilfsfunktion: Steps -> logische mm (damit Δ positiv bei Fahrt nach oben)
inline float stepsToLogicalMm(long deltaSteps) {
  // ein positiver MOTOR-Step entspricht logischer mm = +1 * AXIS_UP_DIR
  return stepsToMm(deltaSteps) * (float)((AXIS_UP_DIR >= 0) ? +1 : -1);
}

/* ============================================================
   BEWEGUNG
   ============================================================ */
void startContinuousLogical(float speedHz, int logicalDir) {
  mode = Mode::CONTINUOUS;
  int motorSign = logicalToMotorDir(logicalDir);
  float sps = speedHz * (float)motorSign;
  stepper.setSpeed(sps);

  // Reporting starten
  motionReportActive = true;
  motionStartSteps   = stepper.currentPosition();
  motionLastReportMs = 0;
  Serial.printf("[MOVE] Start: Richtung=%s, pos=%.3f mm\n",
                (logicalDir >= 0) ? "UP" : "DOWN",
                stepsToMm(stepper.currentPosition()));
}

void stopMotion() {
  // vor dem Stop finalen Weg ausgeben
  if (motionReportActive) {
    long dSteps = stepper.currentPosition() - motionStartSteps;
    float dMm   = stepsToLogicalMm(dSteps);
    Serial.printf("[MOVE] Stop: Δ=%.3f mm, pos=%.3f mm\n",
                  dMm, stepsToMm(stepper.currentPosition()));
  }
  motionReportActive = false;

  mode = Mode::IDLE;
  stepper.stop();
  stepper.setSpeed(0);
}

// LOGISCHE relative Bewegung in mm ( + = nach oben, - = nach unten )
void moveRelativeLogicalMM(float mmLogical, float maxHz) {
  mode = Mode::GOTO;
  stepper.setMaxSpeed(maxHz);
  stepper.setAcceleration(maxHz * 2.0f);
  long stepsMotor = mmToSteps(mmLogical) * ((AXIS_UP_DIR >= 0) ? +1 : -1);
  stepper.move(stepsMotor);
}

void moveRelativeMM(float mm, float maxHz) {
  // Beibehaltung alter API (motor-koordiniert). Für Homing NICHT verwenden.
  mode = Mode::GOTO;
  stepper.setMaxSpeed(maxHz);
  stepper.setAcceleration(maxHz * 2.0f);
  stepper.move(mmToSteps(mm));
}

void moveToAbsMM(float mm, float maxHz) {
  if (mm < 0) mm = 0;
  if (mm > MAX_TRAVEL_MM) mm = MAX_TRAVEL_MM;
  mode = Mode::GOTO;
  stepper.setMaxSpeed(maxHz);
  stepper.setAcceleration(maxHz * 2.0f);
  stepper.moveTo(mmToSteps(mm));
}

/* ============================================================
   HOMING-ABLÄUFE
   ============================================================ */
void beginHomePhase(HomeState next, float speedHz, int logicalDir) {
  hState = next;
  homingPhaseStartMs = millis();

  // UART-basierte StallGuard-Kalibrierung zurücksetzen
  sgCalibStartMs = homingPhaseStartMs;
  sgSamples      = 0;
  sgBaseline     = 0.0f;
  sgAvg          = 0.0f;
  sgLastPollMs   = 0;

  if (next == HomeState::FAST) {
    homingStartSteps = stepper.currentPosition();
  }

  diagTriggered = false;

  startContinuousLogical(speedHz, logicalDir);

  const char *phaseName = "";
  switch (next) {
    case HomeState::FAST:    phaseName = "FAST"; break;
    case HomeState::SLOW:    phaseName = "SLOW"; break;
    case HomeState::BACKOFF: phaseName = "BACKOFF"; break;
    case HomeState::DONE:    phaseName = "DONE"; break;
    default:                 phaseName = "?"; break;
  }
  Serial.printf("[HOME] Starte Phase %s mit %.0f Hz, logDir=%d\n",
                phaseName, speedHz, logicalDir);
}

void startHoming() {
  // kurzer Rückzug vom potenziellen Anschlag: immer weg vom Endstop,
  // d.h. Rückzug in GEGENRICHTUNG zur Homing-Logik
  moveRelativeLogicalMM(BACKOFF_MM * (float)(-homingDirLogical), HOME_SPEED_SLOW_HZ);
  while (stepper.distanceToGo() != 0) stepper.run();

  // schnelle Annäherung in homingDirLogical
  beginHomePhase(HomeState::FAST, HOME_SPEED_FAST_HZ, homingDirLogical);
  mode = Mode::HOMING;
}

// UART-basierte Homing-Auswertung
void updateHoming() {
  if (mode != Mode::HOMING) return;

  uint32_t now = millis();
  bool sgWindowOver = (now - homingPhaseStartMs) > SG_IGNORE_MS;

  // SG_RESULT pollen
  if (now - sgLastPollMs >= SG_POLL_INTERVAL_MS) {
    sgLastPollMs = now;
    uint16_t sg_raw = driver.SG_RESULT();

    // Baseline während der ersten SG_BASELINE_MS aufbauen
    if ((now - sgCalibStartMs) <= SG_BASELINE_MS) {
      sgSamples++;
      sgBaseline = (sgBaseline * (sgSamples - 1) + (float)sg_raw) / (float)sgSamples;
      if (sgSamples < 3) sgAvg = sgBaseline;
    } else {
      sgAvg = ema_update(sgAvg, (float)sg_raw, 0.25f);
    }

    // Debug alle 100 ms
    static uint32_t lastDbg = 0;
    if (now - lastDbg > 100) {
      lastDbg = now;
      long posSteps = stepper.currentPosition();
      Serial.printf("[HOME-DEBUG] state=%d pos=%.2fmm sg_raw=%u sgAvg=%.1f base=%.1f\n",
                    (int)hState, stepsToMm(posSteps), sg_raw, sgAvg, sgBaseline);
    }
  }

  // Stall-Entscheidung: relativer Einbruch ODER unter absolute Schwelle
  bool stallDetected = false;
  if (sgWindowOver && (sgSamples > 5)) {
    float dropThreshold = fmaxf(sgBaseline * (1.0f - SG_DROP_PCT), (float)SG_ABS_THR);
    if (sgAvg <= dropThreshold) stallDetected = true;
  }
  // Mindestfahrstrecke abwarten
  if (stallDetected) {
    long currentSteps = stepper.currentPosition();
    long deltaSteps   = labs(currentSteps - homingStartSteps);
    bool enoughTravel = (deltaSteps >= mmToSteps(MIN_STALL_DISTANCE_MM));
    if (!enoughTravel) stallDetected = false;
  }

  switch (hState) {
    case HomeState::FAST:
      if (stallDetected) {
        Serial.printf("[HOME] SG(UART) triggert im FAST-Modus bei %.2f mm (sgAvg=%.1f, base=%.1f)\n",
                      stepsToMm(stepper.currentPosition()), sgAvg, sgBaseline);
        // 1) SOFORT STOPPEN
        stopMotion();
        // 2) 3 mm in GEGENRICHTUNG fahren
        moveRelativeLogicalMM(BACKOFF_MM * (float)(-homingDirLogical), HOME_SPEED_FAST_HZ);
        while (stepper.distanceToGo() != 0) stepper.run();
        // 3) DIESEN PUNKT ALS NULLPUNKT setzen
        stepper.setCurrentPosition(mmToSteps(HOME_OFFSET_MM));
        hState = HomeState::DONE;
        mode   = Mode::IDLE;
        Serial.println(F("[HOME] Homing abgeschlossen (Nullpunkt nach Backoff gesetzt)"));
      }
      break;

    case HomeState::SLOW:
      // SLOW-Phase wird NICHT mehr genutzt (einphasiges Homing)
      break;

    default:
      break;
  }
}

/* ============================================================
   SERIELLE KOMMANDOS
   ------------------------------------------------------------
   +              -> kontinuierlich NACH OBEN (logisch +1) + Live-Δ
   -              -> kontinuierlich NACH UNTEN (logisch -1) + Live-Δ
   s              -> stoppen (finale Δ-Ausgabe)
   0..5           -> Geschwindigkeitsstufe
   h              -> Homing starten (in aktueller homingDirLogical)
   h+             -> Homing nach oben (setzt homingDirLogical=+1 und startet)
   h-             -> Homing nach unten (setzt homingDirLogical=-1 und startet)
   updir +        -> definiert Motor-Sign +1 als "oben"
   updir -        -> definiert Motor-Sign -1 als "oben"
   updir?         -> zeigt aktuell gültiges Mapping
   p <mm>         -> relativ verfahren (motorbezogen; Homing nutzt moveRelativeLogicalMM)
   g <mm>         -> absolut verfahren (0..MAX_TRAVEL_MM)
   sg <0..255>    -> SGTHRS (wirkt nur auf DIAG)
   cur <run> <halt> -> Ströme (mA)
   ms <8|16|32|64>  -> Microstepping
   stat           -> Status
   diag           -> DIAG-Pin & SG_RESULT (Debug)
   ?              -> Hilfe
   ============================================================ */
String rx;

void printHelp() {
  Serial.println(F("Befehle: +  -  s  0..5  h  h+  h-  updir +  updir -  updir?  p <mm>  g <mm>  sg <0..255>  cur <lauf> <halt>  ms <8|16|32|64>  stat  diag  ?"));
}

void printStatus() {
  Serial.printf("[STATUS] pos=%.3f mm  speed=%.0f Hz  SGTHRS=%u  IRUN=%umA  IHOLD=%umA  AXIS_UP_DIR=%d  homingDir=%+d\n",
                stepsToMm(stepper.currentPosition()),
                fabs(stepper.speed()),
                sg_thrs, IRUN_mA, IHOLD_mA,
                AXIS_UP_DIR, homingDirLogical);
}

void setSpeedLevel(uint8_t lvl) {
  if (lvl >= sizeof(SPEED_PRESETS_HZ)/sizeof(SPEED_PRESETS_HZ[0])) return;
  speedLevel = lvl;
  contSpeedHz = SPEED_PRESETS_HZ[speedLevel];
  if (mode == Mode::CONTINUOUS) {
    // Behalte aktuelle logische Richtung anhand des Motorzeichens bei
    int currentMotorSign = (stepper.speed() >= 0 ? +1 : -1);
    int logicalDir = (currentMotorSign == AXIS_UP_DIR) ? +1 : -1;
    startContinuousLogical(contSpeedHz, logicalDir);
  }
  Serial.printf("[GESCHWINDIGKEIT] Stufe=%u -> %.0f Hz\n", speedLevel, contSpeedHz);
}

void processLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Richtungsfahrten (logisch)
  if (line == "+") { startContinuousLogical(contSpeedHz, +1); return; }
  if (line == "-") { startContinuousLogical(contSpeedHz, -1); return; }

  if (line == "s") { stopMotion(); return; }

  if (line == "h")  { startHoming(); return; }
  if (line == "h+") { homingDirLogical = +1; startHoming(); return; }
  if (line == "h-") { homingDirLogical = -1; startHoming(); return; }

  if (line == "updir?") {
    Serial.printf("[UPDIR] Motor-Sign %d entspricht OBEN\n", AXIS_UP_DIR);
    return;
  }
  if (line.startsWith("updir ")) {
    String v = line.substring(6);
    v.trim();
    if (v == "+") { AXIS_UP_DIR = +1; Serial.println(F("[UPDIR] +1 ist OBEN")); }
    else if (v == "-") { AXIS_UP_DIR = -1; Serial.println(F("[UPDIR] -1 ist OBEN")); }
    else Serial.println(F("[FEHLER] updir <+|->"));
    return;
  }

  if (line == "stat") { printStatus(); return; }
  if (line == "diag") {
    bool d = digitalRead(PIN_DIAG);
    uint16_t sg = driver.SG_RESULT();
    Serial.printf("[DIAG] Pin=%d  SG_RESULT=%u  SGTHRS=%u  AXIS_UP_DIR=%d\n", d, sg, sg_thrs, AXIS_UP_DIR);
    return;
  }
  if (line == "?") { printHelp(); return; }

  // 0..5: Geschwindigkeitsstufe
  if (line.length() == 1 && isDigit(line[0])) {
    setSpeedLevel(line[0] - '0'); return;
  }

  // p <mm> : relativ verfahren (motorbezogen, nicht logisch)
  if (line.startsWith("p ")) {
    float mm = line.substring(2).toFloat();
    moveRelativeMM(mm, fmaxf(400.0f, fabsf(contSpeedHz)));
    return;
  }

  // g <mm> : absolut verfahren
  if (line.startsWith("g ")) {
    float mm = line.substring(2).toFloat();
    moveToAbsMM(mm, fmaxf(400.0f, fabsf(contSpeedHz)));
    return;
  }

  // sg <0..255> : Empfindlichkeit (wirkt auf DIAG, für UART-Logik nicht relevant)
  if (line.startsWith("sg ")) {
    int v = line.substring(3).toInt();
    if (v < 0) v = 0; if (v > 255) v = 255;
    sg_thrs = (uint8_t)v;
    driver.SGTHRS(sg_thrs);
    Serial.printf("[SGTHRS] = %d\n", sg_thrs);
    return;
  }

  // cur <run> <hold> : Ströme setzen (mA)
  if (line.startsWith("cur ")) {
    int sp1 = line.indexOf(' ', 4);
    if (sp1 > 0) {
      uint16_t run_mA  = (uint16_t)line.substring(4, sp1).toInt();
      uint16_t hold_mA = (uint16_t)line.substring(sp1 + 1).toInt();
      setCurrents_mA(run_mA, hold_mA);
      Serial.printf("[STROM] IRUN=%umA  IHOLD=%umA  (Faktor=%.2f)\n",
                    IRUN_mA, IHOLD_mA,
                    (IRUN_mA == 0) ? 0.0f : (float)IHOLD_mA / (float)IRUN_mA);
    }
    return;
  }

  // ms <8|16|32|64>
  if (line.startsWith("ms ")) {
    int ms = line.substring(3).toInt();
    if (ms == 8 || ms == 16 || ms == 32 || ms == 64) {
      driver.microsteps(ms);
      Serial.printf("[MICROSCHRITTE] = %d\n", ms);
    } else {
      Serial.println(F("[FEHLER] Ungültige Mikroschritte (8/16/32/64)"));
    }
    return;
  }

  Serial.println(F("[FEHLER] Unbekannter Befehl. '?' für Hilfe."));
}

/* ============================================================
   ARDUINO-LAUFZEIT
   ============================================================ */
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("\n[TMC2209 Sensorloses Homing – ESP32-S3]"));

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_EN,   OUTPUT);

  // AccelStepper-Basiswerte (für GOTO)
  stepper.setMaxSpeed(2500);      // obere Grenze GOTO (Schritte/s)
  stepper.setAcceleration(5000);  // Beschleunigung (Schritte/s²)

  setupDriver();

  // Softlimits berechnen
  SOFT_MIN_STEPS = 0;
  SOFT_MAX_STEPS = mmToSteps(MAX_TRAVEL_MM);

  Serial.println(F("Tipp: 'updir -' oder 'updir +' setzt, welche Motor-Richtung 'oben' ist."));
  printHelp();
  printStatus();
}

void loop() {
  // Serielle Eingabe verarbeiten
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      processLine(rx);
      rx = "";
    } else {
      rx += c;
    }
  }

  // Bewegungssteuerung je nach Modus
  switch (mode) {
    case Mode::CONTINUOUS:
      stepper.runSpeed();   // konstante Schrittfrequenz
      break;
    case Mode::GOTO:
      stepper.run();        // beschleunigt bis Ziel
      if (stepper.distanceToGo() == 0) mode = Mode::IDLE;
      break;
    case Mode::HOMING:
      stepper.runSpeed();   // Homing fährt mit konstanter Frequenz
      updateHoming();       // SG_RESULT auswerten
      break;
    case Mode::IDLE:
    default:
      break;
  }

  // Live-Δ-Ausgabe bei manueller Fahrt
  if (mode == Mode::CONTINUOUS && motionReportActive) {
    uint32_t now = millis();
    if (now - motionLastReportMs >= 200) {
      motionLastReportMs = now;
      long dSteps = stepper.currentPosition() - motionStartSteps;
      float dMm   = stepsToLogicalMm(dSteps);
      Serial.printf("[MOVE] Δ=%.3f mm, pos=%.3f mm\n",
                    dMm, stepsToMm(stepper.currentPosition()));
    }
  }

  // Softlimits nur außerhalb des Homings anwenden
  if (mode != Mode::HOMING) {
    enforceSoftLimits();
  }
}
