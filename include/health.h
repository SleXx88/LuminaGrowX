// Zentrale, sehr leichte Health-Status-Verwaltung
#pragma once

#include <Arduino.h>

namespace health {

struct HealthModules {
  bool i2c0_ok = true;     // Wire
  bool i2c1_ok = true;     // Wire1
  bool sht_in_ok = false;
  bool sht_out_ok = false;
  bool dac_ok = false;     // GP8211
  bool fan_ok = true;      // PWM grundsätzlich da; Fehler hier selten
  bool rtc_ok = false;     // DS3231
  bool tof_ok = false;     // VL53L1/VL53L0 (ToF)
  bool fs_ok = true;       // LittleFS
};

struct HealthState {
  HealthModules mod;
  // Ein kurzer Sammeltext (optional) für schnelle Anzeige
  String message;
  // Flag, ob zuletzt die Regelung pausiert wurde (Info für UI)
  bool control_paused = false;
};

inline HealthState& state() {
  static HealthState s; return s;
}

// Kritisch für die Regelung: Innen- und Außen-SHT müssen verfügbar sein.
inline bool critical_ok() {
  const auto& m = state().mod;
  return m.sht_in_ok && m.sht_out_ok;
}

// Mindestens ein Modul fehlerhaft?
inline bool any_error() {
  const auto& m = state().mod;
  return !m.i2c0_ok || !m.i2c1_ok || !m.sht_in_ok || !m.sht_out_ok || !m.dac_ok || !m.fan_ok || !m.rtc_ok || !m.tof_ok || !m.fs_ok;
}

// Hilfsfunktionen zum Setzen inkl. kurzer Message
inline void set_sht_in(bool ok, const String& msg = "")  { state().mod.sht_in_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_sht_out(bool ok, const String& msg = "") { state().mod.sht_out_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_dac(bool ok, const String& msg = "")     { state().mod.dac_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_rtc(bool ok, const String& msg = "")     { state().mod.rtc_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_tof(bool ok, const String& msg = "")     { state().mod.tof_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_fs(bool ok, const String& msg = "")      { state().mod.fs_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_i2c0(bool ok, const String& msg = "")    { state().mod.i2c0_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_i2c1(bool ok, const String& msg = "")    { state().mod.i2c1_ok = ok; if (!ok && msg.length()) state().message = msg; }
inline void set_fan(bool ok, const String& msg = "")     { state().mod.fan_ok = ok; if (!ok && msg.length()) state().message = msg; }

} // namespace health

