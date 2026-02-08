#include "../include/setup_flag.h"
#include <Preferences.h>

namespace setup_flag {

// NVS Namespace für Setup-Daten
const char* kPrefNamespace = "setup";
const char* kPrefKeyDone = "done";

bool is_done() {
  Preferences prefs;
  // Namespace öffnen (read-only)
  if (!prefs.begin(kPrefNamespace, true)) {
    return false;
  }
  bool done = prefs.getBool(kPrefKeyDone, false);
  prefs.end();
  return done;
}

bool set_done(bool done) {
  Preferences prefs;
  // Namespace öffnen (read-write)
  if (!prefs.begin(kPrefNamespace, false)) {
    return false;
  }
  size_t bytesWritten = prefs.putBool(kPrefKeyDone, done);
  prefs.end();
  return bytesWritten > 0;
}

} // namespace setup_flag

