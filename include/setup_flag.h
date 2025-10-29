#pragma once

#include <Arduino.h>

// Kleines Hilfsmodul zur Persistenz des Setup-Status in LittleFS.
// Implementierung in src/setup_flag.cpp

namespace setup_flag {

// Pfad der JSON-Statusdatei im LittleFS
extern const char* kPath;

// true wenn Setup abgeschlossen (Datei vorhanden mit done=true)
bool is_done();

// Setzt den Status und speichert ihn. Gibt false bei FS-Fehler zurück.
bool set_done(bool done);

}

