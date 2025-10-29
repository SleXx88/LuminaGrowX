#include "../include/setup_flag.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

namespace setup_flag {

const char* kPath = "/cfg/setup.json";

static bool readText(const char* path, String& out) {
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return false;
  out = f.readString();
  f.close();
  return true;
}

static bool writeText(const char* path, const String& s) {
  LittleFS.mkdir("/cfg");
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) return false;
  f.print(s);
  f.close();
  return true;
}

bool is_done() {
  // FS sollte bereits gemountet sein, aber wir versuchen es non-destruktiv
  LittleFS.begin(false, "/littlefs", 10, "littlefs");
  String s; if (!readText(kPath, s)) return false;
  JsonDocument doc; if (deserializeJson(doc, s)) return false;
  return doc["done"].as<bool>();
}

bool set_done(bool done) {
  // FS sicherstellen (nicht formatieren)
  if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) return false;
  JsonDocument doc; doc["done"] = done; doc["ts"] = (uint64_t)millis();
  String out; serializeJson(doc, out);
  return writeText(kPath, out);
}

} // namespace setup_flag

