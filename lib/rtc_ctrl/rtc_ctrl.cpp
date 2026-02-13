#include "rtc_ctrl.h"

/* ===================== Hilfsfunktionen BCD ===================== */
uint8_t RTC_Ctrl::bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }
uint8_t RTC_Ctrl::bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

/* ===================== Konstruktor / Init ===================== */
RTC_Ctrl::RTC_Ctrl() {}

bool RTC_Ctrl::begin(TwoWire& w, uint8_t i2cAddr) {
  wire_ = &w;
  addr_ = i2cAddr;
  _error = false;
  return isConnected();
}

bool RTC_Ctrl::isConnected() {
  if (_error && (millis() - _lastErrorMs < 2000)) return false; // 2s Backoff
  wire_->beginTransmission(addr_);
  if (wire_->endTransmission() == 0) {
    _error = false;
    return true;
  }
  _error = true; _lastErrorMs = millis();
  return false;
}

/* ===================== DS3231 rohes Lesen/Schreiben (UTC) ===================== */
// DS3231 Register (ab 0x00): Sekunden, Minuten, Stunden, Wochentag, Datum, Monat, Jahr
// Wir ignorieren Wochentag, nutzen 24h-Format (Bit6=0).
bool RTC_Ctrl::readRTC_UTC(DateTime& dt) {
  if (_error && (millis() - _lastErrorMs < 2000)) return false; // 2s Backoff

  wire_->beginTransmission(addr_);
  wire_->write(0x00); // ab Sekunden
  if (wire_->endTransmission(false) != 0) {
     _error = true; _lastErrorMs = millis();
     return false;
  }

  uint8_t n = wire_->requestFrom((int)addr_, 7);
  if (n != 7) {
     _error = true; _lastErrorMs = millis();
     return false;
  }
  
  _error = false; // Success

  uint8_t ss = wire_->read();
  uint8_t mm = wire_->read();
  uint8_t hh = wire_->read();
  (void)wire_->read(); // Wochentag überspringen
  uint8_t d  = wire_->read();
  uint8_t m  = wire_->read();
  uint8_t y  = wire_->read();

  dt.second = bcd2bin(ss & 0x7F);
  dt.minute = bcd2bin(mm);
  // 24h-Format
  if (hh & 0x40) return false; // 12h-Modus -> unerwartet
  dt.hour   = bcd2bin(hh & 0x3F);
  dt.day    = bcd2bin(d);
  dt.month  = bcd2bin(m & 0x1F);
  dt.year   = 2000 + bcd2bin(y);
  return true;
}

bool RTC_Ctrl::writeRTC_UTC(const DateTime& dt) {
  // Eingaben grob prüfen
  if (dt.year < 2000 || dt.year > 2099) return false;
  if (dt.month < 1 || dt.month > 12) return false;
  if (dt.day < 1 || dt.day > 31) return false;
  if (dt.hour < 0 || dt.hour > 23) return false;
  if (dt.minute < 0 || dt.minute > 59) return false;
  if (dt.second < 0 || dt.second > 59) return false;

  wire_->beginTransmission(addr_);
  wire_->write(0x00);
  wire_->write(bin2bcd((uint8_t)dt.second));
  wire_->write(bin2bcd((uint8_t)dt.minute));
  wire_->write(bin2bcd((uint8_t)dt.hour));      // 24h
  wire_->write(0x01);                           // Wochentag Dummy (1=Montag egal)
  wire_->write(bin2bcd((uint8_t)dt.day));
  wire_->write(bin2bcd((uint8_t)dt.month));     // Jahrhundertbit nicht genutzt
  wire_->write(bin2bcd((uint8_t)(dt.year - 2000)));
  return (wire_->endTransmission() == 0);
}

/* ===================== Kalender-/DST-Helfer ===================== */
bool RTC_Ctrl::isLeapYear(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
int RTC_Ctrl::daysInMonth(int y, int m) {
  static const int dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return dm[1] + (isLeapYear(y) ? 1 : 0);
  return dm[m-1];
}
// Sakamoto-Algorithmus: 0 = Sonntag .. 6 = Samstag
int RTC_Ctrl::dayOfWeek(int y, int m, int d) {
  static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}
int RTC_Ctrl::lastSundayOfMonth(int y, int m) {
  int last = daysInMonth(y, m);
  int dow  = dayOfWeek(y, m, last); // 0=So ..6=Sa
  int offset = dow;                  // wie viele Tage zurück bis Sonntag
  return last - offset;
}

// Prüft, ob *UTC*-Zeitpunkt in EU-DST liegt.
// Regel: DST aktiv von 01:00 UTC letzter Sonntag März bis 01:00 UTC letzter Sonntag Oktober.
bool RTC_Ctrl::isDST_EU_fromUTC(const DateTime& utc) {
  int y = utc.year;

  int lsm  = lastSundayOfMonth(y, 3); // März
  int lso  = lastSundayOfMonth(y, 10);// Oktober

  // Start und Ende als (Monat, Tag, Stunde) Tripel
  // Start: März, letzter Sonntag, 01:00 UTC
  // Ende : Oktober, letzter Sonntag, 01:00 UTC (exklusiv)
  // Vergleiche lexikographisch
  auto afterOrEqual = [&](int M, int D, int H) {
    if (utc.month > M) return true;
    if (utc.month < M) return false;
    if (utc.day > D) return true;
    if (utc.day < D) return false;
    return (utc.hour >= H);
  };
  auto before = [&](int M, int D, int H) {
    if (utc.month < M) return true;
    if (utc.month > M) return false;
    if (utc.day < D) return true;
    if (utc.day > D) return false;
    return (utc.hour < H);
  };

  bool inSummer = afterOrEqual(3, lsm, 1) && before(10, lso, 1);
  return inSummer;
}

void RTC_Ctrl::addHours(DateTime& t, int dh) {
  if (dh == 0) return;
  int h = t.hour + dh;
  while (h >= 24) {
    h -= 24;
    // Tag +1
    t.day++;
    if (t.day > daysInMonth(t.year, t.month)) {
      t.day = 1;
      t.month++;
      if (t.month > 12) {
        t.month = 1;
        t.year++;
      }
    }
  }
  while (h < 0) {
    h += 24;
    // Tag -1
    t.day--;
    if (t.day < 1) {
      t.month--;
      if (t.month < 1) {
        t.month = 12;
        t.year--;
      }
      t.day = daysInMonth(t.year, t.month);
    }
  }
  t.hour = h;
}

/* ===================== UTC <-> Lokal (Europa/Berlin) ===================== */
RTC_Ctrl::DateTime RTC_Ctrl::utcToLocal(const DateTime& utc) {
  DateTime local = utc;
  // Grundoffset CET = UTC+1
  addHours(local, 1);
  // Zusätzliche Stunde, falls DST
  if (isDST_EU_fromUTC(utc)) {
    addHours(local, 1);
  }
  return local;
}

RTC_Ctrl::DateTime RTC_Ctrl::localToUTC(const DateTime& localIn) {
  // Umkehrung: Wir müssen wissen, ob zum lokalen Zeitpunkt DST gilt.
  // Strategie: grob UTC schätzen (lokal -1h), dann prüfen ob DST aktiv wäre,
  // ggf. eine weitere Stunde abziehen.
  DateTime utc = localIn;
  // CET Grundoffset zurücknehmen
  addHours(utc, -1);

  // Prüfe, ob der zugehörige (noch ungenaue) UTC-Zeitpunkt innerhalb der DST liegt.
  // Näherung reicht, da Differenz max. 1 Stunde ist.
  if (isDST_EU_fromUTC(utc)) {
    addHours(utc, -1);
  }
  return utc;
}

/* ===================== Parser / Formatter ===================== */
// Erwartetes Format: "DD-MM-YYYY HH-MM-SS" (lokal)
bool RTC_Ctrl::parseLocalString(const String& s, DateTime& outLocal) {
  int DD, MM, YYYY, hh, mm, ss;
  // sscanf ist auf Arduino verfügbar
  if (sscanf(s.c_str(), "%2d-%2d-%4d %2d-%2d-%2d", &DD, &MM, &YYYY, &hh, &mm, &ss) != 6) {
    return false;
  }
  // Plausibilitätsprüfung
  if (YYYY < 2000 || YYYY > 2099) return false;
  if (MM < 1 || MM > 12) return false;
  if (DD < 1 || DD > daysInMonth(YYYY, MM)) return false;
  if (hh < 0 || hh > 23) return false;
  if (mm < 0 || mm > 59) return false;
  if (ss < 0 || ss > 59) return false;

  outLocal.year   = YYYY;
  outLocal.month  = (int8_t)MM;
  outLocal.day    = (int8_t)DD;
  outLocal.hour   = (int8_t)hh;
  outLocal.minute = (int8_t)mm;
  outLocal.second = (int8_t)ss;
  return true;
}

String RTC_Ctrl::formatLocalString(const DateTime& local) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d-%02d-%04d %02d-%02d-%02d",
           local.day, local.month, local.year,
           local.hour, local.minute, local.second);
  return String(buf);
}

/* ===================== Öffentliche API ===================== */
String RTC_Ctrl::readTimeString() {
  DateTime utc;
  if (!readRTC_UTC(utc)) {
    lastReadOk_ = false;
    return String("00-00-0000 00-00-00");
  }
  lastLocal_ = utcToLocal(utc);
  lastReadOk_ = true;
  return formatLocalString(lastLocal_);
}

bool RTC_Ctrl::readComponents(uint16_t& year, uint8_t& month, uint8_t& day,
                              uint8_t& hour, uint8_t& minute, uint8_t& second) {
  DateTime utc;
  if (!readRTC_UTC(utc)) {
    lastReadOk_ = false;
    return false;
  }
  lastLocal_ = utcToLocal(utc);
  lastReadOk_ = true;
  year   = lastLocal_.year;
  month  = (uint8_t)lastLocal_.month;
  day    = (uint8_t)lastLocal_.day;
  hour   = (uint8_t)lastLocal_.hour;
  minute = (uint8_t)lastLocal_.minute;
  second = (uint8_t)lastLocal_.second;
  return true;
}

bool RTC_Ctrl::readUTCComponents(uint16_t& year, uint8_t& month, uint8_t& day,
                                 uint8_t& hour, uint8_t& minute, uint8_t& second) {
  DateTime utc;
  if (!readRTC_UTC(utc)) {
    return false;
  }
  year   = utc.year;
  month  = (uint8_t)utc.month;
  day    = (uint8_t)utc.day;
  hour   = (uint8_t)utc.hour;
  minute = (uint8_t)utc.minute;
  second = (uint8_t)utc.second;
  return true;
}

bool RTC_Ctrl::writeTimeFromString(const String& s) {
  DateTime local;
  if (!parseLocalString(s, local)) return false;
  DateTime utc = localToUTC(local);
  return writeRTC_UTC(utc);
}

/* ===== Einzel-Getter (lesen im Zweifel frisch, um konsistent zu bleiben) ===== */
int RTC_Ctrl::getDay()    { if (!lastReadOk_) readTimeString(); return lastLocal_.day; }
int RTC_Ctrl::getMonth()  { if (!lastReadOk_) readTimeString(); return lastLocal_.month; }
int RTC_Ctrl::getYear()   { if (!lastReadOk_) readTimeString(); return lastLocal_.year; }
int RTC_Ctrl::getHour()   { if (!lastReadOk_) readTimeString(); return lastLocal_.hour; }
int RTC_Ctrl::getMinute() { if (!lastReadOk_) readTimeString(); return lastLocal_.minute; }
int RTC_Ctrl::getSecond() { if (!lastReadOk_) readTimeString(); return lastLocal_.second; }

