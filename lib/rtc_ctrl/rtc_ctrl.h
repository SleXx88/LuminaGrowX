#pragma once
#include <Arduino.h>
#include <Wire.h>

/*
  rtc_ctrl.h  –  Kleine DS3231-Library (ESP32/Arduino)
  ----------------------------------------------------
  - Führt die DS3231 intern in UTC.
  - Lesen gibt lokale Zeit (Europa/Berlin) mit automatischer Sommer-/Winterzeit.
  - Schreiben erwartet lokale Zeit (DD-MM-YYYY HH-MM-SS) und speichert UTC in die RTC.
  - Sehr kleine Implementierung ohne zusätzliche Abhängigkeiten – nur Wire.h.

  Hardware:
    - DS3231 I²C-Adresse: 0x68 (fest)
    - Modul: z. B. AZ-Delivery

  Funktionen:
    - bool begin(TwoWire& w = Wire, uint8_t i2cAddr = 0x68)
    - bool isConnected()
    - String readTimeString()                       // "DD-MM-YYYY HH-MM-SS" (lokal, CET/CEST)
    - bool   readComponents(uint16_t& y, uint8_t& m, uint8_t& d,
                            uint8_t& hh, uint8_t& mm, uint8_t& ss)  // lokal, CET/CEST
    - bool writeTimeFromString(const String& s)     // erwartet lokale Zeit; speichert UTC
    - Getter einzeln:
        int getDay(), getMonth(), getYear(), getHour(), getMinute(), getSecond()

  Hinweise:
    - 24h-Format wird genutzt.
    - Schaltjahre werden beachtet (durch DS3231 + unsere Umrechnungen).
    - Sommer-/Winterzeit nach EU-Regel (seit 1996): 
      DST aktiv: 01:00 UTC letzter Sonntag im März bis 01:00 UTC letzter Sonntag im Oktober.
*/

class RTC_Ctrl {
public:
  RTC_Ctrl();

  bool begin(TwoWire& w = Wire, uint8_t i2cAddr = 0x68);
  bool isConnected();

  // Formatierter Zeitstring (lokal, inkl. Sommer-/Winterzeit)
  String readTimeString();

  // Einzelwerte (lokal)
  int getDay();
  int getMonth();
  int getYear();     // vierstellig
  int getHour();
  int getMinute();
  int getSecond();

  // Komponenten gesammelt (lokal)
  bool readComponents(uint16_t& year, uint8_t& month, uint8_t& day,
                      uint8_t& hour, uint8_t& minute, uint8_t& second);

  // Zeit setzen aus lokalem String "DD-MM-YYYY HH-MM-SS" -> schreibt UTC in die RTC
  bool writeTimeFromString(const String& s);

private:
  TwoWire* wire_ = nullptr;
  uint8_t addr_ = 0x68; // DS3231 default I2C address

  struct DateTime {
    int16_t year;   // 2000..2099 (ausreichend)
    int8_t  month;  // 1..12
    int8_t  day;    // 1..31
    int8_t  hour;   // 0..23
    int8_t  minute; // 0..59
    int8_t  second; // 0..59
  };

  // Low-Level: BCD <-> Binär
  static uint8_t bcd2bin(uint8_t v);
  static uint8_t bin2bcd(uint8_t v);

  // DS3231 UTC lesen/schreiben (rohe Register, 24h)
  bool readRTC_UTC(DateTime& dtUTC);
  bool writeRTC_UTC(const DateTime& dtUTC);

  // EU-DST-Logik (Offset zu CET bestimmen aus UTC)
  static bool isLeapYear(int y);
  static int  daysInMonth(int y, int m);
  static int  dayOfWeek(int y, int m, int d); // 0=Sonntag..6=Samstag
  static int  lastSundayOfMonth(int y, int m);
  static bool isDST_EU_fromUTC(const DateTime& utc);
  static void addHours(DateTime& t, int deltaHours); // kleine Normalisierung (+/- wenige Stunden)

  // Konvertierung UTC <-> Lokal (Europa/Berlin: CET=UTC+1, CEST=UTC+2)
  static DateTime utcToLocal(const DateTime& utc);
  static DateTime localToUTC(const DateTime& local);

  // Parser/Formatter
  static bool parseLocalString(const String& s, DateTime& outLocal);
  static String formatLocalString(const DateTime& local);

  // Cache der letzten gelesenen lokalen Zeit (für Einzel-Getter)
  DateTime lastLocal_ = {2000,1,1,0,0,0};
  bool lastReadOk_ = false;
};

