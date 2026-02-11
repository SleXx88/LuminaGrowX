# Changelog - LuminaGrowX LisaPro

Alle wichtigen Änderungen und Features der automatisierten Growbox-Steuerung.

## [V1.0.1] - 2026-02-12
### Web-UI Redesign & Stabilitäts-Updates

Dieses Update konzentriert sich auf die Verbesserung der Benutzererfahrung im Web-Interface und eine robustere Netzwerk-Konfiguration.

#### 🎨 Web-Interface (UI/UX)
*   **Modernisierte Update-Seite:** Neues Design mit Fortschrittsbalken, Echtzeit-Log-Fenster und automatischer Anzeige der Versions-Highlights direkt aus dem lokalen Changelog.
*   **Optimierte Einstellungen:** Layout für Desktop-Nutzung verbessert (Grid-System). Buttons ohne Unterstreichungen für einen saubereren Look.
*   **Interaktiver Inbetriebnahme-Assistent:** Der Reset des Setups erfolgt nun über einen Bestätigungs-Dialog mit klarer Funktions-Übersicht (WLAN-Reset, Kalibrierung von ToF/RTC).
*   **Begriffs-Korrektur:** Alle Referenzen auf nicht vorhandene Hardware (Waage) im Setup entfernt.

#### 🔧 System & Netzwerk
*   **AP-Persistence:** Im Setup-Modus bleibt der Access Point nun dauerhaft aktiv, auch wenn eine Verbindung zum Heim-WLAN besteht. Dies verhindert den Verbindungsverlust während der Einrichtung.
*   **Changelog-Integration:** Die `CHANGELOG.md` wird nun automatisch auf den ESP32 synchronisiert und ist direkt über das Web-Interface (`/CHANGELOG.md`) abrufbar.
*   **API-Fixes:** Mehrere Kompilierfehler und Scope-Probleme in der Web-Steuerung (`web_ctrl.cpp`) behoben.

#### 🛠 Build & Automation
*   **Build-Skripte:** Automatischer Export des Changelogs in die Daten-Ordner (`data/` und `www/`) vor dem Packen des Release-Tars oder Factory-Bins integriert.

---

## [V1.0.0] - 2026-02-11
### Initial Stable Release - LisaPro Edition

Dies ist das erste stabile Release der LuminaGrowX LisaPro Firmware. Dieses Release markiert den Übergang von der Testphase zur produktiven Nutzung mit einem vollständigen Funktionsumfang für die automatisierte Pflanzenzucht.

#### 🚀 Kernfunktionen (Core Features)
*   **Intelligente VPD-Steuerung:** Vollautomatisierte Regelung des Vapor Pressure Deficit (VPD) basierend auf dem aktuellen Wachstumsstadium. Unterstützung für Keimung (Seedling), Vegetationsphase und Blütephase mit individuellen Zielwerten.
*   **Präzise LED-Höhenverstellung:** Integration von TMC2209 Stepper-Treibern über UART. Ermöglicht flüsterleisen Betrieb (StealthChop) und präzise Positionierung der Beleuchtung. 
    *   *Spezial-Fix:* Eigens entwickelte `TMCTiny` Library zur Behebung von CRC-Kommunikationsfehlern.
    *   *Sicherheit:* Automatisches Homing (Referenzfahrt) und Schutz vor mechanischer Überlastung.
*   **Fortschrittliches Licht-Management:** 0-10V LED-Dimmen via GP8211 DAC. Unterstützung für sanfte Übergänge (Fading) sowie Sonnenaufgangs- und Sonnenuntergangs-Simulationen.
*   **Klima-Sensorik:** Dual-SHT41 Support für präzise Temperatur- und Luftfeuchtigkeitsmessung im Innen- und Außenbereich der Box.
*   **Abstandsmessung:** VL53L0X Time-of-Flight (ToF) Sensor zur Überwachung des Abstands zwischen Canopy (Pflanzendecke) und Lichtquelle.

#### 🌐 Konnektivität & Interface
*   **Modernes Web-UI:** Komplett neues Frontend im "Glasmorphismus"-Design basierend auf Pico CSS. Optimiert für Desktop und mobile Endgeräte.
*   **Echtzeit-Überwachung:** Status-Updates via WebSockets für verzögerungsfreie Anzeige von Sensordaten und Aktuator-Zuständen.
*   **WhatsApp Benachrichtigungen:** Integration von CallMeBot. Erhalte Statusberichte und Warnungen bei Grenzwertüberschreitungen direkt auf dein Smartphone.
*   **MQTT Integration:** Vollständige Unterstützung für Smart-Home-Systeme (wie Home Assistant). Übertragung aller Sensordaten und Steuerung wichtiger Funktionen via MQTT.
*   **Interaktives Setup:** Geführter Erst-Einrichtungsassistent (Captive Portal), der WLAN, Zeit (RTC) und Hardware-Komponenten kalibriert.

#### 🛠 System & Wartung
*   **Duales Update-System:**
    *   **OTA Update (.tar):** Einfaches Einspielen von Firmware- und Web-Assets über das Web-Interface ohne Datenverlust.
    *   **Factory Image (.bin):** Bereitstellung eines Single-Binary-Images für die Erstinstallation via Web-Flasher (USB).
*   **NVS Configuration:** Alle Einstellungen (Phasenpläne, Netzwerkdaten, Kalibrierwerte) werden sicher im nicht-flüchtigen Speicher (NVS) des ESP32-S3 abgelegt.
*   **Health Monitoring:** Kontinuierliche Überprüfung aller Hardware-Module (I2C-Busse, Sensoren, Aktuatoren) mit Fehlermeldungen im Web-Interface.
*   **RTC-Management:** Unterstützung für DS3231/DS1307 Echtzeituhren mit automatischer Synchronisierung via Internetzeit (SNTP).

#### 🔌 Hardware-Belegung (ESP32-S3)
*   **UART Stepper:** TX=17, RX=18 (TMC2209).
*   **I2C Busse:** Haupt-Bus (SDA=1, SCL=2), Zweiter Bus (SDA=19, SCL=20).
*   **Lüfter:** PWM-Steuerung für bis zu 3 Lüfterinstanzen (Abluft, Umluft).
*   **Eingänge:** Digitale Überwachung für Tür-Kontakt und Wasserstandssensor.

---
*Hinweis: Für ein Update von einer Vorversion laden Sie bitte die `LuminaGrowX_Update.tar` über die [Update Seite](http://luminagrowx.local/update) hoch.*
