# Changelog - LuminaGrowX LisaPro

Alle wichtigen Änderungen und Features der automatisierten Growbox-Steuerung.

## [V1.0.6] - 2026-02-19
### Netzwerk-Identifikation & Hostname-Anpassung

Verbesserung der Sichtbarkeit und Identifizierung der LisaPro im Heimnetzwerk (z.B. Fritzbox).

#### 🚀 Neue Features
*   **Eindeutiger Hostname:** Das Gerät meldet sich nun standardmäßig mit `LisaPro-[MAC]` (z.B. `LisaPro-A1B2`) beim Router an, anstatt des generischen Namens `ESP32`.
*   **Konfigurierbarer Hostname:** Der Name für mDNS und DHCP kann nun in den **Einstellungen** individuell angepasst werden.
*   **mDNS Zugriff:** Die Box ist im Browser nun über `http://LisaPro-[MAC].local` (bzw. den gewählten Namen) erreichbar.

#### 🔧 Verbesserungen
*   **Dynamische Links:** Erfolgsmeldungen im Web-Interface zeigen nun automatisch den korrekten Link zum Gerät an, basierend auf dem gewählten Hostnamen.
*   **Setup-Flow:** Der Inbetriebnahme-Assistent erkennt den konfigurierten Namen und bietet am Ende den passenden Link zur Weiterleitung an.

## [V1.0.5] - 2026-02-19
### Performance-Optimierung & Erweiterte Tür-Logik

Dieses Release fokussiert sich auf die Stabilisierung der Regelstrategie, eine intelligentere Reaktion auf Tür-Interaktionen und die Behebung kritischer Anlauf-Probleme der Lüfter.

#### 🔧 Bugs behoben
*   **Sofortige Annäherung:** Triggert jetzt eine sofortige LED-Annäherung, wenn ein Grow gestartet wird (neu oder nach Boot) oder wenn die Tür geschlossen wird (nur während eines aktiven Grows).
*   **Tür-Zustandserkennung:** Korrektur der Anzeige/Erkennung des Türzustands, wenn der Grow gestoppt oder pausiert ist.
*   **Erhöhte Entprellzeit:** Die Zeitspanne, die der Tür-Pin stabil bleiben muss, bevor ein Zustandswechsel akzeptiert wird, wurde von 50 ms auf **200 ms** erhöht.
*   **Lüfter-Anlauf Fix:** Behebung eines Fehlers, bei dem die Abluft-Lüfter (FAN1 & 2) beim Start der Regelung bei 0% PWM blieben, obwohl ein "Lüfter Min (Tag/Nacht)" Wert hinterlegt war.

#### 📈 Regelstrategie verbessert
*   **Stabilität des Regeltakts:** Die Steuerung wurde auf eine feste Frequenz gedrosselt. Dies verhindert "Einfrieren" der Berechnungen durch die extrem hohe Taktrate des Prozessors.
*   **Anlauf-Blockade gelöst:** Entfernung einer minimalen Schwelle, die verhinderte, dass Lüfter ihren sanften Anlauf aus dem Stillstand heraus starten konnten.
*   **Intelligente Priorisierung:** Neusortierung der Sicherheitsfunktionen (z. B. bei extremer Feuchtigkeit/Hitze). Der jeweils höchste Leistungsbedarf gewinnt nun automatisch, ohne gegenseitige Blockade.
*   **Permanenter Klima-Abgleich:** Das System prüft nun kontinuierlich, ob Außenluft tatsächlich zur Verbesserung des Innenklimas beitragen kann, bevor die Lüfterleistung unnötig erhöht wird.
*   **Verstärkte Reaktionskraft:** Die grundlegende Reaktion auf Abweichungen wurde spürbarer eingestellt, sodass das System früher und entschlossener eingreift.
*   **Dynamische Anpassung:** Erhöhung der Geschwindigkeit, mit der Lüfter ihre Drehzahl ändern dürfen, für eine lebendigere Regelung.

#### ✨ Neuerungen
*   **Automatisches Parken:** Das LED-Panel fährt nun automatisch nach oben, sobald der Grow gestoppt wird.
*   **Interaktive Tür-Maßnahmen:** Neue Optionen beim Öffnen der Tür:
    *   Regelung pausieren.
    *   Beleuchtung EIN (Arbeitslicht 10%, auch bei inaktiver Regelung).
    *   Pumpe AUS.
    *   LED-Panel anheben (10cm) für besseren Zugriff.

---

## [V1.0.4] - 2026-02-17
### Erweiterte Pumpen- & Lüftersteuerung

Dieses Release führt phasenabhängige Einstellungen für die Luftpumpe und den LED-Umluftlüfter ein und optimiert die Stabilität der Web-Oberfläche.

#### 🚀 Kernfunktionen & Features
*   **Pumpensteuerung (Luft):** Die Luftpumpe kann nun individuell pro Phase (Keimling, Wachstum, Blüte) im Zeitplan aktiviert oder deaktiviert werden.
*   **Umluft-Lüfter (LED) implementiert:** Der Lüfter an Klemme LOAD1 (`fan3`) kann nun phasenweit geregelt werden. 
    *   *⚠️ ACHTUNG:* Der Betrieb an Klemme LOAD1 erfordert zwingend einen passenden LC-Filter.
*   **Optimierter Silent-Modus:** Der Silent-Modus wurde um dedizierte Einstellungen für den Umluft-Lüfter sowie eine Freigabe-Option für die Pumpe erweitert.

#### 🎨 Web-Interface (UI/UX)
*   **Live Status Redesign:** Vollständig überarbeitete Startseite mit logischer Gruppierung (Klima, Aktoren, Raum, System).
*   **Verbesserte Statusanzeigen:** Neue Live-Anzeige für Umluft-Lüfter und detaillierter Status für den Silent-Modus (Aktiv, Standby, Deaktiviert).
*   **Eingabevalidierung:** Absicherung aller Einstellungsfelder gegen ungültige Werte (0-100%, Min <= Max).
*   **Stabilitäts-Fix:** Behebung von Fehlern beim Speichern großer Konfigurations-JSONs durch optimiertes Buffering im Backend.

#### 🔧 Fehlerbehebungen & Optimierungen
*   **Fan 2 Parallelregelung:** Lüfter 2 wird nun korrekt angesteuert und läuft in der VPD-Regelung parallel zu Lüfter 1.
*   **Begriffsanpassung:** Konsistente Umbenennung von "Bewässerung" in **"Pumpe (Luft)"** im gesamten System.

---

## [V1.0.3] - 2026-02-14
### Optimierte Abstandsregelung & Sicherheit (ToF)

Dieses Release verbessert die Zuverlässigkeit der LED-Positionierung und schützt die Hardware vor Fehlfunktionen des Distanzsensors.

#### 🛡 Sicherheit & ToF-Sensorik
*   **Sicherheits-Stopp bei Sensorfehlern:** Die LED-Fahrt zur Pflanze wird nun sofort unterbrochen, wenn der ToF-Sensor ungültige Werte (z. B. -1 für "Out of Range") liefert. Dies verhindert unkontrollierte Bewegungen bei Sensorausfall.
*   **Fehler-Diagnose:** Tritt während einer aktiven Anpassung oder beim Startup ein Sensorfehler auf, wird dies nun explizit als ERROR in der seriellen Konsole ausgegeben.
*   **Erweitertes Startup-Zeitfenster:** Das Timeout für die sanfte Annäherung beim Systemstart wurde von 30 auf **40 Sekunden** erhöht, um auch bei großen Distanzen oder langsameren P-Regelungs-Geschwindigkeiten sicher das Ziel zu erreichen.

#### 🔧 Sensorik & Hardware
*   **Tür-Entprellung:** Die Erkennung des Türzustands wurde durch eine 50ms Software-Entprellung stabilisiert, was Fehlauslösungen durch mechanische Schwingungen oder elektrische Störungen verhindert.

#### 🌐 MQTT & Update-System
*   **MQTT-Stabilität:** Der MQTT-Sendepuffer wurde auf 3072 Bytes erhöht, um auch bei umfangreichen Status-JSONs Datenverlust zu vermeiden.
*   **Update-Status Fix:** Der binäre Update-Sensor in Home Assistant wurde repariert (Key-Flattener & Template-Optimierung).
*   **Automatischer Update-Check:** Die LisaPro prüft nun alle **6 Stunden** (statt täglich) auf neue Firmware-Versionen, sobald eine Internetverbindung besteht.

---

## [V1.0.2] - 2026-02-12
### System-Monitoring & Update-Fixes

Dieses Release behebt Probleme beim Update-Vorgang und erweitert das System-Monitoring um interne Chip-Diagnosen.

#### 🔧 Update-System & Stabilität
*   **Update-UI Refresh Fix:** Die Update-Seite erkennt nun zuverlässig den Neustart des Geräts (via Uptime-Check), auch wenn die Firmware-Version nach einem fehlerhaften Flash-Versuch gleich geblieben ist.
*   **Log-Filterung:** Doppelte Einträge im Update-Log ("Spam") werden nun unterdrückt.
*   **Robustes Firmware-Handling:** Der Update-Prozess im Backend prüft nun strikt die Integrität der geschriebenen Daten (`Update.end()` Prüfung).
*   **Build-Automation:** Das Skript `make_tar.ps1` erzwingt nun immer einen Neu-Build der Firmware, um sicherzustellen, dass Versionsänderungen garantiert im Paket landen.

#### 📊 Monitoring & MQTT
*   **Interne Chip-Temperatur:** Integration des internen ESP32-S3 Temperatursensors zur Überwachung der System-Gesundheit.
*   **Info-Seite:** Anzeige der Chip-Temperatur auf der `/info` Seite hinzugefügt.
*   **MQTT-Fix:** Der Binär-Sensor für verfügbare Updates in Home Assistant (`update_verfugbar`) wurde repariert und zeigt nun korrekt den Status an.
*   **MQTT-Erweiterung:** Neuer Sensor für die Chip-Temperatur via MQTT hinzugefügt.

---

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
