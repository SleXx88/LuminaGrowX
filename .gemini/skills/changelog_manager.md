---
name: changelog_manager
description: Puffert Änderungen für LuminaGrowX und schreibt das Changelog bei Release.
---
# Skill: Changelog Manager
## Aufgabe
Verwalte Änderungen für das LuminaGrowX Projekt, ohne sofort die Version zu erhöhen.

## Instruktionen
1. **Puffer-Modus (Standard):** - Wenn ich von einer Änderung erzähle, speichere diese stichpunktartig im internen Gedächtnis (`/memory`) unter dem Schlüssel `LuminaGrowX_pending_changes`.
   - Bestätige kurz, dass die Änderung vorgemerkt wurde.
   
2. **Release-Modus (auf Befehl "Hochversionieren"):**
   - Lese alle Einträge aus `LuminaGrowX_pending_changes`.
   - Frage mich nach der neuen Versionsnummer (z.B. v1.0.7).
   - Schreibe diese Änderungen strukturiert (Features, Bugfixes, Refactoring) ganz oben in die Datei `CHANGELOG.md`.
   - Leere danach den Speicher `LuminaGrowX_pending_changes`.

3. **Status:**
   - Auf Anfrage "Was ist im Puffer?", liste alle aktuell gespeicherten Punkte auf.