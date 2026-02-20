# Benutzung des Changelog-Managers

Dieser Skill hilft dabei, Änderungen während der Entwicklung zu sammeln und gesammelt in die `CHANGELOG.md` zu schreiben, wenn ein Release ansteht.

## Workflows

### 1. Änderungen puffern (Während der Arbeit)
Sag Gemini einfach, was erledigt wurde.
- **Beispiel:** *"Merke dir für das Changelog: Bugfix beim WLAN-Reconnect."*
- **Hintergrund:** Die Daten werden im globalen "Memory" gespeichert und bleiben über verschiedene CLI-Sitzungen hinweg erhalten.

### 2. Puffer einsehen
- **Befehl:** *"Was ist im Puffer?"* oder *"Zeig mir die ausstehenden Änderungen."*

### 3. Version erstellen (Release)
Wenn die Arbeit an einer Version abgeschlossen ist.
- **Befehl:** *"Hochversionieren auf V1.x.x"*
- **Aktion:** Gemini liest den Puffer, schreibt die Einträge in die `CHANGELOG.md` und löscht den Puffer.

## Warum das Ganze?
Dadurch verhindern wir, dass die `CHANGELOG.md` bei jeder kleinen Änderung angefasst werden muss, was oft zu Merge-Konflikten oder unübersichtlichen Commits führt. Erst wenn die Version "fertig" ist, wird das Dokument aktualisiert.
