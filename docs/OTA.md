# OTA-Updates (TAR-Pakete)

Ab Version V0.5 unterstützt die Web-UI Updates über ein TAR-Paket:

- Firmware-Update (optional): Datei `firmware.bin` im Paket wird als Sketch geflasht.
- Web-Assets-Update: Alle Dateien unter `www/` im Paket werden selektiv nach LittleFS kopiert (z. B. `www/index.html` -> `/index.html`).
- Konfiguration unter `/cfg` bleibt unberührt.

TAR-Paket erstellen (Beispiel):

```bash
# im Release-Ordner, z. B. build/
# optional: firmware.bin ablegen
mkdir -p www
cp path/to/index.html www/
cp path/to/custom.css www/
tar -cvf package-V0.6.tar firmware.bin www/
```

Manifest für Remote-Updates (z. B. GitHub Pages/raw):

```json
{
  "version": "V0.6",
  "tar_url": "https://<host>/path/to/package-V0.6.tar"
}
```

Die Manifest-URL kann über die Seite `/update` gesetzt werden und wird in `/cfg/update.json` gespeichert.

Web-UI

- `/update` zeigt die aktuelle Version, erlaubt das Prüfen des Manifests, Remote-Update und manuelles Hochladen einer `.tar` Datei.
- Bei Firmware-Update startet das Gerät automatisch neu. Nur Web-Assets ändern keinen Neustart erzwingen.

