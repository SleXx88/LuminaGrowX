# 🌱 vpd_calc – Vapor Pressure Deficit Library

Kleine Library für den ESP32, um den **Vapor Pressure Deficit (VPD)** zu berechnen und zu klassifizieren.  
Verwendet Temperatur und Luftfeuchtigkeit vom SHT41 (oder anderen Sensoren).

---

## 📌 Funktionen

| Funktion                                                               | Eingabe                      | Ausgabe                        | Beschreibung                                            |
| ---------------------------------------------------------------------- | ---------------------------- | ------------------------------ | ------------------------------------------------------- |
| `double saturationVaporPressure(double tempC)`                         | Temperatur °C                | Sättigungsdampfdruck (kPa)     | Berechnet SVP bei gegebener Temperatur                  |
| `double actualVaporPressure(double tempC, double rh)`                  | Temperatur °C, Luftfeuchte % | Aktueller Dampfdruck (kPa)     | Berechnet AVP aus SVP und RH                            |
| `double computeVpd(double tempC, double rh)`                           | Temperatur °C, Luftfeuchte % | VPD (kPa)                      | Hauptfunktion zur Berechnung des VPD                    |
| `const char* classifyVpd(double vpd, GrowthStage stage)`               | VPD (kPa), Wachstumsphase    | `"GREEN"`, `"YELLOW"`, `"RED"` | Klassifiziert den VPD-Wert je nach Pflanzenphase        |
| `double computeRelativeHumidityForVpd(double tempC, double targetVpd)` | Temperatur °C, Ziel-VPD kPa  | Luftfeuchte %                  | Gibt die benötigte RH aus, um den Ziel-VPD zu erreichen |
| `double dewPoint(double tempC, double rh)`                             | Temperatur °C, Luftfeuchte % | Taupunkt °C                    | Nützlich zur Schimmel-/Kondensationswarnung             |

---

## 📌 Wachstumsphasen & VPD-Bereiche

Standardmäßig sind diese Bereiche hinterlegt (kann angepasst werden):

| Phase                     | Grün (optimal) | Gelb (Übergang)       | Rot (ungünstig) |
| ------------------------- | -------------- | --------------------- | --------------- |
| **Seedling / Keimung**    | 0.4 – 0.8 kPa  | 0.3 – 0.4 / 0.8 – 0.9 | <0.3 oder >0.9  |
| **Vegetative / Wachstum** | 0.8 – 1.2 kPa  | 0.7 – 0.8 / 1.2 – 1.3 | <0.7 oder >1.3  |
| **Flowering / Blüte**     | 1.2 – 1.6 kPa  | 1.1 – 1.2 / 1.6 – 1.7 | <1.1 oder >1.7  |

---

## 📌 Beispiel (Arduino / ESP32)

```cpp
#include <Arduino.h>
#include "sht41_ctrl.h"
#include "vpd_calc.h"

using namespace vpd_calc;
SHT41Ctrl sht;

void setup() {
  Serial.begin(115200);
  sensor.begin();

  float tC = NAN, rh = NAN;
  if (!sht.read(tC, rh))
  {
    Serial.println(F("[SHT41] Read FAILED"));
  }

  double vpd = computeVpd(tC, rh);
  const char* status = classifyVpd(vpd, GrowthStage::Vegetative);

  Serial.printf("[VPD]  T=%.2f°C  RH=%.2f%%  VPD=%.2f kPa  Status=%s\n",
                tC, rh, vpd, status);
}

void loop() {
  delay(5000);
}
```
