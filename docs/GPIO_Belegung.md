# GPIO Belegung - LuminaGrowX

Diese Datei bietet eine Übersicht über die aktuell konfigurierten GPIO-Pins für den ESP32-S3 im Projekt **LuminaGrowX**.
Die Konfiguration stammt aus `include/lumina_config.h`.

> **Hinweis:** Bitte vor Änderungen an der Hardware immer die `include/lumina_config.h` auf Konsistenz prüfen.

## Pin-Übersicht

| GPIO | Funktion / Konstante | Typ | Beschreibung / Modul |
| :--- | :--- | :--- | :--- |
| **/** | `TOF_XSHUT` | Output | Reset/Shutdown Pin für VL53L0X (ToF) Distanzsensor. Entfällt da nicht verwendet! |
| **1** | `FAN_PWM_LED` | PWM Output | PWM-Steuersignal für LED-Lüfter. (nur PWM, hat kein Tacho-Feedback) |
| **2** | `FAN_PWM` | PWM Output | PWM-Steuersignal für Lüfter 1 (1. Hauptlüfter). |
| **4** | `CO2_MQ_VAL` | Analog Input | CO2 Analoger Messwert über Spannungsteiler |
| **5** | `DOOR_SWITCH_PIN` | Input (Pullup) | Türschalter (LOW = Tür geschlossen, HIGH = offen). |
| **6** | `AP_RESET_PIN` | Input (Pullup) | Taster für AP-Modus Reset (bei Boot gedrückt halten). |
| **7** | `FAN2_PWM` | PWM Output | PWM-Steuersignal für Lüfter 2 (2. Hauptlüfter).  |
| **8** | `I2C1_SDA` | I2C Data | SDA für I2C Bus 0 (`Wire`). |
| **9** | `I2C1_SCL` | I2C Clock | SCL für I2C Bus 0 (`Wire`). |
| **10** | `Stepper::EN` | Output | Enable Pin für TMC2209 Stepper Driver (LOW = aktiv). |
| **11** | `Stepper::DIR` | Output | Direction Pin für Stepper (Drehrichtung). |
| **12** | `Stepper::STEP` | Output | Step Pin für Stepper (Schrittimpuls). |
| **13** | `Stepper::DIAG` | Input | Diagnose/Stallguard Pin vom TMC2209. |
| **14** | `CO2_MQ_STAT` | Digital Input | CO2 Schwellenwert |
| **15** | `FAN_TACHO` | Input | Tachosignal von Lüfter 1 (RPM Messung). |
| **16** | `FAN2_TACHO` | Input | Tachosignal von Lüfter 2 (RPM Messung). |
| **17** | `Stepper::UART_TX` | UART TX | UART Kommunikation zu TMC2209 (TX -> PDN via Widerstand). |
| **18** | `Stepper::UART_RX` | UART RX | UART Kommunikation zu TMC2209 (RX <- PDN via Widerstand). |
| **21** | `I2C2_SDA` | I2C Clock | SCL für I2C Bus 1 (`Wire1`). |
| **40** | `WATER_LEVEL_STATE` | Digital Input  | Schwimmer, `TRUE` wenn Wasserstand zu niedrig. |
| **41** | `PUMP_EN` | Digital Output | Luftpumpe für Sauerstoffzufuhr. |
| **47** | `I2C2_SCL` | I2C Data | SDA für I2C Bus 1 (`Wire1`). |
| **48** | `LED_RGB` | Output | RGB LED Anzeige (WS2812B) |

## Bus-Systeme & Details

### I2C Busse
*   **Wire (I2C0):** GPIO 8 (SDA), GPIO 9 (SCL)
    *   Verwendet für: SHT41 (Innen) Klemme CH 1A, GP8211 (DAC/LED) Klemme CH 1B
*   **Wire1 (I2C1):** GPIO 21 (SDA), GPIO 47 (SCL)
    *   Verwendet für: SHT41 (Außen) Klemme CH 2B, ToF (VL53L0X) Klemme CH 2A, DS3231 (RTC) Klemme RTC Onboard

### Stepper (TMC2209)
*   Kommunikation über **UART** (GPIO 17/18) im Single-Wire Modus (Y-Kabel mit 1kΩ Widerstand nötig).
*   **DIR Pin (11):** Bekanntes Problem mit Hardware-Verbindung ("Motor dreht nur in eine Richtung"). Workaround in Software aktiv.

### CO2 Sensor (MQ-2 Gas-Sensor)
*   `CO2_MQ_VAL` -> CO2 Analog Ausgang

### Lüfter (PWM)
*   Frequenz: 25 kHz
*   Auflösung: 8-Bit
