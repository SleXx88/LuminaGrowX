/*
 * env_ctrl.h
 *
 * Hochwertige Umweltsteuerung für eine ESP32 Growbox. Diese Klasse
 * kombiniert einen Temperatur-/Feuchtigkeitssensor (SHT41), einen LED-Dimmer
 * (GP8211) und einen PWM-Lüfterregler, um ein Ziel-VPD (Vapor Pressure Deficit)
 * für verschiedene Pflanzenwachstumsphasen aufrechtzuerhalten. Jede
 * Wachstumsphase und Tageszeit hat konfigurierbare LED- und Lüftergrenzen
 * sowie gewünschte VPD-Bereiche. Ein einfacher Proportionalregler
 * passt Lüfter und LED innerhalb ihrer jeweiligen Grenzen auf den Mittelpunkt
 * des konfigurierten VPD-Bereichs an. Standardwerte basieren auf allgemein empfohlenen
 * VPD-Bereichen für Keimlinge, vegetative und Blütephasen.
 *
 * Der Benutzer kann diese Standardwerte zur Laufzeit über Setter-Funktionen
 * überschreiben. Die Regelungsfaktoren für Lüfter und LED können ebenfalls
 * angepasst werden. Diese Bibliothek hängt von vpd_calc.h ab, um das
 * aktuelle VPD aus Temperatur und relativer Luftfeuchtigkeit zu berechnen.
 *
 * Beispiel:
 *   #include "env_ctrl.h"
 *   using namespace env_ctrl;
 *
 *   SHT41Ctrl sensor;
 *   GP8211Ctrl led;
 *   FanCtrl fan;
 *   EnvCtrl controller;
 *
 *   void setup() {
 *     sensor.begin(Wire);
 *     led.begin(Wire);
 *     fan.begin({.pwmPin=2, .ledcChannel=0, .invert=true});
 *     controller.begin(sensor, led, fan);
 *     controller.setStage(vpd_calc::GrowthStage::Vegetative);
 *     controller.setMode(DayMode::Day);
 *   }
 *   void loop() {
 *     if (controller.update()) {
 *       // optional: controller.currentVpd(), controller.currentLedPercent(), etc. protokollieren
 *     }
 *     delay(1000);
 *   }
 */

#pragma once

#include <Arduino.h>
#include "sht41_ctrl.h"
#include "gp8211_ctrl.h"
#include "fan_ctrl.h"
#include "vpd_calc.h"

namespace env_ctrl {

/**
 * Aufzählung für die Tageszeit-Modi. Der Controller kann
 * angewiesen werden, in einem von drei Modi zu arbeiten: Tag, Nacht und
 * NachtSilent. NachtSilent ist identisch mit Nacht, begrenzt aber die
 * maximale Lüftergeschwindigkeit für geringere Geräuschentwicklung.
 */
enum class DayMode {
    Day = 0,
    Night = 1,
    NightSilent = 2
};

/**
 * EnvCtrl – Umweltcontroller zur Aufrechterhaltung des VPD innerhalb
 * konfigurierter Grenzen. Verwendet einen einfachen Proportionalregler für
 * die Differenz zwischen gemessenem VPD und dem Mittelpunkt des
 * konfigurierten VPD-Bereichs. Passt sowohl die LED-Helligkeit als auch die Lüfter-
 * geschwindigkeit auf diesen Mittelpunkt an. LED- und Lüftergrenzen sowie Ziel-
 * VPD-Bereiche sind pro Wachstumsphase und Modus konfigurierbar. Die
 * tatsächliche VPD-Berechnung wird an die vpd_calc-Bibliothek delegiert.
 */
class EnvCtrl {
public:
    /**
     * Erstellt einen neuen EnvCtrl mit Standardeinstellungen. Der
     * Konstruktor initialisiert keine Hardware; rufen Sie begin()
     * auf, bevor Sie update() verwenden. Standard-Regelungsfaktoren sind
     * auf moderate Werte gesetzt, die eine sanfte Steuerung bieten sollten;
     * passen Sie diese über setKpFan() und setKpLed() an Ihr Gehäuse an.
     */
    EnvCtrl();

    /**
     * Bindet den Sensor, LED-Treiber und Lüfterregler an den
     * Umweltcontroller. Dies ruft nicht die begin()-Methoden der übergebenen
     * Objekte auf; Sie sollten Ihren SHT41Ctrl, GP8211Ctrl und FanCtrl
     * extern initialisieren, bevor Sie diese Methode aufrufen. Nach begin()
     * sollten Sie auch setStage() und setMode() aufrufen, um sicherzustellen,
     * dass der Controller mit den richtigen Einstellungen arbeitet.
     */
    void begin(SHT41Ctrl& sensor, GP8211Ctrl& ledDriver, FanCtrl& fan);

    /**
     * Setzt die aktive Wachstumsphase. Die Phase bestimmt den
     * Standard-VPD-Bereich und die LED-/Lüftergrenzen. Verwenden Sie die
     * vpd_calc::GrowthStage-Enum-Werte (Seedling, Vegetative, Flowering).
     */
    void setStage(vpd_calc::GrowthStage stage);

    /**
     * Setzt den Tageszeit-Modus (Tag, Nacht oder NachtSilent). Jeder
     * Modus kann eigene LED- und Lüftergrenzen sowie VPD-Bereiche haben.
     */
    void setMode(DayMode mode);

    /**
     * Überschreibt die LED- und Lüftergrenzen sowie den VPD-Bereich für eine
     * bestimmte Kombination aus Wachstumsphase und Modus. Werte werden intern
     * auf sinnvolle Bereiche begrenzt (LED und Lüfter zwischen 0 und
     * 100 %, VPD-Minimum muss nicht negativ sein und VPD-Maximum
     * muss das VPD-Minimum überschreiten). Mit dieser Funktion können
     * Sie Feineinstellungen aus Ihrem Sketch vornehmen, ohne die Bibliothek zu bearbeiten.
     */
    void setStageModeLimits(vpd_calc::GrowthStage stage, DayMode mode,
                            float ledMin, float ledMax,
                            float fanMin, float fanMax,
                            float vpdMin, float vpdMax);

    /**
     * Setzt den Proportionalfaktor für die Lüftersteuerung. Ein höherer
     * Wert bewirkt eine größere Änderung der Lüftergeschwindigkeit pro VPD-
     * Fehler. Standard ist 20.0. Die Faktoren können im laufenden
     * Betrieb angepasst werden, um die gewünschte Reaktionsfähigkeit ohne
     * Schwingungen zu erreichen.
     */
    void setKpFan(float kp);

    /**
     * Setzt den Proportionalfaktor für die LED-Steuerung. Ein höherer
     * Wert bewirkt eine größere Änderung der LED-Helligkeit pro VPD-
     * Fehler. Standard ist 10.0. Die Faktoren können im laufenden
     * Betrieb angepasst werden, um die gewünschte Reaktionsfähigkeit ohne
     * Schwingungen zu erreichen. Hinweis: Extrem große Werte können dazu führen,
     * dass die LED unter ledMin oder über ledMax geht; die Ausgänge werden
     * intern auf die konfigurierten Grenzen begrenzt.
     */
    void setKpLed(float kp);

    /**
     * Führt einen Steuerzyklus aus. Liest Temperatur und
     * Luftfeuchtigkeit vom gebundenen SHT41-Sensor, berechnet das VPD,
     * berechnet den Fehler relativ zum konfigurierten Ziel und
     * aktualisiert die LED- und Lüfterausgänge entsprechend. Gibt
     * true bei Erfolg oder false zurück, wenn das Auslesen des Sensors fehlgeschlagen ist. Diese
     * Methode sollte regelmäßig in Ihrer loop() aufgerufen werden.
     */
    bool update();

    /**
     * Zugriff auf den zuletzt gemessenen VPD-Wert (kPa).
     */
    double currentVpd() const { return vpd_; }

    /**
     * Zugriff auf die zuletzt gemessene Temperatur (°C).
     */
    double currentTemp() const { return temp_; }

    /**
     * Zugriff auf die zuletzt gemessene relative Luftfeuchtigkeit (%).
     */
    double currentRh() const { return rh_; }

    /**
     * Zugriff auf den zuletzt ausgegebenen LED-Prozentwert (0–100).
     */
    float currentLedPercent() const { return ledPercent_; }

    /**
     * Zugriff auf den zuletzt ausgegebenen Lüfter-Prozentwert (0–100).
     */
    float currentFanPercent() const { return fanPercent_; }

private:
    // Struktur zur Speicherung der phasen- und modusabhängigen Grenzen und Ziel-
    // VPD-Bereiche. LED- und Lüftergrenzen sind in Prozent (0–100) und
    // VPD-Werte in kPa. Dies ermöglicht das Indizieren nach
    // Phase/Modus, um die aktuellen Einstellungen abzurufen.
    struct PhaseModeSettings {
        float ledMin;
        float ledMax;
        float fanMin;
        float fanMax;
        float vpdMin;
        float vpdMax;
    };

    // Standardeinstellungen für drei Wachstumsphasen (Seedling,
    // Vegetative, Flowering) und drei Tagesmodi (Tag, Nacht,
    // NachtSilent). Siehe Konstruktor für Initialisierung.
    PhaseModeSettings phaseSettings_[3][3];

    // Aktuelle Wachstumsphase und Modus
    vpd_calc::GrowthStage stage_;
    DayMode mode_;

    // Regelungsfaktoren
    float kpFan_;
    float kpLed_;

    // Gebundene Hardware
    SHT41Ctrl* sensor_;
    GP8211Ctrl* ledDriver_;
    FanCtrl* fan_;

    // Zuletzt gemessene und berechnete Werte
    float ledPercent_;
    float fanPercent_;
    double vpd_;
    double temp_;
    double rh_;

    // Hilfsfunktion zum Begrenzen eines Wertes zwischen min und max
    static float clamp(float value, float minVal, float maxVal) {
        if (value < minVal) return minVal;
        if (value > maxVal) return maxVal;
        return value;
    }

    // Berechnet den Index einer Wachstumsphase (0,1,2) aus dem Enum
    static int stageIndex(vpd_calc::GrowthStage stage) {
        switch (stage) {
        case vpd_calc::GrowthStage::Seedling:   return 0;
        case vpd_calc::GrowthStage::Vegetative: return 1;
        case vpd_calc::GrowthStage::Flowering:  return 2;
        default: return 0;
        }
    }

    // Berechnet den Index eines Tagesmodus (0,1,2)
    static int modeIndex(DayMode mode) {
        switch (mode) {
        case DayMode::Day:         return 0;
        case DayMode::Night:       return 1;
        case DayMode::NightSilent: return 2;
        default: return 0;
        }
    }

    // Gibt eine Referenz auf die aktuelle Einstellungen-Struktur zurück
    PhaseModeSettings& currentSettings() {
        return phaseSettings_[stageIndex(stage_)][modeIndex(mode_)];
    }
};

} // namespace env_ctrl
