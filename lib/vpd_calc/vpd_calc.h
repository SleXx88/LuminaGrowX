/*
 * vpd_calc.h
 *
 * Deklaration einer kleinen Bibliothek zur Berechnung des Dampfdruckdefizits (VPD)
 * und verwandter Feuchtigkeitsmetriken. Siehe vpd_research.md für Hintergrund
 * und Referenzen zu den verwendeten Formeln. Die Bibliothek stellt Funktionen für
 * Sättigungsdampfdruck, VPD-Berechnung, Klassifizierung basierend auf
 * Wachstumsstadium, Feuchtigkeitsinversen und Taupunkt bereit.
 */

#pragma once

#include <cstdint>

namespace vpd_calc
{
    /**
     * Aufzählung, die die primären Wachstumsstadien einer Pflanze darstellt.
     * Diese Werte werden verwendet, um Standard-VPD-Bereiche auszuwählen.
     * Stadien entsprechen Keimung/frühem vegetativem Wachstum, vegetativem Wachstum und Blüte.
     */
    enum class GrowthStage : std::uint8_t
    {
        Seedling = 1,
        Vegetative = 2,
        Flowering = 3
    };

    /**
     * Struktur, die Schwellenwerte für die VPD-Klassifizierung enthält.
     * Jeder Schwellenwert definiert einen grünen (optimalen) Bereich und einen gelben (Warn-)Bereich.
     * Werte außerhalb beider Bereiche werden als rot betrachtet.
     */
    struct VpdThreshold
    {
        double greenMin;
        double greenMax;
        double yellowMin;
        double yellowMax;
        // Hilfsmethoden sind in der Implementierungsdatei definiert.
        bool isGreen(double vpd) const;
        bool isYellow(double vpd) const;
        bool isRed(double vpd) const;
    };

    // --- VPD- und Feuchtigkeitsberechnungen ---

    /**
     * Berechnet den Sättigungsdampfdruck (SVP) in Kilopascal (kPa) für eine
     * gegebene Lufttemperatur in Grad Celsius. Die Implementierung verwendet eine
     * empirische Formel, die für Temperaturen über dem Gefrierpunkt gültig ist.
     */
    double computeSaturationVapourPressure(double tempC);

    /**
     * Berechnet das Dampfdruckdefizit (VPD) in kPa aus Lufttemperatur
     * und relativer Luftfeuchtigkeit (%). Werte für die relative Luftfeuchtigkeit außerhalb von 0–100 werden
     * begrenzt. Siehe vpd_research.md für die Herleitung.
     */
    double computeVpd(double tempC, double relHumidity);

    /**
     * Gibt die Standard-VPD-Schwellenwerte für ein bestimmtes Wachstumsstadium zurück.
     * Diese Schwellenwerte definieren grüne und gelbe Bereiche und können beim
     * Klassifizieren eines VPD-Werts überschrieben werden.
     */
    VpdThreshold getDefaultThresholds(GrowthStage stage);

    /**
     * Klassifiziert einen VPD-Wert als "GREEN", "YELLOW" oder "RED" entsprechend
     * den Schwellenwerten für das angegebene Wachstumsstadium. Ein Zeiger auf
     * benutzerdefinierte Schwellenwerte kann übergeben werden, um die Standardwerte zu überschreiben.
     * Gibt einen String-Literal zurück.
     */
    const char* classifyVpd(double vpd, GrowthStage stage, const VpdThreshold* custom = nullptr);

    /**
     * Berechnet die relative Luftfeuchtigkeit (%), die erforderlich ist, um einen Ziel-VPD bei einer
     * gegebenen Temperatur zu erreichen. Gibt einen Wert zwischen 0 und 100 zurück.
     */
    double computeRelativeHumidityForVpd(double tempC, double targetVpd);

    /**
     * Berechnet den Taupunkt in Grad Celsius aus Lufttemperatur und
     * relativer Luftfeuchtigkeit. Verwendet eine Magnus-Approximation (siehe Forschungsdatei).
     */
    double computeDewPoint(double tempC, double relHumidity);
} // namespace vpd_calc

