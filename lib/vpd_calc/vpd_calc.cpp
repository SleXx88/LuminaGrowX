/*
 * vpd_calc.cpp
 *
 * Implementierung der VPD-Berechnungsbibliothek. Eine ausführliche
 * Erklärung der hier verwendeten Formeln findet sich in vpd_research.md. Alle
 * Gleitkommaberechnungen verwenden double-Präzision; auf Mikrocontrollern mit
 * eingeschränkten Ressourcen können diese auf float umgestellt werden, was jedoch
 * auf Kosten der Genauigkeit geht.
 */

#include "vpd_calc.h"

#include <cmath>

namespace vpd_calc
{
    // --- VpdThreshold Hilfsmethoden ---

    bool VpdThreshold::isGreen(double vpd) const
    {
        return (vpd >= greenMin) && (vpd <= greenMax);
    }

    bool VpdThreshold::isYellow(double vpd) const
    {
        return (vpd >= yellowMin) && (vpd <= yellowMax);
    }

    bool VpdThreshold::isRed(double vpd) const
    {
        return !isGreen(vpd) && !isYellow(vpd);
    }

    // --- Kernberechnungen ---

    double computeSaturationVapourPressure(double tempC)
    {
        // Koeffizienten basierend auf Monteith & Unsworth (1990)
        constexpr double A = 17.2694;
        constexpr double B = 237.3;
        constexpr double SVP0 = 610.78; // Pa
        double exponent = (A * tempC) / (tempC + B);
        double svpPa = SVP0 * std::exp(exponent);
        return svpPa / 1000.0; // Umrechnung in kPa
    }

    double computeVpd(double tempC, double relHumidity)
    {
        // Relative Luftfeuchtigkeit auf [0,100] begrenzen
        if (relHumidity < 0.0) relHumidity = 0.0;
        if (relHumidity > 100.0) relHumidity = 100.0;
        double svp = computeSaturationVapourPressure(tempC);
        double vpd = svp * (1.0 - (relHumidity / 100.0));
        return vpd;
    }

    VpdThreshold getDefaultThresholds(GrowthStage stage)
    {
        switch (stage)
        {
        case GrowthStage::Seedling:
            // Keimlinge/Klone: optimal 0.4–0.8 kPa; Warnbereich ±0.1 kPa
            return VpdThreshold{0.4, 0.8, 0.3, 0.9};
        case GrowthStage::Vegetative:
            // Vegetativ: optimal 0.8–1.2 kPa; Warnbereich ±0.2 kPa
            return VpdThreshold{0.8, 1.2, 0.6, 1.4};
        case GrowthStage::Flowering:
        default:
            // Blüte: optimal 1.2–1.5 kPa; Warnbereich ±0.2 kPa
            return VpdThreshold{1.2, 1.5, 1.0, 1.6};
        }
    }

    const char* classifyVpd(double vpd, GrowthStage stage, const VpdThreshold* custom)
    {
        VpdThreshold thresholds = custom ? *custom : getDefaultThresholds(stage);
        if (thresholds.isGreen(vpd))
            return "GREEN";
        if (thresholds.isYellow(vpd))
            return "YELLOW";
        return "RED";
    }

    double computeRelativeHumidityForVpd(double tempC, double targetVpd)
    {
        double svp = computeSaturationVapourPressure(tempC);
        if (targetVpd >= svp)
        {
            // Ziel-VPD darf den Sättigungsdampfdruck nicht überschreiten
            return 0.0;
        }
        double rh = (1.0 - (targetVpd / svp)) * 100.0;
        if (rh < 0.0) rh = 0.0;
        if (rh > 100.0) rh = 100.0;
        return rh;
    }

    double computeDewPoint(double tempC, double relHumidity)
    {
        // Magnus-Tetens-Annäherungskoeffizienten
        constexpr double A = 17.27;
        constexpr double B = 237.7;
        if (relHumidity <= 0.0)
        {
            return tempC; // Taupunkt undefiniert; Umgebungstemperatur zurückgeben
        }
        double alpha = std::log(relHumidity / 100.0) + (A * tempC) / (B + tempC);
        double dew = (B * alpha) / (A - alpha);
        return dew;
    }
} // namespace vpd_calc
