#pragma once

#include <cmath>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>

namespace hpop_perturbations
{
namespace atmosphere
{

/**
 * @brief Exponential atmosphere model
 *
 * Simple exponential atmosphere model based on altitude bands.
 * Provides atmospheric density for drag calculations.
 *
 * Reference: US Standard Atmosphere 1976
 */
class ExponentialAtmosphere
{
public:
    /**
     * @brief Get atmospheric density at given altitude
     * @param altitude Geometric altitude above Earth's surface [m]
     * @return Atmospheric density [kg/m^3]
     */
    static double density(double altitude)
    {
        double h_km = altitude / 1000.0;  // Convert to km

        if (h_km < 0.0)
        {
            h_km = 0.0;  // Clamp to surface
        }

        // Find the appropriate altitude band
        size_t idx = 0;
        for (size_t i = 0; i < NUM_BANDS - 1; ++i)
        {
            if (h_km >= altitude_bands_[i].h0 && h_km < altitude_bands_[i + 1].h0)
            {
                idx = i;
                break;
            }
            if (i == NUM_BANDS - 2)
            {
                idx = NUM_BANDS - 1;  // Use last band for very high altitudes
            }
        }

        // Very high altitude - use extrapolation
        if (h_km >= altitude_bands_[NUM_BANDS - 1].h0)
        {
            idx = NUM_BANDS - 1;
        }

        const auto& band = altitude_bands_[idx];
        double rho = band.rho0 * std::exp(-(h_km - band.h0) / band.H);

        return rho;
    }

    /**
     * @brief Get atmospheric scale height at given altitude
     * @param altitude Geometric altitude above Earth's surface [m]
     * @return Scale height [m]
     */
    static double scaleHeight(double altitude)
    {
        double h_km = altitude / 1000.0;

        if (h_km < 0.0) h_km = 0.0;

        // Find the appropriate altitude band
        for (size_t i = 0; i < NUM_BANDS - 1; ++i)
        {
            if (h_km >= altitude_bands_[i].h0 && h_km < altitude_bands_[i + 1].h0)
            {
                return altitude_bands_[i].H * 1000.0;  // Convert to meters
            }
        }

        return altitude_bands_[NUM_BANDS - 1].H * 1000.0;
    }

private:
    struct AtmosphereBand
    {
        double h0;    // Base altitude [km]
        double rho0;  // Reference density [kg/m^3]
        double H;     // Scale height [km]
    };

    static constexpr size_t NUM_BANDS = 28;

    // US Standard Atmosphere 1976 exponential model coefficients
    static constexpr std::array<AtmosphereBand, NUM_BANDS> altitude_bands_ = {{
        {0,     1.225,          7.249},
        {25,    3.899e-2,       6.349},
        {30,    1.774e-2,       6.682},
        {40,    3.972e-3,       7.554},
        {50,    1.057e-3,       8.382},
        {60,    3.206e-4,       7.714},
        {70,    8.770e-5,       6.549},
        {80,    1.905e-5,       5.799},
        {90,    3.396e-6,       5.382},
        {100,   5.297e-7,       5.877},
        {110,   9.661e-8,       7.263},
        {120,   2.438e-8,       9.473},
        {130,   8.484e-9,       12.636},
        {140,   3.845e-9,       16.149},
        {150,   2.070e-9,       22.523},
        {180,   5.464e-10,      29.740},
        {200,   2.789e-10,      37.105},
        {250,   7.248e-11,      45.546},
        {300,   2.418e-11,      53.628},
        {350,   9.518e-12,      53.298},
        {400,   3.725e-12,      58.515},
        {450,   1.585e-12,      60.828},
        {500,   6.967e-13,      63.822},
        {600,   1.454e-13,      71.835},
        {700,   3.614e-14,      88.667},
        {800,   1.170e-14,      124.64},
        {900,   5.245e-15,      181.05},
        {1000,  3.019e-15,      268.00}
    }};
};

/**
 * @brief Harris-Priester atmosphere model
 *
 * More accurate model that accounts for solar activity and
 * diurnal variations in atmospheric density.
 */
class HarrisPriesterAtmosphere
{
public:
    /**
     * @brief Set solar activity parameters
     * @param f107 F10.7 solar flux [SFU]
     * @param f107a 81-day average F10.7 [SFU]
     */
    void setSolarActivity(double f107, double f107a)
    {
        f107_ = f107;
        f107a_ = f107a;
    }

    /**
     * @brief Get atmospheric density with diurnal variation
     * @param altitude Geometric altitude [m]
     * @param sun_ra Right ascension of Sun [rad]
     * @param sun_dec Declination of Sun [rad]
     * @param sat_ra Right ascension of satellite [rad]
     * @param sat_dec Declination of satellite [rad]
     * @return Atmospheric density [kg/m^3]
     */
    double density(double altitude, double sun_ra, double sun_dec,
                   double sat_ra, double sat_dec) const
    {
        double h_km = altitude / 1000.0;

        if (h_km < 100.0) h_km = 100.0;
        if (h_km > 1000.0) return 1e-20;  // Negligible density

        // Find altitude band
        int i = findAltitudeBand(h_km);
        if (i < 0) return 1e-20;

        // Interpolate min/max density
        double h0 = hp_altitudes_[i];
        double h1 = hp_altitudes_[i + 1];
        double t = (h_km - h0) / (h1 - h0);

        double rho_min = hp_rho_min_[i] * std::exp(std::log(hp_rho_min_[i + 1] / hp_rho_min_[i]) * t);
        double rho_max = hp_rho_max_[i] * std::exp(std::log(hp_rho_max_[i + 1] / hp_rho_max_[i]) * t);

        // Calculate lag angle (density maximum lags sun by about 2 hours)
        const double lag_angle = 30.0 * M_PI / 180.0;  // 30 degrees

        // Calculate angle between satellite and density maximum (bulge)
        double cos_psi = std::sin(sat_dec) * std::sin(sun_dec) +
                         std::cos(sat_dec) * std::cos(sun_dec) * std::cos(sat_ra - sun_ra + lag_angle);

        // Diurnal factor (n = 2 for lower atmosphere, 6 for upper)
        double n = (h_km < 200.0) ? 2.0 : (h_km < 600.0) ? 4.0 : 6.0;
        double psi = std::acos(std::clamp(cos_psi, -1.0, 1.0));
        double F = std::pow(0.5 * (1.0 + cos_psi), n / 2.0);

        // Final density
        double rho = rho_min + (rho_max - rho_min) * F;

        return rho;
    }

    /**
     * @brief Get density using simpler model (no diurnal variation)
     */
    double densitySimple(double altitude) const
    {
        double h_km = altitude / 1000.0;

        if (h_km < 100.0) return ExponentialAtmosphere::density(altitude);
        if (h_km > 1000.0) return 1e-20;

        int i = findAltitudeBand(h_km);
        if (i < 0) return 1e-20;

        double h0 = hp_altitudes_[i];
        double h1 = hp_altitudes_[i + 1];
        double t = (h_km - h0) / (h1 - h0);

        // Use average of min/max for moderate solar activity
        double rho_avg_0 = 0.5 * (hp_rho_min_[i] + hp_rho_max_[i]);
        double rho_avg_1 = 0.5 * (hp_rho_min_[i + 1] + hp_rho_max_[i + 1]);

        return rho_avg_0 * std::exp(std::log(rho_avg_1 / rho_avg_0) * t);
    }

private:
    double f107_{150.0};   // F10.7 solar flux (moderate activity)
    double f107a_{150.0};  // 81-day average

    static constexpr int NUM_HP_BANDS = 50;

    // Harris-Priester altitude points [km]
    static constexpr std::array<double, NUM_HP_BANDS> hp_altitudes_ = {
        100, 120, 130, 140, 150, 160, 170, 180, 190, 200,
        210, 220, 230, 240, 250, 260, 270, 280, 290, 300,
        320, 340, 360, 380, 400, 420, 440, 460, 480, 500,
        520, 540, 560, 580, 600, 620, 640, 660, 680, 700,
        720, 740, 760, 780, 800, 840, 880, 920, 960, 1000
    };

    // Minimum density (solar minimum) [kg/m^3]
    static constexpr std::array<double, NUM_HP_BANDS> hp_rho_min_ = {
        4.974e-7, 2.490e-8, 8.377e-9, 3.899e-9, 2.122e-9, 1.263e-9, 8.008e-10, 5.283e-10, 3.617e-10, 2.557e-10,
        1.839e-10, 1.341e-10, 9.949e-11, 7.488e-11, 5.709e-11, 4.403e-11, 3.430e-11, 2.697e-11, 2.139e-11, 1.708e-11,
        1.099e-11, 7.214e-12, 4.824e-12, 3.274e-12, 2.249e-12, 1.558e-12, 1.091e-12, 7.701e-13, 5.474e-13, 3.916e-13,
        2.819e-13, 2.042e-13, 1.488e-13, 1.092e-13, 8.070e-14, 6.012e-14, 4.519e-14, 3.430e-14, 2.632e-14, 2.043e-14,
        1.607e-14, 1.281e-14, 1.036e-14, 8.496e-15, 7.069e-15, 4.680e-15, 3.200e-15, 2.210e-15, 1.560e-15, 1.150e-15
    };

    // Maximum density (solar maximum) [kg/m^3]
    static constexpr std::array<double, NUM_HP_BANDS> hp_rho_max_ = {
        4.974e-7, 2.490e-8, 8.710e-9, 4.059e-9, 2.215e-9, 1.344e-9, 8.758e-10, 6.010e-10, 4.297e-10, 3.162e-10,
        2.396e-10, 1.853e-10, 1.455e-10, 1.157e-10, 9.308e-11, 7.555e-11, 6.182e-11, 5.095e-11, 4.226e-11, 3.526e-11,
        2.511e-11, 1.819e-11, 1.337e-11, 9.955e-12, 7.492e-12, 5.684e-12, 4.355e-12, 3.362e-12, 2.612e-12, 2.042e-12,
        1.605e-12, 1.267e-12, 1.005e-12, 7.997e-13, 6.390e-13, 5.123e-13, 4.121e-13, 3.325e-13, 2.691e-13, 2.185e-13,
        1.779e-13, 1.452e-13, 1.190e-13, 9.776e-14, 8.059e-14, 5.741e-14, 4.210e-14, 3.130e-14, 2.360e-14, 1.810e-14
    };

    int findAltitudeBand(double h_km) const
    {
        for (int i = 0; i < NUM_HP_BANDS - 1; ++i)
        {
            if (h_km >= hp_altitudes_[i] && h_km < hp_altitudes_[i + 1])
            {
                return i;
            }
        }
        return -1;
    }
};

} // namespace atmosphere
} // namespace hpop_perturbations
