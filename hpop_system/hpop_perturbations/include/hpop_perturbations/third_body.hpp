#pragma once

#include <cmath>
#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "srp_model.hpp"  // For SunPosition

namespace hpop_perturbations
{

/**
 * @brief Third-body gravitational perturbation model
 *
 * Computes gravitational acceleration due to Moon and Sun.
 * Uses simplified analytical ephemeris for body positions.
 */
class ThirdBodyModel
{
public:
    // Gravitational parameters [m^3/s^2]
    static constexpr double MU_SUN = 1.32712440018e20;
    static constexpr double MU_MOON = 4.9028695e12;

    ThirdBodyModel()
        : enable_sun_(true), enable_moon_(true)
    {
    }

    /**
     * @brief Enable/disable Sun perturbation
     */
    void enableSun(bool enable) { enable_sun_ = enable; }

    /**
     * @brief Enable/disable Moon perturbation
     */
    void enableMoon(bool enable) { enable_moon_ = enable; }

    /**
     * @brief Compute third-body acceleration
     *
     * @param state Current satellite state in ECI
     * @param jd Julian date
     * @return Total third-body acceleration [m/s^2] in ECI
     */
    hpop_core::Vec3 acceleration(const hpop_core::StateVector& state, double jd) const
    {
        hpop_core::Vec3 a_total{0.0, 0.0, 0.0};

        if (enable_sun_)
        {
            hpop_core::Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
            a_total = a_total + thirdBodyAccel(state.position, sun_pos, MU_SUN);
        }

        if (enable_moon_)
        {
            hpop_core::Vec3 moon_pos = getMoonPositionECI(jd);
            a_total = a_total + thirdBodyAccel(state.position, moon_pos, MU_MOON);
        }

        return a_total;
    }

    /**
     * @brief Compute Sun gravitational acceleration only
     */
    hpop_core::Vec3 sunAcceleration(const hpop_core::StateVector& state, double jd) const
    {
        hpop_core::Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
        return thirdBodyAccel(state.position, sun_pos, MU_SUN);
    }

    /**
     * @brief Compute Moon gravitational acceleration only
     */
    hpop_core::Vec3 moonAcceleration(const hpop_core::StateVector& state, double jd) const
    {
        hpop_core::Vec3 moon_pos = getMoonPositionECI(jd);
        return thirdBodyAccel(state.position, moon_pos, MU_MOON);
    }

    /**
     * @brief Get Moon position in ECI frame
     *
     * Simplified analytical lunar ephemeris.
     *
     * @param jd Julian date
     * @return Moon position in ECI [m]
     */
    static hpop_core::Vec3 getMoonPositionECI(double jd)
    {
        using namespace hpop_core;

        // Julian centuries from J2000.0
        double T = (jd - 2451545.0) / 36525.0;

        // Fundamental arguments [deg]
        // Mean longitude of Moon
        double L0 = 218.3164477 + 481267.88123421 * T
                  - 0.0015786 * T * T + T * T * T / 538841.0;
        L0 = std::fmod(L0, 360.0);
        if (L0 < 0) L0 += 360.0;

        // Mean anomaly of Moon
        double l = 134.9633964 + 477198.8675055 * T
                 + 0.0087414 * T * T + T * T * T / 69699.0;
        l = std::fmod(l, 360.0) * constants::DEG_TO_RAD;

        // Mean anomaly of Sun
        double lp = 357.5291092 + 35999.0502909 * T
                  - 0.0001536 * T * T;
        lp = std::fmod(lp, 360.0) * constants::DEG_TO_RAD;

        // Moon's mean argument of latitude
        double F = 93.2720950 + 483202.0175233 * T
                 - 0.0036539 * T * T;
        F = std::fmod(F, 360.0) * constants::DEG_TO_RAD;

        // Mean elongation of Moon from Sun
        double D = 297.8501921 + 445267.1114034 * T
                 - 0.0018819 * T * T;
        D = std::fmod(D, 360.0) * constants::DEG_TO_RAD;

        // Simplified longitude perturbations [deg]
        double dL = 6.288774 * std::sin(l)
                  + 1.274027 * std::sin(2.0 * D - l)
                  + 0.658314 * std::sin(2.0 * D)
                  + 0.213618 * std::sin(2.0 * l)
                  - 0.185116 * std::sin(lp)
                  - 0.114332 * std::sin(2.0 * F);

        // Simplified latitude perturbations [deg]
        double dB = 5.128122 * std::sin(F)
                  + 0.280602 * std::sin(l + F)
                  + 0.277693 * std::sin(l - F)
                  + 0.173237 * std::sin(2.0 * D - F);

        // Simplified distance perturbations [km]
        double dR = -20905.355 * std::cos(l)
                  - 3699.111 * std::cos(2.0 * D - l)
                  - 2955.968 * std::cos(2.0 * D)
                  - 569.925 * std::cos(2.0 * l);

        // Moon geocentric coordinates
        double lambda = (L0 + dL) * constants::DEG_TO_RAD;  // Ecliptic longitude
        double beta = dB * constants::DEG_TO_RAD;           // Ecliptic latitude
        double r_km = 385000.56 + dR;                       // Distance [km]
        double r_m = r_km * 1000.0;

        // Obliquity of ecliptic
        double epsilon = (23.439291 - 0.0130042 * T) * constants::DEG_TO_RAD;

        // Convert from ecliptic to equatorial (ECI)
        double cos_lam = std::cos(lambda);
        double sin_lam = std::sin(lambda);
        double cos_bet = std::cos(beta);
        double sin_bet = std::sin(beta);
        double cos_eps = std::cos(epsilon);
        double sin_eps = std::sin(epsilon);

        // Ecliptic position
        double x_ecl = r_m * cos_bet * cos_lam;
        double y_ecl = r_m * cos_bet * sin_lam;
        double z_ecl = r_m * sin_bet;

        // Rotate from ecliptic to equatorial
        Vec3 moon_eci;
        moon_eci.x = x_ecl;
        moon_eci.y = y_ecl * cos_eps - z_ecl * sin_eps;
        moon_eci.z = y_ecl * sin_eps + z_ecl * cos_eps;

        return moon_eci;
    }

private:
    bool enable_sun_;
    bool enable_moon_;

    /**
     * @brief Compute third-body gravitational acceleration
     *
     * Uses the formulation:
     * a = mu * (s/|s|^3 - r_body/|r_body|^3)
     *
     * where s = r_body - r_sat (vector from satellite to body)
     *
     * @param r_sat Satellite position ECI [m]
     * @param r_body Third body position ECI [m]
     * @param mu Third body gravitational parameter [m^3/s^2]
     * @return Acceleration [m/s^2]
     */
    hpop_core::Vec3 thirdBodyAccel(const hpop_core::Vec3& r_sat,
                                    const hpop_core::Vec3& r_body,
                                    double mu) const
    {
        // Vector from satellite to third body
        hpop_core::Vec3 s = r_body - r_sat;
        double s_mag = s.norm();
        double s_mag3 = s_mag * s_mag * s_mag;

        // Vector from Earth to third body
        double r_body_mag = r_body.norm();
        double r_body_mag3 = r_body_mag * r_body_mag * r_body_mag;

        // Third-body acceleration
        hpop_core::Vec3 a;
        a.x = mu * (s.x / s_mag3 - r_body.x / r_body_mag3);
        a.y = mu * (s.y / s_mag3 - r_body.y / r_body_mag3);
        a.z = mu * (s.z / s_mag3 - r_body.z / r_body_mag3);

        return a;
    }
};

} // namespace hpop_perturbations
