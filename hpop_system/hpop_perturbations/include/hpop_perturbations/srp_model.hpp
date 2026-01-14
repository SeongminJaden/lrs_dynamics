#pragma once

#include <cmath>
#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"

namespace hpop_perturbations
{

/**
 * @brief Solar Radiation Pressure (SRP) acceleration model
 *
 * Computes acceleration due to solar radiation pressure using
 * the cannonball model with cylindrical Earth shadow.
 *
 * a = -P_sr * Cr * (A/m) * (AU/r_sun)^2 * r_sat_sun_hat
 *
 * where P_sr = solar radiation pressure at 1 AU = 4.56e-6 N/m^2
 */
class SRPModel
{
public:
    // Solar radiation pressure at 1 AU [N/m^2]
    static constexpr double P_SR_AU = 4.56e-6;

    // Astronomical Unit [m]
    static constexpr double AU = 1.495978707e11;

    SRPModel()
        : cr_(1.0), area_(10.0), mass_(1000.0)
    {
    }

    /**
     * @brief Set spacecraft optical parameters
     * @param cr Reflectivity coefficient (0 = absorbing, 1 = reflecting, 1.44 = typical)
     * @param area Cross-sectional area [m^2]
     * @param mass Spacecraft mass [kg]
     */
    void setSpacecraftParams(double cr, double area, double mass)
    {
        cr_ = cr;
        area_ = area;
        mass_ = mass;
    }

    /**
     * @brief Set reflectivity coefficient
     */
    void setReflectivity(double cr) { cr_ = cr; }

    /**
     * @brief Set cross-sectional area [m^2]
     */
    void setCrossSection(double area) { area_ = area; }

    /**
     * @brief Set spacecraft mass [kg]
     */
    void setMass(double mass) { mass_ = mass; }

    /**
     * @brief Compute SRP acceleration in ECI frame
     *
     * @param state Current state vector in ECI frame
     * @param sun_pos_eci Sun position in ECI frame [m]
     * @return Acceleration due to SRP [m/s^2] in ECI frame
     */
    hpop_core::Vec3 acceleration(const hpop_core::StateVector& state,
                                  const hpop_core::Vec3& sun_pos_eci) const
    {
        using namespace hpop_core;

        const Vec3& r_sat = state.position;

        // Vector from satellite to Sun
        Vec3 r_sat_sun = sun_pos_eci - r_sat;
        double r_sat_sun_mag = r_sat_sun.norm();

        // Check shadow condition
        double shadow_factor = shadowFunction(r_sat, sun_pos_eci);

        if (shadow_factor < 1e-10)
        {
            return Vec3(0.0, 0.0, 0.0);  // In complete shadow
        }

        // Unit vector from satellite to Sun
        Vec3 r_hat = r_sat_sun / r_sat_sun_mag;

        // SRP acceleration magnitude
        // a = P_sr * Cr * (A/m) * (AU/r_sun)^2
        double au_ratio = AU / r_sat_sun_mag;
        double a_mag = P_SR_AU * cr_ * (area_ / mass_) * au_ratio * au_ratio;

        // SRP acceleration (pointing away from Sun)
        Vec3 a_srp = r_hat * (-a_mag * shadow_factor);

        return a_srp;
    }

    /**
     * @brief Shadow function (cylindrical model)
     *
     * Returns:
     * - 1.0 if satellite is in full sunlight
     * - 0.0 if satellite is in umbra (total shadow)
     * - 0.0 < nu < 1.0 if in penumbra
     *
     * @param r_sat Satellite position ECI [m]
     * @param r_sun Sun position ECI [m]
     * @return Shadow factor [0, 1]
     */
    double shadowFunction(const hpop_core::Vec3& r_sat,
                          const hpop_core::Vec3& r_sun) const
    {
        using namespace hpop_core;

        // Sun angular radius as seen from Earth (approximately)
        const double SUN_RADIUS = 6.96e8;  // [m]

        // Vector from Earth to Sun
        double r_sun_mag = r_sun.norm();
        Vec3 e_sun = r_sun / r_sun_mag;

        // Satellite position magnitude
        double r_sat_mag = r_sat.norm();

        // Project satellite position onto Sun direction
        double sat_sun_dot = r_sat.dot(e_sun);

        // If satellite is on the sunward side, no shadow
        if (sat_sun_dot > 0)
        {
            return 1.0;
        }

        // Distance from satellite to Earth-Sun line
        Vec3 r_perp = r_sat - e_sun * sat_sun_dot;
        double d_perp = r_perp.norm();

        // Angular radii
        double theta_sun = std::asin(SUN_RADIUS / r_sun_mag);
        double theta_earth = std::asin(constants::EARTH_RADIUS_EQ / r_sat_mag);

        // Apparent angle between Earth center and Sun as seen from satellite
        // For cylindrical model, just check if satellite is behind Earth
        if (d_perp < constants::EARTH_RADIUS_EQ)
        {
            // More refined: penumbra calculation
            double theta_sat = std::asin(d_perp / r_sat_mag);

            if (theta_sat < theta_earth - theta_sun)
            {
                return 0.0;  // Total umbra
            }
            else if (theta_sat < theta_earth + theta_sun)
            {
                // Penumbra - linear transition (simple approximation)
                double x = (theta_sat - (theta_earth - theta_sun)) / (2.0 * theta_sun);
                return x;
            }
        }

        return 1.0;  // Full sunlight
    }

    /**
     * @brief Check if satellite is in eclipse
     */
    bool isInEclipse(const hpop_core::Vec3& r_sat,
                     const hpop_core::Vec3& r_sun) const
    {
        return shadowFunction(r_sat, r_sun) < 0.99;
    }

private:
    double cr_;    // Reflectivity coefficient
    double area_;  // Cross-sectional area [m^2]
    double mass_;  // Spacecraft mass [kg]
};

/**
 * @brief Simple sun position calculator
 *
 * Provides approximate Sun position in ECI for SRP calculations.
 * For high accuracy, use JPL DE ephemeris.
 */
class SunPosition
{
public:
    /**
     * @brief Get Sun position in ECI frame
     *
     * Uses simplified analytical model.
     *
     * @param jd Julian date
     * @return Sun position in ECI frame [m]
     */
    static hpop_core::Vec3 getSunPositionECI(double jd)
    {
        using namespace hpop_core;

        // Julian centuries from J2000.0
        double T = (jd - 2451545.0) / 36525.0;

        // Mean longitude of the Sun [deg]
        double L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;
        L0 = std::fmod(L0, 360.0);
        if (L0 < 0) L0 += 360.0;

        // Mean anomaly of the Sun [deg]
        double M = 357.52911 + 35999.05029 * T - 0.0001537 * T * T;
        M = std::fmod(M, 360.0);
        if (M < 0) M += 360.0;
        double M_rad = M * constants::DEG_TO_RAD;

        // Equation of center [deg]
        double C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * std::sin(M_rad)
                 + (0.019993 - 0.000101 * T) * std::sin(2.0 * M_rad)
                 + 0.000289 * std::sin(3.0 * M_rad);

        // Sun's true longitude [deg]
        double sun_lon = L0 + C;
        double sun_lon_rad = sun_lon * constants::DEG_TO_RAD;

        // Sun's distance from Earth [AU]
        double e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;
        double r_au = (1.000001018 * (1.0 - e * e)) / (1.0 + e * std::cos(M_rad));
        double r_m = r_au * 1.495978707e11;  // Convert to meters

        // Obliquity of ecliptic [deg]
        double epsilon = 23.439291 - 0.0130042 * T;
        double epsilon_rad = epsilon * constants::DEG_TO_RAD;

        // Sun position in ECI (equatorial coordinates)
        double cos_lon = std::cos(sun_lon_rad);
        double sin_lon = std::sin(sun_lon_rad);
        double cos_eps = std::cos(epsilon_rad);
        double sin_eps = std::sin(epsilon_rad);

        Vec3 sun_eci;
        sun_eci.x = r_m * cos_lon;
        sun_eci.y = r_m * sin_lon * cos_eps;
        sun_eci.z = r_m * sin_lon * sin_eps;

        return sun_eci;
    }

    /**
     * @brief Get Sun's right ascension and declination
     * @param jd Julian date
     * @param ra Output: right ascension [rad]
     * @param dec Output: declination [rad]
     */
    static void getSunRaDec(double jd, double& ra, double& dec)
    {
        hpop_core::Vec3 sun = getSunPositionECI(jd);
        double r = sun.norm();
        ra = std::atan2(sun.y, sun.x);
        dec = std::asin(sun.z / r);
    }
};

} // namespace hpop_perturbations
