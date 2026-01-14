#pragma once

#include <cmath>

namespace hpop_core
{

/// Physical and astronomical constants
namespace constants
{

// Mathematical constants
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double HALF_PI = PI / 2.0;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Earth parameters (WGS84)
constexpr double EARTH_RADIUS_EQ = 6378137.0;           // [m] Equatorial radius
constexpr double EARTH_RADIUS_POLAR = 6356752.3142;    // [m] Polar radius
constexpr double EARTH_FLATTENING = 1.0 / 298.257223563;  // [-] Flattening
constexpr double EARTH_ECCENTRICITY_SQ = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING * EARTH_FLATTENING;
constexpr double EARTH_MU = 3.986004418e14;             // [m^3/s^2] Gravitational parameter
constexpr double EARTH_J2 = 1.082626683e-3;             // [-] J2 coefficient
constexpr double EARTH_OMEGA = 7.2921150e-5;            // [rad/s] Angular velocity
constexpr double OMEGA_EARTH = EARTH_OMEGA;              // Alias for EARTH_OMEGA

// Sun parameters
constexpr double SUN_MU = 1.32712440018e20;             // [m^3/s^2] Gravitational parameter
constexpr double SUN_RADIUS = 6.96e8;                   // [m] Radius
constexpr double AU = 1.495978707e11;                   // [m] Astronomical Unit

// Moon parameters
constexpr double MOON_MU = 4.9028e12;                   // [m^3/s^2] Gravitational parameter
constexpr double MOON_RADIUS = 1.7374e6;                // [m] Radius
constexpr double MOON_SEMI_MAJOR_AXIS = 3.844e8;        // [m] Mean distance from Earth

// Solar radiation pressure
constexpr double SOLAR_FLUX = 1361.0;                   // [W/m^2] Solar constant at 1 AU
constexpr double SOLAR_PRESSURE = 4.56e-6;              // [N/m^2] Solar radiation pressure at 1 AU
constexpr double SPEED_OF_LIGHT = 299792458.0;          // [m/s] Speed of light

// Time constants
constexpr double SECONDS_PER_DAY = 86400.0;
constexpr double SECONDS_PER_HOUR = 3600.0;
constexpr double SECONDS_PER_MINUTE = 60.0;
constexpr double DAYS_PER_JULIAN_CENTURY = 36525.0;
constexpr double J2000_JD = 2451545.0;                  // Julian date of J2000 epoch

// Atmospheric model parameters
constexpr double EARTH_ATMOSPHERE_HEIGHT = 1000000.0;   // [m] Nominal atmosphere height

// Default spacecraft parameters
constexpr double DEFAULT_MASS = 1000.0;                 // [kg]
constexpr double DEFAULT_AREA = 10.0;                   // [m^2]
constexpr double DEFAULT_CD = 2.2;                      // [-] Drag coefficient
constexpr double DEFAULT_CR = 1.5;                      // [-] Reflectivity coefficient

} // namespace constants

/// Utility functions
namespace utils
{

/// Normalize angle to [0, 2*PI)
inline double normalizeAngle(double angle)
{
    angle = std::fmod(angle, constants::TWO_PI);
    if (angle < 0.0)
        angle += constants::TWO_PI;
    return angle;
}

/// Normalize angle to [-PI, PI)
inline double normalizeAnglePM(double angle)
{
    angle = normalizeAngle(angle);
    if (angle >= constants::PI)
        angle -= constants::TWO_PI;
    return angle;
}

/// Convert degrees to radians
inline double deg2rad(double deg)
{
    return deg * constants::DEG_TO_RAD;
}

/// Convert radians to degrees
inline double rad2deg(double rad)
{
    return rad * constants::RAD_TO_DEG;
}

/// Sign function
template<typename T>
inline int sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

} // namespace utils

} // namespace hpop_core
