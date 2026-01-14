#pragma once

#include <cmath>
#include <array>
#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/time_system.hpp"

namespace hpop_core
{

/// Reference frame types
enum class ReferenceFrame
{
    ECI_J2000,   // Earth-Centered Inertial (J2000)
    ECEF_ITRF,   // Earth-Centered Earth-Fixed (ITRF)
    LVLH,        // Local Vertical Local Horizontal
    RTN,         // Radial-Transverse-Normal (same as LVLH, satellite-centered)
    NTW          // Normal-Transverse-Wedge (velocity-aligned)
};

/// Geodetic coordinates
struct GeodeticCoord
{
    double latitude{0.0};   // [rad] Geodetic latitude (-PI/2 to PI/2)
    double longitude{0.0};  // [rad] Geodetic longitude (-PI to PI)
    double altitude{0.0};   // [m] Height above WGS84 ellipsoid
};

/// 3x3 Rotation matrix
struct RotationMatrix
{
    std::array<std::array<double, 3>, 3> m{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    RotationMatrix() = default;

    /// Apply rotation to vector
    Vec3 rotate(const Vec3& v) const
    {
        return Vec3{
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    /// Transpose (inverse for orthogonal matrices)
    RotationMatrix transpose() const
    {
        RotationMatrix r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    /// Matrix multiplication
    RotationMatrix operator*(const RotationMatrix& other) const
    {
        RotationMatrix result;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                result.m[i][j] = 0;
                for (int k = 0; k < 3; ++k)
                    result.m[i][j] += m[i][k] * other.m[k][j];
            }
        }
        return result;
    }

    /// Create rotation matrix around X axis
    static RotationMatrix rotX(double angle)
    {
        RotationMatrix r;
        double c = std::cos(angle);
        double s = std::sin(angle);
        r.m[1][1] = c;  r.m[1][2] = s;
        r.m[2][1] = -s; r.m[2][2] = c;
        return r;
    }

    /// Create rotation matrix around Y axis
    static RotationMatrix rotY(double angle)
    {
        RotationMatrix r;
        double c = std::cos(angle);
        double s = std::sin(angle);
        r.m[0][0] = c;  r.m[0][2] = -s;
        r.m[2][0] = s;  r.m[2][2] = c;
        return r;
    }

    /// Create rotation matrix around Z axis
    static RotationMatrix rotZ(double angle)
    {
        RotationMatrix r;
        double c = std::cos(angle);
        double s = std::sin(angle);
        r.m[0][0] = c;  r.m[0][1] = s;
        r.m[1][0] = -s; r.m[1][1] = c;
        return r;
    }
};

/// Coordinate transformation utilities
class CoordinateFrames
{
public:
    //=========================================================================
    // ECI <-> ECEF conversions
    //=========================================================================

    /// Convert ECI to ECEF at given Julian Date
    static StateVector eciToEcef(const StateVector& eci, double jd)
    {
        double gmst = TimeSystem::gmst(jd);
        RotationMatrix R = RotationMatrix::rotZ(gmst);

        StateVector ecef;
        ecef.position = R.rotate(eci.position);

        // Velocity transformation includes Earth rotation
        Vec3 omega{0, 0, constants::EARTH_OMEGA};
        ecef.velocity = R.rotate(eci.velocity) - omega.cross(ecef.position);
        ecef.epoch = jd;

        return ecef;
    }

    /// Convert ECEF to ECI at given Julian Date
    static StateVector ecefToEci(const StateVector& ecef, double jd)
    {
        double gmst = TimeSystem::gmst(jd);
        RotationMatrix R = RotationMatrix::rotZ(-gmst);

        StateVector eci;
        eci.position = R.rotate(ecef.position);

        // Velocity transformation includes Earth rotation
        Vec3 omega{0, 0, constants::EARTH_OMEGA};
        Vec3 v_ecef_corrected = ecef.velocity + omega.cross(ecef.position);
        eci.velocity = R.rotate(v_ecef_corrected);
        eci.epoch = jd;

        return eci;
    }

    //=========================================================================
    // ECEF <-> Geodetic conversions
    //=========================================================================

    /// Convert ECEF Cartesian to Geodetic coordinates
    static GeodeticCoord ecefToGeodetic(const Vec3& ecef)
    {
        GeodeticCoord geo;

        const double a = constants::EARTH_RADIUS_EQ;
        const double e2 = constants::EARTH_ECCENTRICITY_SQ;

        double x = ecef.x;
        double y = ecef.y;
        double z = ecef.z;

        // Longitude is straightforward
        geo.longitude = std::atan2(y, x);

        // Iterative solution for latitude (Bowring's method)
        double p = std::sqrt(x * x + y * y);
        double lat = std::atan2(z, p * (1.0 - e2));  // Initial guess

        for (int i = 0; i < 10; ++i)
        {
            double sin_lat = std::sin(lat);
            double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            double lat_new = std::atan2(z + e2 * N * sin_lat, p);

            if (std::abs(lat_new - lat) < 1e-12) break;
            lat = lat_new;
        }

        geo.latitude = lat;

        // Altitude
        double sin_lat = std::sin(lat);
        double cos_lat = std::cos(lat);
        double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

        if (std::abs(cos_lat) > 1e-10)
        {
            geo.altitude = p / cos_lat - N;
        }
        else
        {
            geo.altitude = std::abs(z) - N * (1.0 - e2);
        }

        return geo;
    }

    /// Convert Geodetic coordinates to ECEF Cartesian
    static Vec3 geodeticToEcef(const GeodeticCoord& geo)
    {
        const double a = constants::EARTH_RADIUS_EQ;
        const double e2 = constants::EARTH_ECCENTRICITY_SQ;

        double sin_lat = std::sin(geo.latitude);
        double cos_lat = std::cos(geo.latitude);
        double sin_lon = std::sin(geo.longitude);
        double cos_lon = std::cos(geo.longitude);

        double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

        return Vec3{
            (N + geo.altitude) * cos_lat * cos_lon,
            (N + geo.altitude) * cos_lat * sin_lon,
            (N * (1.0 - e2) + geo.altitude) * sin_lat
        };
    }

    //=========================================================================
    // LVLH/RTN frame conversions
    //=========================================================================

    /// Get rotation matrix from ECI to LVLH frame
    /// LVLH: X = radial (outward), Y = along-track, Z = cross-track
    static RotationMatrix eciToLvlhMatrix(const StateVector& state)
    {
        Vec3 r_hat = state.position.normalized();
        Vec3 h = state.position.cross(state.velocity);
        Vec3 h_hat = h.normalized();
        Vec3 t_hat = h_hat.cross(r_hat);

        RotationMatrix R;
        // Radial (R)
        R.m[0][0] = r_hat.x; R.m[0][1] = r_hat.y; R.m[0][2] = r_hat.z;
        // Along-track (T)
        R.m[1][0] = t_hat.x; R.m[1][1] = t_hat.y; R.m[1][2] = t_hat.z;
        // Cross-track (N)
        R.m[2][0] = h_hat.x; R.m[2][1] = h_hat.y; R.m[2][2] = h_hat.z;

        return R;
    }

    /// Convert ECI position to LVLH frame relative to reference satellite
    static Vec3 eciToLvlh(const Vec3& pos_eci, const StateVector& reference)
    {
        Vec3 rel_pos = pos_eci - reference.position;
        RotationMatrix R = eciToLvlhMatrix(reference);
        return R.rotate(rel_pos);
    }

    /// Convert LVLH position to ECI frame
    static Vec3 lvlhToEci(const Vec3& pos_lvlh, const StateVector& reference)
    {
        RotationMatrix R = eciToLvlhMatrix(reference).transpose();
        return reference.position + R.rotate(pos_lvlh);
    }

    /// Convert relative state from ECI to LVLH
    static StateVector relativeStateToLvlh(const StateVector& target,
                                           const StateVector& chaser)
    {
        RotationMatrix R = eciToLvlhMatrix(target);

        // Relative position in LVLH
        Vec3 rel_pos = chaser.position - target.position;
        Vec3 pos_lvlh = R.rotate(rel_pos);

        // Angular velocity of LVLH frame
        Vec3 h = target.position.cross(target.velocity);
        double r = target.position.norm();
        Vec3 omega_lvlh = h / (r * r);

        // Relative velocity in LVLH
        Vec3 rel_vel = chaser.velocity - target.velocity;
        Vec3 vel_lvlh = R.rotate(rel_vel) - omega_lvlh.cross(pos_lvlh);

        return StateVector{pos_lvlh, vel_lvlh, target.epoch};
    }

    //=========================================================================
    // Spherical <-> Cartesian conversions
    //=========================================================================

    /// Convert Cartesian to spherical coordinates
    /// Returns {r, theta (colatitude), phi (longitude)}
    static std::array<double, 3> cartesianToSpherical(const Vec3& cart)
    {
        double r = cart.norm();
        if (r < 1e-10) return {0, 0, 0};

        double theta = std::acos(cart.z / r);  // Colatitude [0, PI]
        double phi = std::atan2(cart.y, cart.x);  // Longitude [-PI, PI]

        return {r, theta, phi};
    }

    /// Convert spherical to Cartesian coordinates
    static Vec3 sphericalToCartesian(double r, double theta, double phi)
    {
        double sin_theta = std::sin(theta);
        return Vec3{
            r * sin_theta * std::cos(phi),
            r * sin_theta * std::sin(phi),
            r * std::cos(theta)
        };
    }

    //=========================================================================
    // Utility functions
    //=========================================================================

    /// Calculate elevation and azimuth of satellite from ground station
    /// Returns {elevation, azimuth} in radians
    static std::array<double, 2> calculateAzEl(const Vec3& sat_ecef,
                                                const GeodeticCoord& station)
    {
        Vec3 station_ecef = geodeticToEcef(station);
        Vec3 range = sat_ecef - station_ecef;

        double sin_lat = std::sin(station.latitude);
        double cos_lat = std::cos(station.latitude);
        double sin_lon = std::sin(station.longitude);
        double cos_lon = std::cos(station.longitude);

        // Transform to topocentric (East-North-Up) frame
        double east = -sin_lon * range.x + cos_lon * range.y;
        double north = -sin_lat * cos_lon * range.x - sin_lat * sin_lon * range.y + cos_lat * range.z;
        double up = cos_lat * cos_lon * range.x + cos_lat * sin_lon * range.y + sin_lat * range.z;

        double horizontal = std::sqrt(east * east + north * north);
        double elevation = std::atan2(up, horizontal);
        double azimuth = std::atan2(east, north);

        if (azimuth < 0) azimuth += constants::TWO_PI;

        return {elevation, azimuth};
    }

    /// Calculate range between two positions
    static double range(const Vec3& pos1, const Vec3& pos2)
    {
        return (pos2 - pos1).norm();
    }

    /// Calculate range rate (closing velocity)
    static double rangeRate(const StateVector& state1, const StateVector& state2)
    {
        Vec3 rel_pos = state2.position - state1.position;
        Vec3 rel_vel = state2.velocity - state1.velocity;
        double r = rel_pos.norm();
        if (r < 1e-10) return 0;
        return rel_pos.dot(rel_vel) / r;
    }
};

} // namespace hpop_core
