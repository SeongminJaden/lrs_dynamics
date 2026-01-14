#pragma once

#include <array>
#include <cmath>
#include <string>
#include "hpop_core/constants.hpp"

namespace hpop_core
{

/// 3D Vector for positions, velocities, accelerations
struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Vector operations
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const
    {
        return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
    }

    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    double normSq() const { return x * x + y * y + z * z; }

    Vec3 normalized() const
    {
        double n = norm();
        return (n > 0) ? (*this / n) : Vec3{};
    }

    std::array<double, 3> toArray() const { return {x, y, z}; }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

/// State vector containing position, velocity, and optional attitude
struct StateVector
{
    Vec3 position;      // [m] Position in reference frame
    Vec3 velocity;      // [m/s] Velocity in reference frame
    double epoch{0.0};  // Julian date of epoch

    // Optional 6DOF attitude (quaternion)
    std::array<double, 4> quaternion{0.0, 0.0, 0.0, 1.0};  // [x, y, z, w]
    Vec3 angular_velocity;  // [rad/s] Angular velocity in body frame

    StateVector() = default;
    StateVector(const Vec3& pos, const Vec3& vel, double jd = 0.0)
        : position(pos), velocity(vel), epoch(jd) {}

    /// Get state as 6-element array [x, y, z, vx, vy, vz]
    std::array<double, 6> toArray() const
    {
        return {position.x, position.y, position.z,
                velocity.x, velocity.y, velocity.z};
    }

    /// Set state from 6-element array
    void fromArray(const std::array<double, 6>& arr)
    {
        position = {arr[0], arr[1], arr[2]};
        velocity = {arr[3], arr[4], arr[5]};
    }

    /// Get orbital radius
    double radius() const { return position.norm(); }

    /// Get orbital speed
    double speed() const { return velocity.norm(); }

    /// Get specific angular momentum vector h = r x v
    Vec3 angularMomentum() const { return position.cross(velocity); }

    /// Get specific orbital energy
    double orbitalEnergy(double mu = constants::EARTH_MU) const
    {
        return 0.5 * velocity.normSq() - mu / position.norm();
    }
};

/// Orbital elements (Keplerian)
struct OrbitalElements
{
    double sma{0.0};        // [m] Semi-major axis (a)
    double ecc{0.0};        // [-] Eccentricity (e)
    double inc{0.0};        // [rad] Inclination (i)
    double raan{0.0};       // [rad] Right Ascension of Ascending Node (Omega)
    double aop{0.0};        // [rad] Argument of Periapsis (omega)
    double ta{0.0};         // [rad] True Anomaly (nu)
    double epoch{0.0};      // Julian date of epoch

    // Derived quantities
    double meanAnomaly() const;
    double eccentricAnomaly() const;
    double period(double mu = constants::EARTH_MU) const;
    double meanMotion(double mu = constants::EARTH_MU) const;
    double periapsis() const { return sma * (1.0 - ecc); }
    double apoapsis() const { return sma * (1.0 + ecc); }

    // Conversion to/from state vector
    StateVector toStateVector(double mu = constants::EARTH_MU) const;
    static OrbitalElements fromStateVector(const StateVector& state,
                                           double mu = constants::EARTH_MU);
};

// Implementation of derived quantities
inline double OrbitalElements::meanMotion(double mu) const
{
    return std::sqrt(mu / (sma * sma * sma));
}

inline double OrbitalElements::period(double mu) const
{
    return constants::TWO_PI / meanMotion(mu);
}

inline double OrbitalElements::eccentricAnomaly() const
{
    double cosE = (ecc + std::cos(ta)) / (1.0 + ecc * std::cos(ta));
    double sinE = std::sqrt(1.0 - ecc * ecc) * std::sin(ta) / (1.0 + ecc * std::cos(ta));
    return std::atan2(sinE, cosE);
}

inline double OrbitalElements::meanAnomaly() const
{
    double E = eccentricAnomaly();
    return E - ecc * std::sin(E);
}

inline StateVector OrbitalElements::toStateVector(double mu) const
{
    // Semi-latus rectum
    double p = sma * (1.0 - ecc * ecc);

    // Position magnitude
    double r = p / (1.0 + ecc * std::cos(ta));

    // Position in perifocal frame (PQW)
    double cos_ta = std::cos(ta);
    double sin_ta = std::sin(ta);
    Vec3 r_pqw{r * cos_ta, r * sin_ta, 0.0};

    // Velocity in perifocal frame
    double sqrt_mu_p = std::sqrt(mu / p);
    Vec3 v_pqw{-sqrt_mu_p * sin_ta, sqrt_mu_p * (ecc + cos_ta), 0.0};

    // Rotation matrix from PQW to ECI
    double cos_raan = std::cos(raan);
    double sin_raan = std::sin(raan);
    double cos_aop = std::cos(aop);
    double sin_aop = std::sin(aop);
    double cos_inc = std::cos(inc);
    double sin_inc = std::sin(inc);

    // Rotation matrix elements
    double R11 = cos_raan * cos_aop - sin_raan * sin_aop * cos_inc;
    double R12 = -cos_raan * sin_aop - sin_raan * cos_aop * cos_inc;
    double R21 = sin_raan * cos_aop + cos_raan * sin_aop * cos_inc;
    double R22 = -sin_raan * sin_aop + cos_raan * cos_aop * cos_inc;
    double R31 = sin_aop * sin_inc;
    double R32 = cos_aop * sin_inc;

    // Transform position
    Vec3 position{
        R11 * r_pqw.x + R12 * r_pqw.y,
        R21 * r_pqw.x + R22 * r_pqw.y,
        R31 * r_pqw.x + R32 * r_pqw.y
    };

    // Transform velocity
    Vec3 velocity{
        R11 * v_pqw.x + R12 * v_pqw.y,
        R21 * v_pqw.x + R22 * v_pqw.y,
        R31 * v_pqw.x + R32 * v_pqw.y
    };

    return StateVector{position, velocity, epoch};
}

inline OrbitalElements OrbitalElements::fromStateVector(const StateVector& state, double mu)
{
    OrbitalElements oe;
    oe.epoch = state.epoch;

    const Vec3& r = state.position;
    const Vec3& v = state.velocity;

    double r_mag = r.norm();
    double v_mag = v.norm();

    // Angular momentum vector
    Vec3 h = r.cross(v);
    double h_mag = h.norm();

    // Node vector (K x h)
    Vec3 n{-h.y, h.x, 0.0};
    double n_mag = n.norm();

    // Eccentricity vector
    Vec3 e_vec = ((v_mag * v_mag - mu / r_mag) * r - r.dot(v) * v) / mu;
    oe.ecc = e_vec.norm();

    // Semi-major axis
    double energy = 0.5 * v_mag * v_mag - mu / r_mag;
    if (std::abs(oe.ecc - 1.0) > 1e-10)
    {
        oe.sma = -mu / (2.0 * energy);
    }
    else
    {
        oe.sma = std::numeric_limits<double>::infinity();  // Parabolic
    }

    // Inclination
    oe.inc = std::acos(h.z / h_mag);

    // Right ascension of ascending node
    if (n_mag > 1e-10)
    {
        oe.raan = std::acos(n.x / n_mag);
        if (n.y < 0) oe.raan = constants::TWO_PI - oe.raan;
    }
    else
    {
        oe.raan = 0.0;  // Equatorial orbit
    }

    // Argument of periapsis
    if (n_mag > 1e-10 && oe.ecc > 1e-10)
    {
        oe.aop = std::acos(n.dot(e_vec) / (n_mag * oe.ecc));
        if (e_vec.z < 0) oe.aop = constants::TWO_PI - oe.aop;
    }
    else
    {
        oe.aop = 0.0;  // Circular or equatorial
    }

    // True anomaly
    if (oe.ecc > 1e-10)
    {
        oe.ta = std::acos(e_vec.dot(r) / (oe.ecc * r_mag));
        if (r.dot(v) < 0) oe.ta = constants::TWO_PI - oe.ta;
    }
    else
    {
        // Circular orbit: use argument of latitude
        if (n_mag > 1e-10)
        {
            oe.ta = std::acos(n.dot(r) / (n_mag * r_mag));
            if (r.z < 0) oe.ta = constants::TWO_PI - oe.ta;
        }
        else
        {
            oe.ta = std::acos(r.x / r_mag);
            if (v.x > 0) oe.ta = constants::TWO_PI - oe.ta;
        }
    }

    return oe;
}

} // namespace hpop_core
