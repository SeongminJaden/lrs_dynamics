#pragma once

#include <cmath>
#include <memory>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "atmosphere/exponential.hpp"

namespace hpop_perturbations
{

/**
 * @brief Atmospheric drag acceleration model
 *
 * Computes acceleration due to atmospheric drag using the
 * cannonball model: a = -0.5 * (Cd * A / m) * rho * v_rel^2 * v_hat
 *
 * The velocity relative to the rotating atmosphere is used.
 */
class DragModel
{
public:
    enum class AtmosphereType
    {
        Exponential,
        HarrisPriester
    };

    DragModel()
        : cd_(2.2), area_(10.0), mass_(1000.0), atm_type_(AtmosphereType::Exponential)
    {
    }

    /**
     * @brief Set spacecraft parameters
     * @param cd Drag coefficient (typically 2.0-2.5)
     * @param area Cross-sectional area [m^2]
     * @param mass Spacecraft mass [kg]
     */
    void setSpacecraftParams(double cd, double area, double mass)
    {
        cd_ = cd;
        area_ = area;
        mass_ = mass;
        ballistic_coeff_ = mass / (cd * area);
    }

    /**
     * @brief Set drag coefficient
     */
    void setDragCoefficient(double cd) { cd_ = cd; }

    /**
     * @brief Set cross-sectional area [m^2]
     */
    void setCrossSection(double area) { area_ = area; }

    /**
     * @brief Set spacecraft mass [kg]
     */
    void setMass(double mass) { mass_ = mass; }

    /**
     * @brief Get ballistic coefficient [kg/m^2]
     */
    double getBallisticCoefficient() const
    {
        return mass_ / (cd_ * area_);
    }

    /**
     * @brief Set atmosphere model type
     */
    void setAtmosphereModel(AtmosphereType type)
    {
        atm_type_ = type;
    }

    /**
     * @brief Set solar activity for Harris-Priester model
     */
    void setSolarActivity(double f107, double f107a)
    {
        hp_atmosphere_.setSolarActivity(f107, f107a);
    }

    /**
     * @brief Compute atmospheric drag acceleration in ECI frame
     *
     * @param state Current state vector in ECI frame
     * @param jd Julian date (for Earth rotation)
     * @return Acceleration due to drag [m/s^2] in ECI frame
     */
    hpop_core::Vec3 acceleration(const hpop_core::StateVector& state, double jd) const
    {
        using namespace hpop_core;

        // Get position and velocity in ECI
        const Vec3& r_eci = state.position;
        const Vec3& v_eci = state.velocity;

        // Calculate altitude
        double r_mag = r_eci.norm();
        double altitude = r_mag - constants::EARTH_RADIUS_EQ;

        // Check if altitude is too high for significant drag
        if (altitude > 1500000.0)  // 1500 km
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        // Get atmospheric density
        double rho = getDensity(altitude);

        if (rho < 1e-20)
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        // Compute velocity relative to rotating atmosphere
        // v_rel = v_eci - omega_earth x r_eci
        // omega_earth = [0, 0, OMEGA_EARTH]
        Vec3 omega_earth(0.0, 0.0, constants::OMEGA_EARTH);
        Vec3 v_atm = omega_earth.cross(r_eci);  // Atmosphere velocity at satellite position
        Vec3 v_rel = v_eci - v_atm;             // Relative velocity

        double v_rel_mag = v_rel.norm();

        if (v_rel_mag < 1.0)  // Negligible velocity
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        // Drag acceleration: a = -0.5 * (Cd * A / m) * rho * v_rel^2 * v_hat
        double drag_factor = -0.5 * (cd_ * area_ / mass_) * rho * v_rel_mag;
        Vec3 a_drag = v_rel * drag_factor;

        return a_drag;
    }

    /**
     * @brief Compute drag acceleration with solar position for Harris-Priester model
     */
    hpop_core::Vec3 accelerationWithSolar(const hpop_core::StateVector& state, double jd,
                                           double sun_ra, double sun_dec) const
    {
        using namespace hpop_core;

        const Vec3& r_eci = state.position;
        const Vec3& v_eci = state.velocity;

        double r_mag = r_eci.norm();
        double altitude = r_mag - constants::EARTH_RADIUS_EQ;

        if (altitude > 1500000.0)
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        // Get atmospheric density with diurnal variation
        double rho;
        if (atm_type_ == AtmosphereType::HarrisPriester)
        {
            // Calculate satellite RA and DEC
            double sat_ra = std::atan2(r_eci.y, r_eci.x);
            double sat_dec = std::asin(r_eci.z / r_mag);
            rho = hp_atmosphere_.density(altitude, sun_ra, sun_dec, sat_ra, sat_dec);
        }
        else
        {
            rho = atmosphere::ExponentialAtmosphere::density(altitude);
        }

        if (rho < 1e-20)
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        // Velocity relative to rotating atmosphere
        Vec3 omega_earth(0.0, 0.0, constants::OMEGA_EARTH);
        Vec3 v_atm = omega_earth.cross(r_eci);
        Vec3 v_rel = v_eci - v_atm;
        double v_rel_mag = v_rel.norm();

        if (v_rel_mag < 1.0)
        {
            return Vec3(0.0, 0.0, 0.0);
        }

        double drag_factor = -0.5 * (cd_ * area_ / mass_) * rho * v_rel_mag;
        return v_rel * drag_factor;
    }

    /**
     * @brief Get atmospheric density at altitude
     */
    double getDensity(double altitude) const
    {
        if (atm_type_ == AtmosphereType::HarrisPriester)
        {
            return hp_atmosphere_.densitySimple(altitude);
        }
        return atmosphere::ExponentialAtmosphere::density(altitude);
    }

    /**
     * @brief Get scale height at altitude [m]
     */
    double getScaleHeight(double altitude) const
    {
        return atmosphere::ExponentialAtmosphere::scaleHeight(altitude);
    }

private:
    double cd_;           // Drag coefficient
    double area_;         // Cross-sectional area [m^2]
    double mass_;         // Spacecraft mass [kg]
    double ballistic_coeff_;  // Ballistic coefficient [kg/m^2]

    AtmosphereType atm_type_;
    atmosphere::HarrisPriesterAtmosphere hp_atmosphere_;
};

} // namespace hpop_perturbations
