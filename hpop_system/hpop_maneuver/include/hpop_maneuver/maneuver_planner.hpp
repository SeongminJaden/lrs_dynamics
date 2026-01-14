/**
 * @file maneuver_planner.hpp
 * @brief Orbital maneuver planning utilities
 *
 * Includes:
 * - Hohmann transfer calculations
 * - Bi-elliptic transfer
 * - Plane change maneuvers
 * - Station keeping
 */

#ifndef HPOP_MANEUVER_MANEUVER_PLANNER_HPP
#define HPOP_MANEUVER_MANEUVER_PLANNER_HPP

#include <vector>
#include <cmath>
#include <algorithm>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/time_system.hpp"

namespace hpop_maneuver
{

/**
 * @brief Delta-V maneuver definition
 */
struct DeltaV
{
    double epoch;          // Maneuver time [JD]
    hpop_core::Vec3 dv;    // Delta-V vector in ECI [m/s]
    double magnitude;      // |dv| [m/s]
    std::string type;      // Description (e.g., "Hohmann burn 1")

    DeltaV() = default;

    DeltaV(double t, const hpop_core::Vec3& v, const std::string& desc = "")
        : epoch(t), dv(v), magnitude(v.norm()), type(desc) {}
};

/**
 * @brief Maneuver sequence result
 */
struct ManeuverPlan
{
    std::vector<DeltaV> burns;          // All required burns
    double total_delta_v;               // Total delta-v [m/s]
    double transfer_time;               // Transfer duration [s]
    hpop_core::OrbitalElements initial_orbit;
    hpop_core::OrbitalElements final_orbit;
    bool is_feasible;
    std::string description;

    ManeuverPlan() : total_delta_v(0), transfer_time(0), is_feasible(true) {}

    void addBurn(const DeltaV& burn)
    {
        burns.push_back(burn);
        total_delta_v += burn.magnitude;
    }
};

/**
 * @brief Hohmann transfer calculator
 */
class HohmannTransfer
{
public:
    /**
     * @brief Calculate Hohmann transfer between circular orbits
     *
     * @param r1 Initial orbit radius [m]
     * @param r2 Final orbit radius [m]
     * @param mu Gravitational parameter [m^3/s^2]
     * @return ManeuverPlan with two burns
     */
    static ManeuverPlan calculate(double r1, double r2,
                                  double mu = hpop_core::constants::EARTH_MU)
    {
        ManeuverPlan plan;

        // Transfer orbit semi-major axis
        double a_trans = (r1 + r2) / 2.0;

        // Velocities in circular orbits
        double v1 = std::sqrt(mu / r1);  // Initial orbit velocity
        double v2 = std::sqrt(mu / r2);  // Final orbit velocity

        // Velocities at apsides of transfer orbit
        double v_trans_peri = std::sqrt(mu * (2.0 / r1 - 1.0 / a_trans));
        double v_trans_apo = std::sqrt(mu * (2.0 / r2 - 1.0 / a_trans));

        // Delta-V calculations
        double dv1, dv2;

        if (r2 > r1)
        {
            // Transfer to higher orbit
            dv1 = v_trans_peri - v1;   // Prograde at periapsis
            dv2 = v2 - v_trans_apo;    // Prograde at apoapsis
            plan.description = "Hohmann transfer (raise orbit)";
        }
        else
        {
            // Transfer to lower orbit
            dv1 = v1 - v_trans_peri;   // Retrograde at periapsis
            dv2 = v_trans_apo - v2;    // Retrograde at apoapsis
            plan.description = "Hohmann transfer (lower orbit)";
        }

        // Transfer time (half period of transfer ellipse)
        double T_trans = hpop_core::constants::PI * std::sqrt(a_trans * a_trans * a_trans / mu);

        // Create burn vectors (tangential)
        hpop_core::Vec3 dv1_vec(0, dv1, 0);  // Along-track (simplified)
        hpop_core::Vec3 dv2_vec(0, dv2, 0);

        plan.addBurn(DeltaV(0, dv1_vec, "Hohmann burn 1 (departure)"));
        plan.addBurn(DeltaV(T_trans / 86400.0, dv2_vec, "Hohmann burn 2 (arrival)"));

        plan.transfer_time = T_trans;
        plan.is_feasible = true;

        return plan;
    }

    /**
     * @brief Calculate Hohmann transfer between elliptical orbits
     *
     * @param oe1 Initial orbital elements
     * @param r_target Target apoapsis or periapsis [m]
     * @param raise_apo true to raise apoapsis, false to change periapsis
     * @return ManeuverPlan
     */
    static ManeuverPlan calculateElliptical(const hpop_core::OrbitalElements& oe1,
                                            double r_target,
                                            bool raise_apo = true)
    {
        using namespace hpop_core;
        ManeuverPlan plan;

        double a = oe1.sma;
        double e = oe1.ecc;
        double rp = a * (1.0 - e);  // Periapsis
        double ra = a * (1.0 + e);  // Apoapsis

        if (raise_apo)
        {
            // Change apoapsis: burn at periapsis
            double v_peri = std::sqrt(constants::EARTH_MU * (2.0 / rp - 1.0 / a));
            double a_new = (rp + r_target) / 2.0;
            double v_peri_new = std::sqrt(constants::EARTH_MU * (2.0 / rp - 1.0 / a_new));
            double dv = v_peri_new - v_peri;

            Vec3 dv_vec(0, dv, 0);
            plan.addBurn(DeltaV(0, dv_vec, "Apoapsis raise burn (at periapsis)"));
            plan.description = "Raise apoapsis to " + std::to_string(r_target / 1000.0) + " km";
        }
        else
        {
            // Change periapsis: burn at apoapsis
            double v_apo = std::sqrt(constants::EARTH_MU * (2.0 / ra - 1.0 / a));
            double a_new = (r_target + ra) / 2.0;
            double v_apo_new = std::sqrt(constants::EARTH_MU * (2.0 / ra - 1.0 / a_new));
            double dv = v_apo_new - v_apo;

            Vec3 dv_vec(0, dv, 0);
            plan.addBurn(DeltaV(0, dv_vec, "Periapsis change burn (at apoapsis)"));
            plan.description = "Lower periapsis to " + std::to_string(r_target / 1000.0) + " km";
        }

        plan.is_feasible = true;
        return plan;
    }

    /**
     * @brief Calculate optimal transfer between two altitudes
     *
     * Automatically chooses between Hohmann and bi-elliptic based on ratio.
     * For r2/r1 > 11.94, bi-elliptic is more efficient.
     */
    static ManeuverPlan calculateOptimal(double r1, double r2,
                                         double mu = hpop_core::constants::EARTH_MU)
    {
        double ratio = std::max(r1, r2) / std::min(r1, r2);

        if (ratio < 11.94)
        {
            return calculate(r1, r2, mu);
        }
        else
        {
            return calculateBielliptic(r1, r2, mu);
        }
    }

    /**
     * @brief Calculate bi-elliptic transfer (for large orbital changes)
     */
    static ManeuverPlan calculateBielliptic(double r1, double r2,
                                            double mu = hpop_core::constants::EARTH_MU,
                                            double r_int = 0.0)
    {
        ManeuverPlan plan;

        // If no intermediate radius specified, use optimal
        if (r_int <= 0)
        {
            r_int = std::max(r1, r2) * 15.0;  // High intermediate orbit
        }

        // First transfer: r1 to r_int (Hohmann-like)
        double a1 = (r1 + r_int) / 2.0;
        double v1 = std::sqrt(mu / r1);
        double v1_trans = std::sqrt(mu * (2.0 / r1 - 1.0 / a1));
        double dv1 = std::abs(v1_trans - v1);

        // At intermediate apoapsis
        double v_int_1 = std::sqrt(mu * (2.0 / r_int - 1.0 / a1));

        // Second transfer: r_int to r2
        double a2 = (r_int + r2) / 2.0;
        double v_int_2 = std::sqrt(mu * (2.0 / r_int - 1.0 / a2));
        double dv2 = std::abs(v_int_2 - v_int_1);

        // Final circularization
        double v2_trans = std::sqrt(mu * (2.0 / r2 - 1.0 / a2));
        double v2 = std::sqrt(mu / r2);
        double dv3 = std::abs(v2 - v2_trans);

        // Transfer times
        double T1 = hpop_core::constants::PI * std::sqrt(a1 * a1 * a1 / mu);
        double T2 = hpop_core::constants::PI * std::sqrt(a2 * a2 * a2 / mu);

        hpop_core::Vec3 dv1_vec(0, dv1, 0);
        hpop_core::Vec3 dv2_vec(0, dv2, 0);
        hpop_core::Vec3 dv3_vec(0, -dv3, 0);

        plan.addBurn(DeltaV(0, dv1_vec, "Bi-elliptic burn 1 (departure)"));
        plan.addBurn(DeltaV(T1 / 86400.0, dv2_vec, "Bi-elliptic burn 2 (intermediate)"));
        plan.addBurn(DeltaV((T1 + T2) / 86400.0, dv3_vec, "Bi-elliptic burn 3 (arrival)"));

        plan.transfer_time = T1 + T2;
        plan.description = "Bi-elliptic transfer via " + std::to_string(r_int / 1000.0) + " km";
        plan.is_feasible = true;

        return plan;
    }
};

/**
 * @brief Plane change maneuver calculator
 */
class PlaneChange
{
public:
    /**
     * @brief Calculate simple plane change delta-v
     *
     * @param v Orbital velocity [m/s]
     * @param delta_i Inclination change [rad]
     * @return Delta-V magnitude [m/s]
     */
    static double calculateDeltaV(double v, double delta_i)
    {
        return 2.0 * v * std::sin(std::abs(delta_i) / 2.0);
    }

    /**
     * @brief Calculate combined plane change with altitude change
     *
     * More efficient to combine plane change with apoapsis burn.
     *
     * @param r Current radius [m]
     * @param v Current velocity [m/s]
     * @param delta_i Inclination change [rad]
     * @return Delta-V magnitude [m/s]
     */
    static double calculateCombined(double r, double v, double delta_i,
                                    double v_new = 0.0)
    {
        if (v_new <= 0) v_new = v;

        // Combined maneuver using law of cosines
        return std::sqrt(v * v + v_new * v_new -
                         2.0 * v * v_new * std::cos(delta_i));
    }

    /**
     * @brief Calculate inclination change maneuver plan
     */
    static ManeuverPlan calculate(const hpop_core::OrbitalElements& oe,
                                  double target_inc)
    {
        using namespace hpop_core;
        ManeuverPlan plan;

        double delta_i = target_inc - oe.inc;

        // Velocity at ascending/descending node
        double a = oe.sma;
        double e = oe.ecc;

        // For circular orbits, perform at node
        double v = std::sqrt(constants::EARTH_MU / a);
        double dv = calculateDeltaV(v, delta_i);

        // Out-of-plane burn
        Vec3 dv_vec(0, 0, dv * (delta_i > 0 ? 1.0 : -1.0));
        plan.addBurn(DeltaV(0, dv_vec, "Inclination change at node"));

        plan.description = "Plane change: " +
                           std::to_string(oe.inc * constants::RAD_TO_DEG) + " -> " +
                           std::to_string(target_inc * constants::RAD_TO_DEG) + " deg";
        plan.is_feasible = true;

        return plan;
    }
};

/**
 * @brief Station keeping maneuver calculator (for GEO)
 */
class StationKeeping
{
public:
    /**
     * @brief Calculate annual delta-v budget for GEO station keeping
     *
     * @param area Cross-sectional area [m^2]
     * @param mass Spacecraft mass [kg]
     * @param cr Reflectivity coefficient
     * @return Annual delta-v [m/s/year]
     */
    static double annualBudget(double area, double mass, double cr = 1.5)
    {
        // North-South station keeping (inclination)
        double dv_ns = 50.0;  // ~50 m/s/year for Sun/Moon perturbations

        // East-West station keeping (eccentricity/longitude)
        double dv_ew = 2.0;   // ~2 m/s/year

        // SRP contribution (depends on A/m ratio)
        double am_ratio = area / mass;
        double dv_srp = 0.5 * am_ratio * cr;  // Approximate

        return dv_ns + dv_ew + dv_srp;
    }

    /**
     * @brief Calculate inclination correction maneuver
     *
     * @param current_inc Current inclination [rad]
     * @param target_inc Target inclination (usually 0) [rad]
     * @return Delta-V [m/s]
     */
    static double inclinationCorrection(double current_inc, double target_inc = 0.0)
    {
        double v_geo = std::sqrt(hpop_core::constants::EARTH_MU / 42164000.0);
        return PlaneChange::calculateDeltaV(v_geo, current_inc - target_inc);
    }

    /**
     * @brief Calculate eccentricity correction
     *
     * @param current_ecc Current eccentricity
     * @param target_ecc Target eccentricity (usually ~0)
     * @return Delta-V [m/s]
     */
    static double eccentricityCorrection(double current_ecc, double target_ecc = 0.0)
    {
        double v_geo = std::sqrt(hpop_core::constants::EARTH_MU / 42164000.0);
        double de = current_ecc - target_ecc;
        return v_geo * de;  // Approximate
    }
};

/**
 * @brief Phasing maneuver for rendezvous
 */
class PhasingManeuver
{
public:
    /**
     * @brief Calculate phasing orbit to catch up or fall back
     *
     * @param a Current semi-major axis [m]
     * @param phase_angle Desired phase change [rad] (positive = catch up)
     * @param n_orbits Number of orbits for the maneuver
     * @return ManeuverPlan
     */
    static ManeuverPlan calculate(double a, double phase_angle, int n_orbits = 1)
    {
        using namespace hpop_core;
        ManeuverPlan plan;

        double T = 2.0 * constants::PI * std::sqrt(a * a * a / constants::EARTH_MU);
        double n = 2.0 * constants::PI / T;  // Mean motion

        // Time to achieve phase change
        double dt = phase_angle / n;

        // Required period for phasing orbit
        double T_phase = (n_orbits * T - dt) / n_orbits;

        // Semi-major axis of phasing orbit
        double a_phase = std::pow(constants::EARTH_MU * T_phase * T_phase /
                                  (4.0 * constants::PI * constants::PI), 1.0 / 3.0);

        // Delta-V to enter phasing orbit
        double v = std::sqrt(constants::EARTH_MU / a);
        double r_peri = (phase_angle > 0) ? a : 2.0 * a_phase - a;  // Lower if catching up

        double v_phase_peri = std::sqrt(constants::EARTH_MU * (2.0 / a - 1.0 / a_phase));
        double dv1 = std::abs(v_phase_peri - v);

        Vec3 dv1_vec(0, (phase_angle > 0 ? -dv1 : dv1), 0);
        Vec3 dv2_vec(0, (phase_angle > 0 ? dv1 : -dv1), 0);

        plan.addBurn(DeltaV(0, dv1_vec, "Phasing burn 1 (enter phasing orbit)"));
        plan.addBurn(DeltaV(n_orbits * T_phase / 86400.0, dv2_vec, "Phasing burn 2 (return to original orbit)"));

        plan.transfer_time = n_orbits * T_phase;
        plan.description = "Phasing maneuver: " +
                           std::to_string(phase_angle * constants::RAD_TO_DEG) + " deg in " +
                           std::to_string(n_orbits) + " orbits";
        plan.is_feasible = true;

        return plan;
    }
};

} // namespace hpop_maneuver

#endif // HPOP_MANEUVER_MANEUVER_PLANNER_HPP
