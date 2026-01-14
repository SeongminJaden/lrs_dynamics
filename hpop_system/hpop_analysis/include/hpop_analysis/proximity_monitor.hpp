/**
 * @file proximity_monitor.hpp
 * @brief Satellite proximity and collision analysis
 *
 * Calculates closest approach between satellites for:
 * - Collision avoidance screening
 * - Rendezvous planning
 * - Formation flying analysis
 */

#ifndef HPOP_ANALYSIS_PROXIMITY_MONITOR_HPP
#define HPOP_ANALYSIS_PROXIMITY_MONITOR_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"

namespace hpop_analysis
{

/**
 * @brief Proximity event between two satellites
 */
struct ProximityEvent
{
    std::string primary_id;
    std::string secondary_id;
    double tca_jd;                     // Time of Closest Approach [JD]
    double miss_distance;              // [m]
    double relative_velocity;          // [m/s]

    // Relative state in LVLH frame (centered on primary)
    hpop_core::Vec3 relative_position; // [m] R-bar, V-bar, H-bar
    hpop_core::Vec3 relative_velocity_lvlh; // [m/s]

    // Risk assessment
    double collision_probability;      // Pc (if covariance available)
    bool requires_maneuver;            // Based on screening criteria

    // Screening criteria
    static constexpr double COLLISION_THRESHOLD = 1000.0;      // 1 km
    static constexpr double WARNING_THRESHOLD = 5000.0;        // 5 km
    static constexpr double SCREENING_THRESHOLD = 25000.0;     // 25 km (typical LEO)
};

/**
 * @brief Conjunction screening result
 */
struct ConjunctionSummary
{
    int total_conjunctions;
    int high_risk;             // < 1 km
    int medium_risk;           // 1-5 km
    int low_risk;              // 5-25 km
    double minimum_distance;   // [m]
    double tca_of_minimum;     // [JD]

    std::string getRiskLevel() const
    {
        if (high_risk > 0) return "HIGH";
        if (medium_risk > 0) return "MEDIUM";
        if (low_risk > 0) return "LOW";
        return "NOMINAL";
    }
};

/**
 * @brief Proximity monitor class
 */
class ProximityMonitor
{
public:
    ProximityMonitor() = default;

    /**
     * @brief Set screening threshold distance
     */
    void setScreeningThreshold(double threshold_m)
    {
        screening_threshold_ = threshold_m;
    }

    /**
     * @brief Calculate relative state in LVLH frame
     *
     * LVLH (Local Vertical Local Horizontal) frame:
     * - R-bar: Radial (from Earth center through primary)
     * - V-bar: Along-track (velocity direction)
     * - H-bar: Cross-track (angular momentum direction)
     *
     * @param primary Primary satellite state (target)
     * @param secondary Secondary satellite state (chaser)
     * @return Relative state in LVLH frame
     */
    static std::pair<hpop_core::Vec3, hpop_core::Vec3> getRelativeStateLVLH(
        const hpop_core::StateVector& primary,
        const hpop_core::StateVector& secondary)
    {
        using namespace hpop_core;

        // LVLH frame unit vectors
        Vec3 r_hat = primary.position.normalized();  // R-bar (radial outward)
        Vec3 h = primary.position.cross(primary.velocity);
        Vec3 h_hat = h.normalized();                  // H-bar (cross-track)
        Vec3 v_hat = h_hat.cross(r_hat);             // V-bar (along-track)

        // Relative position and velocity in ECI
        Vec3 dr = secondary.position - primary.position;
        Vec3 dv = secondary.velocity - primary.velocity;

        // Rotation to LVLH
        Vec3 rel_pos_lvlh(
            dr.dot(r_hat),
            dr.dot(v_hat),
            dr.dot(h_hat)
        );

        // Account for rotating frame
        double r = primary.position.norm();
        double v = primary.velocity.norm();
        double omega = v / r;  // Angular velocity of LVLH frame

        Vec3 rel_vel_lvlh(
            dv.dot(r_hat) - omega * rel_pos_lvlh.y,
            dv.dot(v_hat) + omega * rel_pos_lvlh.x,
            dv.dot(h_hat)
        );

        return {rel_pos_lvlh, rel_vel_lvlh};
    }

    /**
     * @brief Calculate distance between two satellites
     */
    static double calculateDistance(const hpop_core::StateVector& sat1,
                                    const hpop_core::StateVector& sat2)
    {
        return (sat1.position - sat2.position).norm();
    }

    /**
     * @brief Calculate relative velocity magnitude
     */
    static double calculateRelativeVelocity(const hpop_core::StateVector& sat1,
                                            const hpop_core::StateVector& sat2)
    {
        return (sat1.velocity - sat2.velocity).norm();
    }

    /**
     * @brief Find closest approach in two trajectories
     *
     * @param primary_id Primary satellite ID
     * @param primary_traj Primary satellite trajectory [(JD, StateVector), ...]
     * @param secondary_id Secondary satellite ID
     * @param secondary_traj Secondary satellite trajectory
     * @return Proximity event at closest approach
     */
    ProximityEvent findClosestApproach(
        const std::string& primary_id,
        const std::vector<std::pair<double, hpop_core::StateVector>>& primary_traj,
        const std::string& secondary_id,
        const std::vector<std::pair<double, hpop_core::StateVector>>& secondary_traj) const
    {
        ProximityEvent result;
        result.primary_id = primary_id;
        result.secondary_id = secondary_id;
        result.miss_distance = std::numeric_limits<double>::max();

        // Simple approach: iterate through synchronized trajectories
        // Assumes both trajectories have same time steps
        size_t n = std::min(primary_traj.size(), secondary_traj.size());

        for (size_t i = 0; i < n; ++i)
        {
            const auto& [jd1, state1] = primary_traj[i];
            const auto& [jd2, state2] = secondary_traj[i];

            // Interpolate if times don't match exactly
            double jd = jd1;

            double dist = calculateDistance(state1, state2);

            if (dist < result.miss_distance)
            {
                result.miss_distance = dist;
                result.tca_jd = jd;
                result.relative_velocity = calculateRelativeVelocity(state1, state2);

                auto [rel_pos, rel_vel] = getRelativeStateLVLH(state1, state2);
                result.relative_position = rel_pos;
                result.relative_velocity_lvlh = rel_vel;
            }
        }

        // Set risk assessment
        result.requires_maneuver = result.miss_distance < ProximityEvent::COLLISION_THRESHOLD;
        result.collision_probability = estimateCollisionProbability(result);

        return result;
    }

    /**
     * @brief Screen for close approaches over time period
     */
    std::vector<ProximityEvent> screenConjunctions(
        const std::string& primary_id,
        const std::vector<std::pair<double, hpop_core::StateVector>>& primary_traj,
        const std::string& secondary_id,
        const std::vector<std::pair<double, hpop_core::StateVector>>& secondary_traj) const
    {
        std::vector<ProximityEvent> conjunctions;

        if (primary_traj.empty() || secondary_traj.empty())
            return conjunctions;

        size_t n = std::min(primary_traj.size(), secondary_traj.size());

        bool in_conjunction = false;
        double min_dist = std::numeric_limits<double>::max();
        size_t tca_idx = 0;

        for (size_t i = 0; i < n; ++i)
        {
            double dist = calculateDistance(primary_traj[i].second, secondary_traj[i].second);

            if (dist < screening_threshold_)
            {
                if (!in_conjunction)
                {
                    in_conjunction = true;
                    min_dist = dist;
                    tca_idx = i;
                }
                else if (dist < min_dist)
                {
                    min_dist = dist;
                    tca_idx = i;
                }
            }
            else if (in_conjunction)
            {
                // End of conjunction - record event
                in_conjunction = false;

                ProximityEvent event;
                event.primary_id = primary_id;
                event.secondary_id = secondary_id;
                event.tca_jd = primary_traj[tca_idx].first;
                event.miss_distance = min_dist;
                event.relative_velocity = calculateRelativeVelocity(
                    primary_traj[tca_idx].second, secondary_traj[tca_idx].second);

                auto [rel_pos, rel_vel] = getRelativeStateLVLH(
                    primary_traj[tca_idx].second, secondary_traj[tca_idx].second);
                event.relative_position = rel_pos;
                event.relative_velocity_lvlh = rel_vel;
                event.requires_maneuver = min_dist < ProximityEvent::COLLISION_THRESHOLD;
                event.collision_probability = estimateCollisionProbability(event);

                conjunctions.push_back(event);
                min_dist = std::numeric_limits<double>::max();
            }
        }

        // Handle conjunction at end of trajectory
        if (in_conjunction)
        {
            ProximityEvent event;
            event.primary_id = primary_id;
            event.secondary_id = secondary_id;
            event.tca_jd = primary_traj[tca_idx].first;
            event.miss_distance = min_dist;
            event.relative_velocity = calculateRelativeVelocity(
                primary_traj[tca_idx].second, secondary_traj[tca_idx].second);

            auto [rel_pos, rel_vel] = getRelativeStateLVLH(
                primary_traj[tca_idx].second, secondary_traj[tca_idx].second);
            event.relative_position = rel_pos;
            event.relative_velocity_lvlh = rel_vel;
            event.requires_maneuver = min_dist < ProximityEvent::COLLISION_THRESHOLD;
            event.collision_probability = estimateCollisionProbability(event);

            conjunctions.push_back(event);
        }

        return conjunctions;
    }

    /**
     * @brief Generate conjunction summary
     */
    ConjunctionSummary generateSummary(const std::vector<ProximityEvent>& conjunctions) const
    {
        ConjunctionSummary summary{};
        summary.total_conjunctions = static_cast<int>(conjunctions.size());
        summary.minimum_distance = std::numeric_limits<double>::max();

        for (const auto& event : conjunctions)
        {
            if (event.miss_distance < ProximityEvent::COLLISION_THRESHOLD)
                summary.high_risk++;
            else if (event.miss_distance < ProximityEvent::WARNING_THRESHOLD)
                summary.medium_risk++;
            else
                summary.low_risk++;

            if (event.miss_distance < summary.minimum_distance)
            {
                summary.minimum_distance = event.miss_distance;
                summary.tca_of_minimum = event.tca_jd;
            }
        }

        return summary;
    }

    /**
     * @brief Calculate Clohessy-Wiltshire (CW) relative motion prediction
     *
     * Used for short-term proximity prediction in circular reference orbit.
     * The Hill-Clohessy-Wiltshire equations for relative motion.
     *
     * @param rel_pos Initial relative position [m] in LVLH
     * @param rel_vel Initial relative velocity [m/s] in LVLH
     * @param n Mean motion of reference orbit [rad/s]
     * @param dt Prediction time [s]
     * @return Predicted relative state (position, velocity)
     */
    static std::pair<hpop_core::Vec3, hpop_core::Vec3> predictRelativeMotionCW(
        const hpop_core::Vec3& rel_pos,
        const hpop_core::Vec3& rel_vel,
        double n,
        double dt)
    {
        using namespace hpop_core;

        double nt = n * dt;
        double snt = std::sin(nt);
        double cnt = std::cos(nt);

        // Initial state components
        double x0 = rel_pos.x;  // R-bar
        double y0 = rel_pos.y;  // V-bar
        double z0 = rel_pos.z;  // H-bar
        double vx0 = rel_vel.x;
        double vy0 = rel_vel.y;
        double vz0 = rel_vel.z;

        // CW state transition matrix solution
        Vec3 pos_new;
        pos_new.x = (4.0 - 3.0 * cnt) * x0 + snt / n * vx0 + 2.0 / n * (1.0 - cnt) * vy0;
        pos_new.y = 6.0 * (snt - nt) * x0 + y0 - 2.0 / n * (1.0 - cnt) * vx0 + (4.0 * snt - 3.0 * nt) / n * vy0;
        pos_new.z = cnt * z0 + snt / n * vz0;

        Vec3 vel_new;
        vel_new.x = 3.0 * n * snt * x0 + cnt * vx0 + 2.0 * snt * vy0;
        vel_new.y = 6.0 * n * (cnt - 1.0) * x0 - 2.0 * snt * vx0 + (4.0 * cnt - 3.0) * vy0;
        vel_new.z = -n * snt * z0 + cnt * vz0;

        return {pos_new, vel_new};
    }

private:
    double screening_threshold_ = ProximityEvent::SCREENING_THRESHOLD;

    /**
     * @brief Simple collision probability estimation (Foster method simplified)
     */
    double estimateCollisionProbability(const ProximityEvent& event) const
    {
        // Very simplified Pc estimation
        // Assumes combined hard body radius of 10 m
        constexpr double combined_radius = 10.0;  // m
        constexpr double position_uncertainty = 100.0;  // m (assumed 1-sigma)

        if (event.miss_distance > 10.0 * position_uncertainty)
            return 0.0;

        // Gaussian approximation
        double sigma2 = 2.0 * position_uncertainty * position_uncertainty;
        double d2 = event.miss_distance * event.miss_distance;
        double r2 = combined_radius * combined_radius;

        double pc = r2 / sigma2 * std::exp(-d2 / sigma2);

        return std::min(pc, 1.0);
    }
};

/**
 * @brief Rendezvous trajectory planner using CW equations
 */
class RendezvousPlanner
{
public:
    /**
     * @brief Calculate V-bar hop delta-v
     *
     * Simple V-bar approach maneuver.
     *
     * @param distance Approach distance [m]
     * @param n Mean motion [rad/s]
     * @return Required delta-v [m/s]
     */
    static double calculateVbarHop(double distance, double n)
    {
        // Half period V-bar hop
        double T = hpop_core::constants::PI / n;
        return 2.0 * distance / T;
    }

    /**
     * @brief Calculate radial impulse for R-bar station keeping
     */
    static double calculateRbarCorrection(double r_bar_error, double n)
    {
        return n * r_bar_error / 2.0;
    }

    /**
     * @brief Calculate delta-v for phase adjustment
     *
     * Adjust along-track position using altitude change.
     *
     * @param phase_error Phase error [rad]
     * @param sma Semi-major axis [m]
     * @param time Available time [s]
     * @return Required delta-v [m/s]
     */
    static double calculatePhaseCorrection(double phase_error, double sma, double time)
    {
        using namespace hpop_core;

        double n = std::sqrt(constants::EARTH_MU / (sma * sma * sma));
        double v = n * sma;

        // Linear approximation for small phase changes
        // dn/da = -3/2 * n/a
        // d_phase = dn * time
        double da = -2.0 * sma * phase_error / (3.0 * n * time);

        // Hohmann-like delta-v
        double dv = std::abs(v * da / (2.0 * sma));

        return dv;
    }
};

} // namespace hpop_analysis

#endif // HPOP_ANALYSIS_PROXIMITY_MONITOR_HPP
