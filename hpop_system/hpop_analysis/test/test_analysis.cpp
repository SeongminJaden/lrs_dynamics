/**
 * @file test_analysis.cpp
 * @brief Test contact prediction and proximity analysis
 *
 * Results are saved for paper publication.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cmath>
#include <vector>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_core/propagator.hpp"
#include "hpop_analysis/contact_predictor.hpp"
#include "hpop_analysis/proximity_monitor.hpp"

using namespace hpop_core;
using namespace hpop_analysis;

// J2 perturbation force model
Vec3 j2Perturbation(const Vec3& pos, const Vec3& /*vel*/, double /*jd*/)
{
    double r = pos.norm();
    double r2 = r * r;
    double r5 = r2 * r2 * r;
    double re2 = constants::EARTH_RADIUS_EQ * constants::EARTH_RADIUS_EQ;
    double z2 = pos.z * pos.z;

    double factor = -1.5 * constants::EARTH_J2 * constants::EARTH_MU * re2 / r5;

    Vec3 a;
    a.x = factor * pos.x * (1.0 - 5.0 * z2 / r2);
    a.y = factor * pos.y * (1.0 - 5.0 * z2 / r2);
    a.z = factor * pos.z * (3.0 - 5.0 * z2 / r2);

    return a;
}

/**
 * @brief Generate satellite trajectory
 */
std::vector<std::pair<double, StateVector>> propagateOrbit(
    const OrbitalElements& oe,
    double duration_h,
    double step_s = 60.0)
{
    std::vector<std::pair<double, StateVector>> trajectory;

    OrbitPropagator propagator;
    propagator.setIntegrator(IntegratorType::RK4);
    propagator.setStepSize(step_s);
    propagator.addForceModel("J2", j2Perturbation);

    StateVector state = oe.toStateVector();
    double jd = oe.epoch;
    double duration_s = duration_h * 3600.0;

    for (double t = 0; t <= duration_s; t += step_s)
    {
        trajectory.emplace_back(jd + t / 86400.0, state);

        if (t < duration_s)
        {
            PropagationStep result = propagator.singleStep(state, step_s);
            state = result.state;
        }
    }

    return trajectory;
}

void testContactPrediction()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Ground Station Contact Prediction Test\n";
    std::cout << "============================================================\n\n";

    // ISS-like orbit
    OrbitalElements oe;
    oe.sma = 6778000.0;           // ~400 km altitude
    oe.ecc = 0.0001;
    oe.inc = 51.64 * constants::DEG_TO_RAD;
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::calendarToJD(2024, 6, 21, 0, 0, 0);

    std::cout << "Satellite: ISS-like orbit\n";
    std::cout << "  Altitude: " << (oe.sma - constants::EARTH_RADIUS_EQ) / 1000.0 << " km\n";
    std::cout << "  Inclination: " << oe.inc * constants::RAD_TO_DEG << " deg\n";

    // Setup ground stations
    ContactPredictor predictor;
    predictor.addGroundStation(ground_stations::DAEJEON());
    predictor.addGroundStation(ground_stations::SVALBARD());
    predictor.addGroundStation(ground_stations::CANBERRA());
    predictor.addGroundStation(ground_stations::GOLDSTONE());

    std::cout << "\nGround Stations:\n";
    for (const auto& gs : predictor.getGroundStations())
    {
        std::cout << "  " << gs.id << " (" << gs.name << "): "
                  << gs.latitude * constants::RAD_TO_DEG << "N, "
                  << gs.longitude * constants::RAD_TO_DEG << "E\n";
    }

    // Generate trajectory (24 hours, 1 minute steps)
    std::cout << "\nGenerating 24-hour trajectory...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    auto trajectory = propagateOrbit(oe, 24.0, 60.0);

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::cout << "Trajectory generated in " << std::fixed << std::setprecision(1)
              << elapsed_ms << " ms (" << trajectory.size() << " points)\n";

    // Predict contacts
    std::cout << "\nPredicting contacts...\n";
    start_time = std::chrono::high_resolution_clock::now();

    auto contacts = predictor.predictContacts("ISS", trajectory);

    end_time = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::cout << "Contact prediction completed in " << elapsed_ms << " ms\n";

    // Print contacts
    std::cout << "\n=== Contact Windows (24 hours) ===\n";
    std::cout << std::setw(10) << "Station" << " | "
              << std::setw(12) << "AOS (UTC)" << " | "
              << std::setw(12) << "LOS (UTC)" << " | "
              << std::setw(8) << "Duration" << " | "
              << std::setw(6) << "MaxEl" << "\n";
    std::cout << std::string(60, '-') << "\n";

    for (const auto& contact : contacts)
    {
        int aos_h, aos_m, los_h, los_m;
        double aos_s, los_s;
        int y, mo, d;

        // Convert JD to time
        TimeSystem::jdToCalendar(contact.aos_jd, y, mo, d, aos_h, aos_m, aos_s);
        TimeSystem::jdToCalendar(contact.los_jd, y, mo, d, los_h, los_m, los_s);

        std::cout << std::setw(10) << contact.station_id << " | "
                  << std::setfill('0')
                  << std::setw(2) << aos_h << ":" << std::setw(2) << aos_m << ":"
                  << std::setw(2) << static_cast<int>(aos_s) << " | "
                  << std::setw(2) << los_h << ":" << std::setw(2) << los_m << ":"
                  << std::setw(2) << static_cast<int>(los_s) << " | "
                  << std::setfill(' ')
                  << std::setw(5) << std::setprecision(1) << contact.getDurationMinutes() << " min | "
                  << std::setw(4) << std::setprecision(1)
                  << contact.max_elevation * constants::RAD_TO_DEG << " deg\n";
    }

    std::cout << "\nTotal contacts: " << contacts.size() << "\n";

    // Save contact data to CSV
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/contact_windows.csv");
    if (csv.is_open())
    {
        csv << "station_id,aos_jd,los_jd,duration_min,max_elevation_deg,aos_azimuth_deg,los_azimuth_deg\n";
        for (const auto& c : contacts)
        {
            csv << c.station_id << ","
                << std::fixed << std::setprecision(8) << c.aos_jd << ","
                << c.los_jd << ","
                << std::setprecision(2) << c.getDurationMinutes() << ","
                << c.max_elevation * constants::RAD_TO_DEG << ","
                << c.aos_azimuth * constants::RAD_TO_DEG << ","
                << c.los_azimuth * constants::RAD_TO_DEG << "\n";
        }
        csv.close();
        std::cout << "\nContact data saved to results/data/contact_windows.csv\n";
    }

    // Calculate and save ground track
    auto ground_track = ContactPredictor::calculateGroundTrack(trajectory);

    std::ofstream gt_csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/ground_track.csv");
    if (gt_csv.is_open())
    {
        gt_csv << "jd,latitude_deg,longitude_deg\n";
        for (const auto& [jd, lat, lon] : ground_track)
        {
            gt_csv << std::fixed << std::setprecision(8) << jd << ","
                   << std::setprecision(4) << lat << "," << lon << "\n";
        }
        gt_csv.close();
        std::cout << "Ground track saved to results/data/ground_track.csv\n";
    }
}

void testProximityAnalysis()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Satellite Proximity Analysis Test\n";
    std::cout << "============================================================\n\n";

    // Primary satellite (ISS-like)
    OrbitalElements oe_primary;
    oe_primary.sma = 6778000.0;
    oe_primary.ecc = 0.0001;
    oe_primary.inc = 51.64 * constants::DEG_TO_RAD;
    oe_primary.raan = 0.0;
    oe_primary.aop = 0.0;
    oe_primary.ta = 0.0;
    oe_primary.epoch = TimeSystem::calendarToJD(2024, 6, 21, 0, 0, 0);

    // Secondary satellite (slightly different orbit - simulating close approach)
    OrbitalElements oe_secondary;
    oe_secondary.sma = 6778000.0 + 500.0;     // 500 m higher
    oe_secondary.ecc = 0.0002;
    oe_secondary.inc = 51.64 * constants::DEG_TO_RAD + 0.001;  // Slightly different inclination
    oe_secondary.raan = 0.001;                 // Slightly different RAAN
    oe_secondary.aop = 0.0;
    oe_secondary.ta = 0.01;                    // Slightly behind in orbit
    oe_secondary.epoch = oe_primary.epoch;

    std::cout << "Primary Satellite: ISS-like orbit\n";
    std::cout << "Secondary Satellite: 500m higher, slightly different orientation\n";

    // Generate trajectories (1 orbit, 10 second steps for precision)
    double period_h = 2.0 * constants::PI *
                      std::sqrt(std::pow(oe_primary.sma, 3) / constants::EARTH_MU) / 3600.0;
    std::cout << "Orbital period: " << std::setprecision(2) << period_h * 60.0 << " minutes\n";

    std::cout << "\nGenerating trajectories (1 orbit, 10s steps)...\n";
    auto start_time = std::chrono::high_resolution_clock::now();

    auto traj_primary = propagateOrbit(oe_primary, period_h, 10.0);
    auto traj_secondary = propagateOrbit(oe_secondary, period_h, 10.0);

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    std::cout << "Trajectories generated in " << std::fixed << std::setprecision(1)
              << elapsed_ms << " ms\n";

    // Find closest approach
    ProximityMonitor monitor;
    monitor.setScreeningThreshold(50000.0);  // 50 km screening threshold

    std::cout << "\nAnalyzing proximity...\n";
    start_time = std::chrono::high_resolution_clock::now();

    auto closest = monitor.findClosestApproach("PRIMARY", traj_primary,
                                               "SECONDARY", traj_secondary);

    end_time = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::cout << "\n=== Closest Approach Analysis ===\n";
    std::cout << "Analysis completed in " << elapsed_ms << " ms\n\n";

    int y, mo, d, h, m;
    double s;
    TimeSystem::jdToCalendar(closest.tca_jd, y, mo, d, h, m, s);

    std::cout << "Time of Closest Approach: " << y << "-"
              << std::setfill('0') << std::setw(2) << mo << "-"
              << std::setw(2) << d << " "
              << std::setw(2) << h << ":" << std::setw(2) << m << ":"
              << std::setw(2) << static_cast<int>(s) << " UTC\n";
    std::cout << std::setfill(' ');
    std::cout << "Miss Distance: " << std::setprecision(1) << closest.miss_distance << " m\n";
    std::cout << "Relative Velocity: " << std::setprecision(3) << closest.relative_velocity << " m/s\n";
    std::cout << "\nRelative Position (LVLH):\n";
    std::cout << "  R-bar (radial):      " << std::setprecision(1) << closest.relative_position.x << " m\n";
    std::cout << "  V-bar (along-track): " << closest.relative_position.y << " m\n";
    std::cout << "  H-bar (cross-track): " << closest.relative_position.z << " m\n";
    std::cout << "\nRisk Assessment:\n";
    std::cout << "  Collision Probability: " << std::scientific << std::setprecision(2)
              << closest.collision_probability << "\n";
    std::cout << "  Requires Maneuver: " << (closest.requires_maneuver ? "YES" : "NO") << "\n";

    // Screen for all conjunctions
    auto conjunctions = monitor.screenConjunctions("PRIMARY", traj_primary,
                                                   "SECONDARY", traj_secondary);
    auto summary = monitor.generateSummary(conjunctions);

    std::cout << "\n=== Conjunction Summary ===\n";
    std::cout << "Total Conjunctions: " << summary.total_conjunctions << "\n";
    std::cout << "Risk Level: " << summary.getRiskLevel() << "\n";
    std::cout << "  High Risk (< 1 km):   " << summary.high_risk << "\n";
    std::cout << "  Medium Risk (1-5 km): " << summary.medium_risk << "\n";
    std::cout << "  Low Risk (5-25 km):   " << summary.low_risk << "\n";

    // Save proximity data
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/proximity_analysis.csv");
    if (csv.is_open())
    {
        csv << "time_min,distance_m,rel_vel_ms,r_bar_m,v_bar_m,h_bar_m\n";

        for (size_t i = 0; i < std::min(traj_primary.size(), traj_secondary.size()); ++i)
        {
            double dist = ProximityMonitor::calculateDistance(
                traj_primary[i].second, traj_secondary[i].second);
            double rel_vel = ProximityMonitor::calculateRelativeVelocity(
                traj_primary[i].second, traj_secondary[i].second);

            auto [rel_pos, rel_vel_lvlh] = ProximityMonitor::getRelativeStateLVLH(
                traj_primary[i].second, traj_secondary[i].second);

            double time_min = (traj_primary[i].first - oe_primary.epoch) * 1440.0;

            csv << std::fixed << std::setprecision(2) << time_min << ","
                << std::setprecision(1) << dist << ","
                << std::setprecision(4) << rel_vel << ","
                << std::setprecision(1) << rel_pos.x << ","
                << rel_pos.y << "," << rel_pos.z << "\n";
        }
        csv.close();
        std::cout << "\nProximity data saved to results/data/proximity_analysis.csv\n";
    }
}

void testRendezvousPlanning()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Rendezvous Planning Test (CW Equations)\n";
    std::cout << "============================================================\n\n";

    // Reference orbit parameters
    double altitude = 400000.0;  // 400 km
    double r = constants::EARTH_RADIUS_EQ + altitude;
    double n = std::sqrt(constants::EARTH_MU / (r * r * r));
    double period_min = 2.0 * constants::PI / n / 60.0;

    std::cout << "Reference Orbit:\n";
    std::cout << "  Altitude: " << altitude / 1000.0 << " km\n";
    std::cout << "  Period: " << std::setprecision(1) << period_min << " minutes\n";
    std::cout << "  Mean motion: " << std::setprecision(6) << n << " rad/s\n";

    // Initial relative state (chaser 10 km behind and 100 m below)
    Vec3 rel_pos_0(-100.0, -10000.0, 0.0);  // R-bar, V-bar, H-bar
    Vec3 rel_vel_0(0.0, 0.0, 0.0);

    std::cout << "\nInitial Relative State (LVLH):\n";
    std::cout << "  R-bar: " << rel_pos_0.x << " m (radial)\n";
    std::cout << "  V-bar: " << rel_pos_0.y << " m (along-track)\n";
    std::cout << "  H-bar: " << rel_pos_0.z << " m (cross-track)\n";

    // Predict natural motion for 1 orbit
    std::cout << "\n=== Natural Drift Prediction (CW Equations) ===\n";

    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/cw_motion.csv");
    if (csv.is_open())
    {
        csv << "time_min,r_bar_m,v_bar_m,h_bar_m,range_m\n";
    }

    int num_points = 100;
    double T = period_min * 60.0;  // One orbit in seconds

    Vec3 rel_pos = rel_pos_0;
    Vec3 rel_vel = rel_vel_0;

    for (int i = 0; i <= num_points; ++i)
    {
        double dt = i * T / num_points;
        auto [pos, vel] = ProximityMonitor::predictRelativeMotionCW(rel_pos_0, rel_vel_0, n, dt);

        double range = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);

        if (csv.is_open())
        {
            csv << std::fixed << std::setprecision(2) << dt / 60.0 << ","
                << std::setprecision(1) << pos.x << "," << pos.y << ","
                << pos.z << "," << range << "\n";
        }

        if (i == 0 || i == num_points / 4 || i == num_points / 2 ||
            i == 3 * num_points / 4 || i == num_points)
        {
            std::cout << "t=" << std::setw(5) << std::setprecision(1) << dt / 60.0
                      << " min: R=" << std::setw(7) << std::setprecision(1) << pos.x
                      << " m, V=" << std::setw(8) << pos.y
                      << " m, H=" << std::setw(6) << pos.z
                      << " m, Range=" << std::setw(8) << range << " m\n";
        }
    }

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nCW motion data saved to results/data/cw_motion.csv\n";
    }

    // Calculate required delta-v for V-bar approach
    std::cout << "\n=== Rendezvous Maneuver Planning ===\n";

    double approach_distance = std::abs(rel_pos_0.y);  // 10 km
    double dv_vbar = RendezvousPlanner::calculateVbarHop(approach_distance, n);
    std::cout << "V-bar approach (10 km): Delta-V = " << std::setprecision(3)
              << dv_vbar << " m/s\n";

    double r_bar_error = std::abs(rel_pos_0.x);  // 100 m
    double dv_rbar = RendezvousPlanner::calculateRbarCorrection(r_bar_error, n);
    std::cout << "R-bar correction (100 m): Delta-V = " << std::setprecision(4)
              << dv_rbar << " m/s\n";
}

int main()
{
    std::cout << "============================================================\n";
    std::cout << "     HPOP Analysis Module Test Suite\n";
    std::cout << "============================================================\n";

    testContactPrediction();
    testProximityAnalysis();
    testRendezvousPlanning();

    std::cout << "\n============================================================\n";
    std::cout << "                    All Tests Complete\n";
    std::cout << "============================================================\n";

    return 0;
}
