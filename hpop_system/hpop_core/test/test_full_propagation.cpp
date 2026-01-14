/**
 * @file test_full_propagation.cpp
 * @brief Comprehensive orbit propagation test with all perturbation models
 *
 * Tests propagation accuracy with:
 * - J2 gravitational perturbation
 * - Atmospheric drag
 * - Solar radiation pressure
 * - Third-body perturbations (Sun/Moon)
 *
 * Results are saved for paper publication.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cmath>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_core/propagator.hpp"
#include "hpop_perturbations/drag_model.hpp"
#include "hpop_perturbations/srp_model.hpp"
#include "hpop_perturbations/third_body.hpp"

using namespace hpop_core;
using namespace hpop_perturbations;

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

void printHeader()
{
    std::cout << "============================================================\n";
    std::cout << "     HPOP Full Perturbation Propagation Test Suite\n";
    std::cout << "============================================================\n";
}

void testISSPropagation()
{
    std::cout << "\n=== Test 1: ISS-like Orbit Propagation (24 hours) ===\n";

    // ISS-like orbital elements
    OrbitalElements oe;
    oe.sma = 6778000.0;           // Semi-major axis [m] (~400 km altitude)
    oe.ecc = 0.0001;              // Eccentricity (nearly circular)
    oe.inc = 51.64 * constants::DEG_TO_RAD;   // Inclination [rad]
    oe.raan = 0.0;                // RAAN [rad]
    oe.aop = 0.0;                 // Argument of periapsis [rad]
    oe.ta = 0.0;                  // True anomaly [rad]
    oe.epoch = TimeSystem::calendarToJD(2024, 6, 21, 0, 0, 0);

    StateVector initial_state = oe.toStateVector();

    // Spacecraft parameters (ISS-like)
    double mass = 420000.0;       // kg
    double area = 1500.0;         // m^2
    double cd = 2.2;
    double cr = 1.3;

    // Setup perturbation models
    DragModel drag;
    drag.setSpacecraftParams(cd, area, mass);

    SRPModel srp;
    srp.setSpacecraftParams(cr, area, mass);

    ThirdBodyModel third_body;

    // Create propagator configurations
    struct TestCase
    {
        std::string name;
        std::vector<std::string> perturbations;
    };

    std::vector<TestCase> test_cases = {
        {"Two-body only", {}},
        {"Two-body + J2", {"J2"}},
        {"Two-body + J2 + Drag", {"J2", "Drag"}},
        {"Two-body + J2 + Drag + SRP", {"J2", "Drag", "SRP"}},
        {"Full (J2 + Drag + SRP + 3rd-body)", {"J2", "Drag", "SRP", "3rd-body"}}
    };

    // Propagation parameters
    double duration = 24.0 * 3600.0;  // 24 hours
    double output_step = 600.0;       // Output every 10 minutes

    // Open CSV file for results
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/full_propagation_comparison.csv");
    if (csv.is_open())
    {
        csv << "time_h,case_name,x_km,y_km,z_km,vx_kmps,vy_kmps,vz_kmps,r_km,altitude_km,sma_km,ecc,inc_deg\n";
    }

    std::cout << "\nComparing propagation with different perturbation models:\n";
    std::cout << "Duration: 24 hours, Output step: 10 minutes\n";
    std::cout << "Spacecraft: Mass=" << mass << " kg, Area=" << area << " m2, Cd=" << cd << ", Cr=" << cr << "\n\n";

    for (const auto& test : test_cases)
    {
        std::cout << "Running: " << test.name << "... ";
        std::cout.flush();

        OrbitPropagator propagator;
        propagator.setIntegrator(IntegratorType::RKF78);
        propagator.setStepSize(60.0);
        propagator.setTolerance(1e-12);

        // Add selected perturbations
        for (const auto& pert : test.perturbations)
        {
            if (pert == "J2")
            {
                propagator.addForceModel("J2", j2Perturbation);
            }
            else if (pert == "Drag")
            {
                propagator.addForceModel("Drag", [&drag](const Vec3& pos, const Vec3& vel, double jd) {
                    StateVector state;
                    state.position = pos;
                    state.velocity = vel;
                    state.epoch = jd;
                    return drag.acceleration(state, jd);
                });
            }
            else if (pert == "SRP")
            {
                propagator.addForceModel("SRP", [&srp](const Vec3& pos, const Vec3& vel, double jd) {
                    StateVector state;
                    state.position = pos;
                    state.velocity = vel;
                    state.epoch = jd;
                    Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
                    return srp.acceleration(state, sun_pos);
                });
            }
            else if (pert == "3rd-body")
            {
                propagator.addForceModel("3rd-body", [&third_body](const Vec3& pos, const Vec3& vel, double jd) {
                    StateVector state;
                    state.position = pos;
                    state.velocity = vel;
                    state.epoch = jd;
                    return third_body.acceleration(state, jd);
                });
            }
        }

        // Propagate and record trajectory
        StateVector state = initial_state;
        double t = 0.0;

        auto start_time = std::chrono::high_resolution_clock::now();

        while (t <= duration)
        {
            // Record state
            if (csv.is_open() && std::fmod(t, output_step) < 1.0)
            {
                double r = state.position.norm();
                double alt = r - constants::EARTH_RADIUS_EQ;
                OrbitalElements oe_current = OrbitalElements::fromStateVector(state);

                csv << (t / 3600.0) << "," << test.name << ","
                    << state.position.x / 1000.0 << "," << state.position.y / 1000.0 << ","
                    << state.position.z / 1000.0 << ","
                    << state.velocity.x / 1000.0 << "," << state.velocity.y / 1000.0 << ","
                    << state.velocity.z / 1000.0 << ","
                    << r / 1000.0 << "," << alt / 1000.0 << ","
                    << oe_current.sma / 1000.0 << "," << oe_current.ecc << ","
                    << oe_current.inc * constants::RAD_TO_DEG << "\n";
            }

            // Propagate one step
            double step = std::min(60.0, duration - t);
            if (step <= 0) break;

            PropagationStep result = propagator.singleStep(state, step);
            state = result.state;
            t += step;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Final state comparison
        double dr = (state.position - initial_state.position).norm();
        OrbitalElements final_oe = OrbitalElements::fromStateVector(state);

        std::cout << "Done! (" << std::fixed << std::setprecision(1) << elapsed_ms << " ms)\n";
        std::cout << "  Final position difference: " << std::setprecision(3) << dr / 1000.0 << " km\n";
        std::cout << "  SMA change: " << (final_oe.sma - oe.sma) / 1000.0 << " km\n";
        std::cout << "  Ecc change: " << std::scientific << std::setprecision(4)
                  << (final_oe.ecc - oe.ecc) << "\n";
    }

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nTrajectory data saved to results/data/full_propagation_comparison.csv\n";
    }
}

void testOrbitDecay()
{
    std::cout << "\n=== Test 2: LEO Orbit Decay Simulation (30 days) ===\n";

    // LEO orbit (350 km altitude) - moderate drag for realistic decay
    OrbitalElements oe;
    oe.sma = 6728000.0;           // ~350 km altitude
    oe.ecc = 0.001;
    oe.inc = 51.64 * constants::DEG_TO_RAD;
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::calendarToJD(2024, 1, 1, 0, 0, 0);

    StateVector initial_state = oe.toStateVector();
    double initial_altitude = (oe.sma - constants::EARTH_RADIUS_EQ) / 1000.0;

    // Spacecraft parameters (small satellite, high area-to-mass ratio)
    double mass = 50.0;           // 50 kg cubesat
    double area = 0.5;            // 0.5 m^2 cross-section
    double cd = 2.2;

    DragModel drag;
    drag.setSpacecraftParams(cd, area, mass);

    OrbitPropagator propagator;
    propagator.setIntegrator(IntegratorType::RKF78);
    propagator.setStepSize(60.0);
    propagator.setTolerance(1e-10);
    propagator.addForceModel("J2", j2Perturbation);
    propagator.addForceModel("Drag", [&drag](const Vec3& pos, const Vec3& vel, double jd) {
        StateVector state;
        state.position = pos;
        state.velocity = vel;
        state.epoch = jd;
        return drag.acceleration(state, jd);
    });

    double duration = 30.0 * 24.0 * 3600.0;  // 30 days
    double output_step = 3600.0;              // Every hour
    double reentry_altitude = 80000.0;        // 80 km (re-entry threshold)

    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/orbit_decay.csv");
    if (csv.is_open())
    {
        csv << "time_h,altitude_km,sma_km,ecc,period_min\n";
    }

    std::cout << "Initial altitude: " << std::fixed << std::setprecision(2) << initial_altitude << " km\n";
    std::cout << "Mass: " << mass << " kg, Area: " << area << " m2, Cd: " << cd << "\n";
    std::cout << "Area-to-mass ratio: " << (area / mass) * 1000.0 << " cm2/kg\n";
    std::cout << "Propagating for up to 30 days with J2 + Drag...\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    StateVector state = initial_state;
    double t = 0.0;
    bool reentry_detected = false;
    double reentry_time = 0.0;

    while (t <= duration)
    {
        // Calculate current altitude
        double r = state.position.norm();
        double current_altitude = r - constants::EARTH_RADIUS_EQ;

        // Check for re-entry
        if (current_altitude < reentry_altitude)
        {
            reentry_detected = true;
            reentry_time = t;
            std::cout << "*** Re-entry detected at t=" << (t / 3600.0) << " hours! ***\n";
            std::cout << "    Final altitude: " << (current_altitude / 1000.0) << " km\n";
            break;
        }

        // Record data at output intervals
        if (std::fmod(t, output_step) < 1.0)
        {
            OrbitalElements oe_current = OrbitalElements::fromStateVector(state);
            double alt = oe_current.sma - constants::EARTH_RADIUS_EQ;
            double period = 2.0 * constants::PI * std::sqrt(std::pow(oe_current.sma, 3) / constants::EARTH_MU);

            if (csv.is_open())
            {
                csv << (t / 3600.0) << "," << alt / 1000.0 << ","
                    << oe_current.sma / 1000.0 << "," << oe_current.ecc << ","
                    << period / 60.0 << "\n";
            }
        }

        double step = std::min(60.0, duration - t);
        if (step <= 0) break;

        PropagationStep result = propagator.singleStep(state, step);
        state = result.state;
        t += step;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_s = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "Completed in " << std::fixed << std::setprecision(2) << elapsed_s << " seconds\n";

    if (!reentry_detected)
    {
        OrbitalElements final_oe = OrbitalElements::fromStateVector(state);
        double final_altitude = (final_oe.sma - constants::EARTH_RADIUS_EQ) / 1000.0;
        double alt_loss = initial_altitude - final_altitude;
        double days_elapsed = t / 86400.0;

        std::cout << "Final altitude: " << final_altitude << " km\n";
        std::cout << "Altitude loss: " << alt_loss << " km over " << days_elapsed << " days\n";
        std::cout << "Average decay rate: " << (alt_loss / days_elapsed) << " km/day\n";
    }
    else
    {
        double days_to_reentry = reentry_time / 86400.0;
        std::cout << "Satellite re-entered after " << std::setprecision(1) << days_to_reentry << " days\n";
    }

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nOrbit decay data saved to results/data/orbit_decay.csv\n";
    }
}

void testGEOStationKeeping()
{
    std::cout << "\n=== Test 3: GEO Satellite Perturbations (30 days) ===\n";

    // GEO orbit
    double r_geo = 42164000.0;  // GEO radius [m]
    OrbitalElements oe;
    oe.sma = r_geo;
    oe.ecc = 0.0001;
    oe.inc = 0.1 * constants::DEG_TO_RAD;  // Slight inclination
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::calendarToJD(2024, 1, 1, 0, 0, 0);

    StateVector initial_state = oe.toStateVector();

    // Spacecraft parameters
    double mass = 3000.0;
    double area = 50.0;
    double cr = 1.5;

    SRPModel srp;
    srp.setSpacecraftParams(cr, area, mass);

    ThirdBodyModel third_body;

    OrbitPropagator propagator;
    propagator.setIntegrator(IntegratorType::RKF78);
    propagator.setStepSize(300.0);  // 5 minute steps for GEO
    propagator.setTolerance(1e-10);
    propagator.addForceModel("J2", j2Perturbation);
    propagator.addForceModel("SRP", [&srp](const Vec3& pos, const Vec3& vel, double jd) {
        StateVector state;
        state.position = pos;
        state.velocity = vel;
        Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
        return srp.acceleration(state, sun_pos);
    });
    propagator.addForceModel("3rd-body", [&third_body](const Vec3& pos, const Vec3& vel, double jd) {
        StateVector state;
        state.position = pos;
        state.velocity = vel;
        state.epoch = jd;
        return third_body.acceleration(state, jd);
    });

    double duration = 30.0 * 24.0 * 3600.0;  // 30 days
    double output_step = 6.0 * 3600.0;       // Every 6 hours

    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/geo_perturbations.csv");
    if (csv.is_open())
    {
        csv << "time_days,r_km,altitude_km,inc_deg,ecc\n";
    }

    std::cout << "GEO satellite with SRP and third-body perturbations\n";
    std::cout << "Mass: " << mass << " kg, Area: " << area << " m2, Cr: " << cr << "\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    StateVector state = initial_state;
    double t = 0.0;

    while (t <= duration)
    {
        if (std::fmod(t, output_step) < 10.0)
        {
            double r = state.position.norm();
            double alt = r - constants::EARTH_RADIUS_EQ;
            OrbitalElements oe_current = OrbitalElements::fromStateVector(state);

            if (csv.is_open())
            {
                csv << (t / 86400.0) << "," << r / 1000.0 << ","
                    << alt / 1000.0 << ","
                    << oe_current.inc * constants::RAD_TO_DEG << ","
                    << oe_current.ecc << "\n";
            }
        }

        double step = std::min(300.0, duration - t);
        if (step <= 0) break;

        PropagationStep result = propagator.singleStep(state, step);
        state = result.state;
        t += step;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_s = std::chrono::duration<double>(end_time - start_time).count();

    OrbitalElements final_oe = OrbitalElements::fromStateVector(state);

    std::cout << "Completed in " << std::fixed << std::setprecision(2) << elapsed_s << " seconds\n";
    std::cout << "Inclination drift: " << (final_oe.inc - oe.inc) * constants::RAD_TO_DEG << " deg/month\n";
    std::cout << "Eccentricity change: " << std::scientific << final_oe.ecc - oe.ecc << "\n";

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nGEO data saved to results/data/geo_perturbations.csv\n";
    }
}

void testPropagatorPerformance()
{
    std::cout << "\n=== Test 4: Propagator Performance Benchmark ===\n";

    OrbitalElements oe;
    oe.sma = 7000000.0;
    oe.ecc = 0.01;
    oe.inc = 45.0 * constants::DEG_TO_RAD;
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::calendarToJD(2024, 1, 1, 0, 0, 0);

    StateVector initial_state = oe.toStateVector();

    DragModel drag;
    drag.setSpacecraftParams(2.2, 10.0, 1000.0);

    SRPModel srp;
    srp.setSpacecraftParams(1.4, 10.0, 1000.0);

    ThirdBodyModel third_body;

    double duration = 24.0 * 3600.0;

    std::cout << "\nPerformance for 24-hour propagation:\n";
    std::cout << "Configuration          | Time (ms) | Steps/sec\n";
    std::cout << "-----------------------+-----------+----------\n";

    // RK4 vs RKF78
    for (auto integrator : {IntegratorType::RK4, IntegratorType::RKF78})
    {
        OrbitPropagator propagator;
        propagator.setIntegrator(integrator);
        propagator.setStepSize(60.0);
        propagator.setTolerance(1e-10);
        propagator.addForceModel("J2", j2Perturbation);
        propagator.addForceModel("Drag", [&drag](const Vec3& pos, const Vec3& vel, double jd) {
            StateVector state;
            state.position = pos;
            state.velocity = vel;
            state.epoch = jd;
            return drag.acceleration(state, jd);
        });
        propagator.addForceModel("SRP", [&srp](const Vec3& pos, const Vec3& vel, double jd) {
            StateVector state;
            state.position = pos;
            state.velocity = vel;
            Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
            return srp.acceleration(state, sun_pos);
        });
        propagator.addForceModel("3rd-body", [&third_body](const Vec3& pos, const Vec3& vel, double jd) {
            StateVector state;
            state.position = pos;
            state.velocity = vel;
            state.epoch = jd;
            return third_body.acceleration(state, jd);
        });

        auto start = std::chrono::high_resolution_clock::now();
        int num_steps = 0;

        StateVector state = initial_state;
        double t = 0.0;
        while (t < duration)
        {
            PropagationStep result = propagator.singleStep(state, 60.0);
            state = result.state;
            t += 60.0;
            num_steps++;
        }

        auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

        std::cout << std::left << std::setw(22) << propagator.integratorName()
                  << " | " << std::fixed << std::setprecision(1)
                  << std::setw(9) << elapsed_ms << " | "
                  << std::setw(8) << (num_steps * 1000.0 / elapsed_ms) << "\n";
    }
}

int main()
{
    printHeader();

    testISSPropagation();
    testOrbitDecay();
    testGEOStationKeeping();
    testPropagatorPerformance();

    std::cout << "\n============================================================\n";
    std::cout << "                    All Tests Complete\n";
    std::cout << "============================================================\n";

    // Save log
    std::ofstream log("/home/seongmin/ros2_ws/src/lrs_dynamics/results/logs/full_propagation_test.log",
                      std::ios::app);
    if (log.is_open())
    {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        log << "\n=== Full Propagation Test: " << std::ctime(&time);
        log << "Tests completed:\n";
        log << "  - ISS-like orbit with all perturbations (24h)\n";
        log << "  - LEO orbit decay simulation (7 days)\n";
        log << "  - GEO station-keeping perturbations (30 days)\n";
        log << "  - Performance benchmark (RK4 vs RKF78)\n";
    }

    return 0;
}
