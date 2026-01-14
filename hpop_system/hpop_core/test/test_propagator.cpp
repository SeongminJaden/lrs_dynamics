#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <fstream>

#include "hpop_core/propagator.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"

using namespace hpop_core;

void printState(const std::string& label, const StateVector& state)
{
    std::cout << label << ":\n";
    std::cout << "  Position: [" << std::fixed << std::setprecision(3)
              << state.position.x / 1000.0 << ", "
              << state.position.y / 1000.0 << ", "
              << state.position.z / 1000.0 << "] km\n";
    std::cout << "  Velocity: [" << std::setprecision(6)
              << state.velocity.x / 1000.0 << ", "
              << state.velocity.y / 1000.0 << ", "
              << state.velocity.z / 1000.0 << "] km/s\n";
    std::cout << "  Radius: " << state.radius() / 1000.0 << " km\n";
    std::cout << "  Speed: " << state.speed() / 1000.0 << " km/s\n";
}

void testOrbitalElements()
{
    std::cout << "\n=== Orbital Elements Conversion Test ===\n\n";

    // ISS-like orbit
    OrbitalElements oe;
    oe.sma = 6778000.0;       // ~400 km altitude
    oe.ecc = 0.0001;          // Nearly circular
    oe.inc = 51.6 * constants::DEG_TO_RAD;
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;

    std::cout << "Original Orbital Elements:\n";
    std::cout << "  SMA: " << oe.sma / 1000.0 << " km\n";
    std::cout << "  ECC: " << oe.ecc << "\n";
    std::cout << "  INC: " << oe.inc * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  RAAN: " << oe.raan * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  AOP: " << oe.aop * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  TA: " << oe.ta * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  Period: " << oe.period() / 60.0 << " min\n\n";

    // Convert to state vector
    StateVector state = oe.toStateVector();
    printState("State Vector", state);

    // Convert back to orbital elements
    OrbitalElements oe2 = OrbitalElements::fromStateVector(state);

    std::cout << "\nConverted back:\n";
    std::cout << "  SMA error: " << std::abs(oe2.sma - oe.sma) << " m\n";
    std::cout << "  ECC error: " << std::abs(oe2.ecc - oe.ecc) << "\n";
    std::cout << "  INC error: " << std::abs(oe2.inc - oe.inc) * constants::RAD_TO_DEG << " deg\n";
}

void testCoordinateFrames()
{
    std::cout << "\n=== Coordinate Frame Conversion Test ===\n\n";

    // Test geodetic <-> ECEF
    GeodeticCoord daejeon;
    daejeon.latitude = 36.35 * constants::DEG_TO_RAD;
    daejeon.longitude = 127.38 * constants::DEG_TO_RAD;
    daejeon.altitude = 100.0;

    Vec3 ecef = CoordinateFrames::geodeticToEcef(daejeon);
    std::cout << "Daejeon Station:\n";
    std::cout << "  Geodetic: lat=" << daejeon.latitude * constants::RAD_TO_DEG
              << " deg, lon=" << daejeon.longitude * constants::RAD_TO_DEG
              << " deg, alt=" << daejeon.altitude << " m\n";
    std::cout << "  ECEF: [" << ecef.x / 1000.0 << ", " << ecef.y / 1000.0
              << ", " << ecef.z / 1000.0 << "] km\n";

    // Convert back
    GeodeticCoord geo2 = CoordinateFrames::ecefToGeodetic(ecef);
    std::cout << "  Round-trip error:\n";
    std::cout << "    Lat: " << std::abs(geo2.latitude - daejeon.latitude) * constants::RAD_TO_DEG * 3600 << " arcsec\n";
    std::cout << "    Lon: " << std::abs(geo2.longitude - daejeon.longitude) * constants::RAD_TO_DEG * 3600 << " arcsec\n";
    std::cout << "    Alt: " << std::abs(geo2.altitude - daejeon.altitude) << " m\n";

    // Test ECI <-> ECEF
    std::cout << "\nECI <-> ECEF Test:\n";
    double jd = TimeSystem::currentJD();
    StateVector sat_eci;
    sat_eci.position = {6878000.0, 0.0, 0.0};
    sat_eci.velocity = {0.0, 7700.0, 0.0};
    sat_eci.epoch = jd;

    StateVector sat_ecef = CoordinateFrames::eciToEcef(sat_eci, jd);
    StateVector sat_eci2 = CoordinateFrames::ecefToEci(sat_ecef, jd);

    std::cout << "  Position round-trip error: "
              << (sat_eci2.position - sat_eci.position).norm() << " m\n";
    std::cout << "  Velocity round-trip error: "
              << (sat_eci2.velocity - sat_eci.velocity).norm() << " m/s\n";
}

void testPropagator(const std::string& output_dir)
{
    std::cout << "\n=== Orbit Propagator Test ===\n\n";

    OrbitPropagator propagator;

    // Test with ISS-like orbit
    OrbitalElements oe;
    oe.sma = 6778000.0;
    oe.ecc = 0.0001;
    oe.inc = 51.6 * constants::DEG_TO_RAD;
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::currentJD();

    StateVector initial = oe.toStateVector();
    initial.epoch = oe.epoch;

    // Test both integrators
    for (auto type : {IntegratorType::RK4, IntegratorType::RKF78})
    {
        propagator.setIntegrator(type);
        propagator.setStepSize(60.0);
        propagator.setTolerance(1e-10);

        std::cout << "Integrator: " << propagator.integratorName() << "\n";

        // Propagate for one orbital period
        double period = oe.period();
        std::vector<PropagationStep> trajectory;

        auto t0 = std::chrono::high_resolution_clock::now();
        StateVector final_state = propagator.propagate(initial, period, &trajectory);
        auto t1 = std::chrono::high_resolution_clock::now();

        double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Check closure (should return to same position after one period)
        double pos_error = (final_state.position - initial.position).norm();
        double vel_error = (final_state.velocity - initial.velocity).norm();

        std::cout << "  Propagation time: " << elapsed_ms << " ms\n";
        std::cout << "  Trajectory points: " << trajectory.size() << "\n";
        std::cout << "  Period closure error:\n";
        std::cout << "    Position: " << pos_error << " m\n";
        std::cout << "    Velocity: " << vel_error << " m/s\n\n";

        // Save trajectory to file for paper
        std::string filename = output_dir + "/" + propagator.integratorName() + "_trajectory.csv";
        std::ofstream file(filename);
        if (file.is_open())
        {
            file << "time_s,x_km,y_km,z_km,vx_km_s,vy_km_s,vz_km_s,r_km,v_km_s,ax,ay,az\n";
            double t = 0;
            file << std::fixed << std::setprecision(6);
            for (const auto& step : trajectory)
            {
                t += step.dt;
                file << t << ","
                     << step.state.position.x / 1000.0 << ","
                     << step.state.position.y / 1000.0 << ","
                     << step.state.position.z / 1000.0 << ","
                     << step.state.velocity.x / 1000.0 << ","
                     << step.state.velocity.y / 1000.0 << ","
                     << step.state.velocity.z / 1000.0 << ","
                     << step.state.radius() / 1000.0 << ","
                     << step.state.speed() / 1000.0 << ","
                     << step.acceleration.x << ","
                     << step.acceleration.y << ","
                     << step.acceleration.z << "\n";
            }
            file.close();
            std::cout << "  Saved trajectory to: " << filename << "\n\n";
        }
    }
}

void testJ2Perturbation(const std::string& output_dir)
{
    std::cout << "\n=== J2 Perturbation Effect Test ===\n\n";

    OrbitPropagator propagator;
    propagator.setIntegrator(IntegratorType::RKF78);
    propagator.setTolerance(1e-12);

    // Add J2 perturbation force model
    propagator.addForceModel("J2", [](const Vec3& pos, const Vec3&, double) -> Vec3 {
        double r = pos.norm();
        double r2 = r * r;
        double z2 = pos.z * pos.z;

        double factor = -1.5 * constants::EARTH_MU * constants::EARTH_J2
                       * constants::EARTH_RADIUS_EQ * constants::EARTH_RADIUS_EQ / (r2 * r2 * r);

        double coef = 5.0 * z2 / r2 - 1.0;
        return Vec3{
            factor * pos.x * coef,
            factor * pos.y * coef,
            factor * pos.z * (5.0 * z2 / r2 - 3.0)
        };
    });

    // Sun-synchronous orbit
    OrbitalElements oe;
    oe.sma = 7078000.0;  // ~700 km altitude
    oe.ecc = 0.001;
    oe.inc = 98.2 * constants::DEG_TO_RAD;  // Sun-synchronous
    oe.raan = 0.0;
    oe.aop = 0.0;
    oe.ta = 0.0;
    oe.epoch = TimeSystem::currentJD();

    StateVector initial = oe.toStateVector();
    initial.epoch = oe.epoch;

    // Propagate for 1 day
    double duration = 86400.0;  // 1 day
    std::vector<PropagationStep> trajectory;

    auto t0 = std::chrono::high_resolution_clock::now();
    StateVector final_state = propagator.propagate(initial, duration, &trajectory);
    auto t1 = std::chrono::high_resolution_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Calculate orbital element changes
    OrbitalElements final_oe = OrbitalElements::fromStateVector(final_state);

    std::cout << "1-day propagation with J2:\n";
    std::cout << "  Computation time: " << elapsed_ms << " ms\n";
    std::cout << "  Trajectory points: " << trajectory.size() << "\n";
    std::cout << "\nOrbital element changes:\n";
    std::cout << "  RAAN drift: " << (final_oe.raan - oe.raan) * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  AOP drift: " << (final_oe.aop - oe.aop) * constants::RAD_TO_DEG << " deg\n";
    std::cout << "  SMA change: " << (final_oe.sma - oe.sma) << " m\n";

    // Expected RAAN drift for sun-sync
    double n = oe.meanMotion();
    double a = oe.sma;
    double Re = constants::EARTH_RADIUS_EQ;
    double expected_raan_rate = -1.5 * n * constants::EARTH_J2 * (Re * Re / (a * a))
                               * std::cos(oe.inc) / std::pow(1 - oe.ecc * oe.ecc, 2);
    double expected_raan_drift = expected_raan_rate * duration * constants::RAD_TO_DEG;

    std::cout << "  Expected RAAN drift: " << expected_raan_drift << " deg\n";

    // Save J2 propagation results
    std::string filename = output_dir + "/j2_propagation.csv";
    std::ofstream file(filename);
    if (file.is_open())
    {
        file << "time_h,x_km,y_km,z_km,r_km,inc_deg,raan_deg\n";
        double t = 0;
        file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < trajectory.size(); i += 10)  // Every 10th point
        {
            const auto& step = trajectory[i];
            OrbitalElements step_oe = OrbitalElements::fromStateVector(step.state);
            t += step.dt * 10;
            file << t / 3600.0 << ","
                 << step.state.position.x / 1000.0 << ","
                 << step.state.position.y / 1000.0 << ","
                 << step.state.position.z / 1000.0 << ","
                 << step.state.radius() / 1000.0 << ","
                 << step_oe.inc * constants::RAD_TO_DEG << ","
                 << step_oe.raan * constants::RAD_TO_DEG << "\n";
        }
        file.close();
        std::cout << "\nSaved J2 propagation to: " << filename << "\n";
    }
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "   HPOP Core Library Test Suite\n";
    std::cout << "========================================\n";

    // Output directory for paper materials
    std::string output_dir = "/home/seongmin/ros2_ws/src/lrs_dynamics/results/data";

    testOrbitalElements();
    testCoordinateFrames();
    testPropagator(output_dir);
    testJ2Perturbation(output_dir);

    std::cout << "\n========================================\n";
    std::cout << "   All tests completed!\n";
    std::cout << "========================================\n";

    return 0;
}
