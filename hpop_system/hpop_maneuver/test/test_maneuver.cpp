/**
 * @file test_maneuver.cpp
 * @brief Test orbital maneuver planning algorithms
 *
 * Tests Hohmann transfer, plane changes, Lambert solver.
 * Results saved for paper publication.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_maneuver/maneuver_planner.hpp"
#include "hpop_maneuver/lambert_solver.hpp"

using namespace hpop_core;
using namespace hpop_maneuver;

void printManeuverPlan(const ManeuverPlan& plan)
{
    std::cout << "Description: " << plan.description << "\n";
    std::cout << "Total Delta-V: " << std::fixed << std::setprecision(3)
              << plan.total_delta_v << " m/s\n";

    if (plan.transfer_time > 0)
    {
        double hours = plan.transfer_time / 3600.0;
        std::cout << "Transfer Time: " << std::setprecision(2) << hours << " hours\n";
    }

    std::cout << "Burns:\n";
    for (size_t i = 0; i < plan.burns.size(); ++i)
    {
        const auto& burn = plan.burns[i];
        std::cout << "  " << (i + 1) << ". " << burn.type << ": "
                  << std::setprecision(3) << burn.magnitude << " m/s\n";
    }
}

void testHohmannTransfer()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Hohmann Transfer Test\n";
    std::cout << "============================================================\n\n";

    // Test 1: LEO to MEO (GPS altitude)
    double r_leo = constants::EARTH_RADIUS_EQ + 400000.0;   // 400 km
    double r_meo = constants::EARTH_RADIUS_EQ + 20200000.0; // GPS altitude

    std::cout << "=== Test 1: LEO (400 km) to MEO (20,200 km) ===\n";
    std::cout << "Initial altitude: 400 km\n";
    std::cout << "Target altitude: 20,200 km\n\n";

    auto plan_leo_meo = HohmannTransfer::calculate(r_leo, r_meo);
    printManeuverPlan(plan_leo_meo);

    // Test 2: LEO to GEO
    double r_geo = 42164000.0;  // GEO radius

    std::cout << "\n=== Test 2: LEO (400 km) to GEO (35,786 km) ===\n";
    std::cout << "Initial altitude: 400 km\n";
    std::cout << "Target altitude: 35,786 km\n\n";

    auto plan_leo_geo = HohmannTransfer::calculate(r_leo, r_geo);
    printManeuverPlan(plan_leo_geo);

    // Test 3: ISS reboost (400 km to 420 km)
    double r_iss_low = constants::EARTH_RADIUS_EQ + 400000.0;
    double r_iss_high = constants::EARTH_RADIUS_EQ + 420000.0;

    std::cout << "\n=== Test 3: ISS Reboost (400 km to 420 km) ===\n";
    auto plan_reboost = HohmannTransfer::calculate(r_iss_low, r_iss_high);
    printManeuverPlan(plan_reboost);

    // Test 4: Compare Hohmann vs Bi-elliptic for large orbit change
    double r_low = constants::EARTH_RADIUS_EQ + 200000.0;
    double r_high = constants::EARTH_RADIUS_EQ + 100000000.0;  // Very high orbit

    std::cout << "\n=== Test 4: Hohmann vs Bi-elliptic Comparison ===\n";
    std::cout << "LEO (200 km) to very high orbit (100,000 km)\n";
    std::cout << "Altitude ratio: " << std::setprecision(1)
              << r_high / r_low << "\n\n";

    auto plan_hohmann = HohmannTransfer::calculate(r_low, r_high);
    std::cout << "Hohmann Transfer:\n";
    printManeuverPlan(plan_hohmann);

    std::cout << "\nBi-elliptic Transfer:\n";
    auto plan_bielliptic = HohmannTransfer::calculateBielliptic(r_low, r_high);
    printManeuverPlan(plan_bielliptic);

    std::cout << "\nDelta-V Savings: "
              << std::setprecision(1)
              << (plan_hohmann.total_delta_v - plan_bielliptic.total_delta_v) << " m/s ("
              << (100.0 * (plan_hohmann.total_delta_v - plan_bielliptic.total_delta_v) /
                  plan_hohmann.total_delta_v) << "%)\n";

    // Save results to CSV
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/hohmann_transfers.csv");
    if (csv.is_open())
    {
        csv << "transfer_type,r1_km,r2_km,dv1_ms,dv2_ms,total_dv_ms,tof_hours\n";

        csv << "LEO_to_MEO," << (r_leo - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << (r_meo - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << plan_leo_meo.burns[0].magnitude << ","
            << plan_leo_meo.burns[1].magnitude << ","
            << plan_leo_meo.total_delta_v << ","
            << plan_leo_meo.transfer_time / 3600.0 << "\n";

        csv << "LEO_to_GEO," << (r_leo - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << (r_geo - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << plan_leo_geo.burns[0].magnitude << ","
            << plan_leo_geo.burns[1].magnitude << ","
            << plan_leo_geo.total_delta_v << ","
            << plan_leo_geo.transfer_time / 3600.0 << "\n";

        csv << "ISS_reboost," << (r_iss_low - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << (r_iss_high - constants::EARTH_RADIUS_EQ) / 1000.0 << ","
            << plan_reboost.burns[0].magnitude << ","
            << plan_reboost.burns[1].magnitude << ","
            << plan_reboost.total_delta_v << ","
            << plan_reboost.transfer_time / 3600.0 << "\n";

        csv.close();
        std::cout << "\nHohmann transfer data saved to results/data/hohmann_transfers.csv\n";
    }
}

void testPlaneChange()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Plane Change Maneuver Test\n";
    std::cout << "============================================================\n\n";

    // ISS to polar orbit
    OrbitalElements oe_iss;
    oe_iss.sma = 6778000.0;
    oe_iss.ecc = 0.0001;
    oe_iss.inc = 51.64 * constants::DEG_TO_RAD;

    double target_inc = 90.0 * constants::DEG_TO_RAD;  // Polar

    std::cout << "=== ISS Orbit (51.64 deg) to Polar (90 deg) ===\n";
    auto plan = PlaneChange::calculate(oe_iss, target_inc);
    printManeuverPlan(plan);

    // Small inclination change
    std::cout << "\n=== Small Inclination Change (1 degree) ===\n";
    double v_iss = std::sqrt(constants::EARTH_MU / oe_iss.sma);
    double dv_1deg = PlaneChange::calculateDeltaV(v_iss, 1.0 * constants::DEG_TO_RAD);
    std::cout << "Orbital velocity: " << std::setprecision(1) << v_iss << " m/s\n";
    std::cout << "Delta-V for 1 deg change: " << std::setprecision(3) << dv_1deg << " m/s\n";

    // GEO inclination correction
    std::cout << "\n=== GEO Inclination Correction (0.05 deg) ===\n";
    double v_geo = std::sqrt(constants::EARTH_MU / 42164000.0);
    double dv_geo = PlaneChange::calculateDeltaV(v_geo, 0.05 * constants::DEG_TO_RAD);
    std::cout << "GEO orbital velocity: " << std::setprecision(1) << v_geo << " m/s\n";
    std::cout << "Delta-V for 0.05 deg correction: " << std::setprecision(3) << dv_geo << " m/s\n";
}

void testStationKeeping()
{
    std::cout << "\n============================================================\n";
    std::cout << "     GEO Station Keeping Budget\n";
    std::cout << "============================================================\n\n";

    // Typical GEO satellite
    double mass = 3000.0;  // kg
    double area = 50.0;    // m^2
    double cr = 1.5;

    double annual_dv = StationKeeping::annualBudget(area, mass, cr);

    std::cout << "Satellite: " << mass << " kg, " << area << " m^2, Cr=" << cr << "\n";
    std::cout << "Annual delta-V budget: " << std::setprecision(1) << annual_dv << " m/s/year\n";

    // 15-year mission
    double mission_life = 15.0;
    double total_dv = annual_dv * mission_life;
    std::cout << "15-year mission total: " << total_dv << " m/s\n";

    // Inclination correction
    double inc_drift = 0.1 * constants::DEG_TO_RAD;  // 0.1 deg/year typical
    double dv_inc = StationKeeping::inclinationCorrection(inc_drift);
    std::cout << "\nInclination drift (0.1 deg) correction: " << std::setprecision(3)
              << dv_inc << " m/s\n";
}

void testPhasingManeuver()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Phasing Maneuver Test\n";
    std::cout << "============================================================\n\n";

    double a_iss = 6778000.0;  // ISS orbit

    // Catch up by 30 degrees in 3 orbits
    std::cout << "=== Catch up 30 deg in 3 orbits ===\n";
    auto plan_30 = PhasingManeuver::calculate(a_iss, 30.0 * constants::DEG_TO_RAD, 3);
    printManeuverPlan(plan_30);

    // Fall back by 10 degrees in 1 orbit
    std::cout << "\n=== Fall back 10 deg in 1 orbit ===\n";
    auto plan_10 = PhasingManeuver::calculate(a_iss, -10.0 * constants::DEG_TO_RAD, 1);
    printManeuverPlan(plan_10);
}

void testLambertSolver()
{
    std::cout << "\n============================================================\n";
    std::cout << "     Lambert Solver Test\n";
    std::cout << "============================================================\n\n";

    // Simple transfer scenario
    // Starting position (LEO)
    Vec3 r1(6778000.0, 0.0, 0.0);  // LEO at x-axis
    Vec3 v1(0.0, 7670.0, 0.0);     // Circular velocity

    // Target position (after quarter orbit, raised apoapsis)
    Vec3 r2(0.0, 7000000.0, 500000.0);  // Quarter orbit away, slightly higher
    Vec3 v2(0.0, 0.0, 7500.0);           // Approximate target velocity

    // Time of flight: 30 minutes
    double tof = 30.0 * 60.0;

    std::cout << "=== Simple Lambert Transfer ===\n";
    std::cout << "Initial position: (" << r1.x / 1000.0 << ", " << r1.y / 1000.0
              << ", " << r1.z / 1000.0 << ") km\n";
    std::cout << "Target position: (" << r2.x / 1000.0 << ", " << r2.y / 1000.0
              << ", " << r2.z / 1000.0 << ") km\n";
    std::cout << "Time of flight: " << tof / 60.0 << " minutes\n\n";

    auto sol = LambertSolver::solve(r1, r2, tof);

    if (sol.converged)
    {
        std::cout << "Solution converged!\n";
        std::cout << "Transfer velocity at departure: ("
                  << std::setprecision(3)
                  << sol.v1.x << ", " << sol.v1.y << ", " << sol.v1.z << ") m/s\n";
        std::cout << "Transfer velocity at arrival: ("
                  << sol.v2.x << ", " << sol.v2.y << ", " << sol.v2.z << ") m/s\n";

        // Calculate delta-v
        Vec3 dv1 = sol.v1 - v1;
        Vec3 dv2 = v2 - sol.v2;
        std::cout << "\nDeparture delta-V: " << dv1.norm() << " m/s\n";
        std::cout << "Arrival delta-V: " << dv2.norm() << " m/s\n";
        std::cout << "Total delta-V: " << (dv1.norm() + dv2.norm()) << " m/s\n";
    }
    else
    {
        std::cout << "Solution did not converge.\n";
    }

    // Rendezvous scenario
    std::cout << "\n=== Rendezvous Scenario ===\n";

    StateVector chaser;
    chaser.position = Vec3(6778000.0, 0.0, 0.0);
    chaser.velocity = Vec3(0.0, 7670.0, 0.0);

    StateVector target;
    target.position = Vec3(6778000.0 * std::cos(0.1), 6778000.0 * std::sin(0.1), 0.0);
    target.velocity = Vec3(-7670.0 * std::sin(0.1), 7670.0 * std::cos(0.1), 0.0);

    std::cout << "Chaser at: (" << chaser.position.x / 1000.0 << ", "
              << chaser.position.y / 1000.0 << ") km\n";
    std::cout << "Target at: (" << target.position.x / 1000.0 << ", "
              << target.position.y / 1000.0 << ") km\n";

    double rdv_tof = 20.0 * 60.0;  // 20 minutes
    auto rdv_sol = LambertSolver::solveRendezvous(chaser, target, rdv_tof);

    if (rdv_sol.converged)
    {
        std::cout << "\nRendezvous solution found:\n";
        std::cout << "Departure delta-V: " << std::setprecision(3) << rdv_sol.dv1 << " m/s\n";
        std::cout << "Arrival delta-V: " << rdv_sol.dv2 << " m/s\n";
        std::cout << "Total delta-V: " << rdv_sol.total_dv << " m/s\n";
    }

    // Save Lambert results
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/lambert_solutions.csv");
    if (csv.is_open())
    {
        csv << "scenario,tof_min,dv1_ms,dv2_ms,total_dv_ms,converged\n";
        csv << "simple_transfer," << tof / 60.0 << ","
            << (sol.converged ? (sol.v1 - v1).norm() : 0) << ","
            << (sol.converged ? (v2 - sol.v2).norm() : 0) << ","
            << (sol.converged ? ((sol.v1 - v1).norm() + (v2 - sol.v2).norm()) : 0) << ","
            << (sol.converged ? "true" : "false") << "\n";
        csv << "rendezvous," << rdv_tof / 60.0 << ","
            << rdv_sol.dv1 << "," << rdv_sol.dv2 << "," << rdv_sol.total_dv << ","
            << (rdv_sol.converged ? "true" : "false") << "\n";
        csv.close();
        std::cout << "\nLambert solutions saved to results/data/lambert_solutions.csv\n";
    }
}

int main()
{
    std::cout << "============================================================\n";
    std::cout << "     HPOP Maneuver Planning Test Suite\n";
    std::cout << "============================================================\n";

    testHohmannTransfer();
    testPlaneChange();
    testStationKeeping();
    testPhasingManeuver();
    testLambertSolver();

    std::cout << "\n============================================================\n";
    std::cout << "                    All Tests Complete\n";
    std::cout << "============================================================\n";

    return 0;
}
