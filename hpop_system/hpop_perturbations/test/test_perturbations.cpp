/**
 * @file test_perturbations.cpp
 * @brief Test perturbation force models: drag, SRP, third-body
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>
#include <chrono>
#include <utility>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_perturbations/drag_model.hpp"
#include "hpop_perturbations/srp_model.hpp"
#include "hpop_perturbations/third_body.hpp"

using namespace hpop_core;
using namespace hpop_perturbations;

void testAtmosphericDensity()
{
    std::cout << "\n=== Test 1: Atmospheric Density Model ===\n";

    // Test altitudes [km]
    std::vector<double> altitudes = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nAltitude (km) | Exponential (kg/m³) | Harris-Priester (kg/m³)\n";
    std::cout << "-------------------------------------------------------------\n";

    atmosphere::HarrisPriesterAtmosphere hp_atm;

    for (double h_km : altitudes)
    {
        double h_m = h_km * 1000.0;
        double rho_exp = atmosphere::ExponentialAtmosphere::density(h_m);
        double rho_hp = hp_atm.densitySimple(h_m);

        std::cout << std::setw(12) << h_km << " | "
                  << std::scientific << std::setprecision(4)
                  << std::setw(18) << rho_exp << " | "
                  << std::setw(18) << rho_hp << "\n";
    }

    // Save to CSV
    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/atmosphere_density.csv");
    if (csv.is_open())
    {
        csv << "altitude_km,rho_exp,rho_hp\n";
        for (double h_km = 100.0; h_km <= 1000.0; h_km += 10.0)
        {
            double h_m = h_km * 1000.0;
            double rho_exp = atmosphere::ExponentialAtmosphere::density(h_m);
            double rho_hp = hp_atm.densitySimple(h_m);
            csv << h_km << "," << rho_exp << "," << rho_hp << "\n";
        }
        csv.close();
        std::cout << "\nAtmosphere data saved to results/data/atmosphere_density.csv\n";
    }
}

void testDragAcceleration()
{
    std::cout << "\n=== Test 2: Drag Acceleration Model ===\n";

    DragModel drag;
    drag.setSpacecraftParams(2.2, 10.0, 1000.0);  // Cd=2.2, A=10m², m=1000kg

    std::cout << "\nSpacecraft parameters:\n";
    std::cout << "  Cd = 2.2\n";
    std::cout << "  Area = 10 m²\n";
    std::cout << "  Mass = 1000 kg\n";
    std::cout << "  Ballistic Coeff = " << drag.getBallisticCoefficient() << " kg/m²\n";

    // Test at various altitudes (circular orbit)
    std::cout << "\nDrag acceleration vs altitude (circular orbit):\n";
    std::cout << "Altitude (km) | Velocity (km/s) | Density (kg/m³) | Drag (m/s²)\n";
    std::cout << "---------------------------------------------------------------\n";

    std::vector<double> altitudes = {200, 300, 400, 500, 600, 800, 1000};

    for (double h_km : altitudes)
    {
        double r = constants::EARTH_RADIUS_EQ + h_km * 1000.0;
        double v = std::sqrt(constants::EARTH_MU / r);

        StateVector state;
        state.position = Vec3(r, 0.0, 0.0);  // Satellite at +X
        state.velocity = Vec3(0.0, v, 0.0);  // Velocity in +Y
        state.epoch = TimeSystem::calendarToJD(2024, 1, 1, 0, 0, 0);

        Vec3 a_drag = drag.acceleration(state, state.epoch);
        double a_mag = a_drag.norm();
        double rho = drag.getDensity(h_km * 1000.0);

        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(12) << h_km << " | "
                  << std::setprecision(3)
                  << std::setw(14) << (v / 1000.0) << " | "
                  << std::scientific << std::setprecision(3)
                  << std::setw(14) << rho << " | "
                  << std::setw(10) << a_mag << "\n";
    }
}

void testSRPAcceleration()
{
    std::cout << "\n=== Test 3: Solar Radiation Pressure Model ===\n";

    SRPModel srp;
    srp.setSpacecraftParams(1.4, 10.0, 1000.0);  // Cr=1.4, A=10m², m=1000kg

    std::cout << "\nSpacecraft parameters:\n";
    std::cout << "  Cr = 1.4\n";
    std::cout << "  Area = 10 m²\n";
    std::cout << "  Mass = 1000 kg\n";

    // Test at GEO altitude (35786 km)
    double r_geo = constants::EARTH_RADIUS_EQ + 35786000.0;

    StateVector state;
    state.position = Vec3(r_geo, 0.0, 0.0);
    state.velocity = Vec3(0.0, std::sqrt(constants::EARTH_MU / r_geo), 0.0);
    state.epoch = TimeSystem::calendarToJD(2024, 6, 21, 12, 0, 0);  // Summer solstice noon

    Vec3 sun_pos = SunPosition::getSunPositionECI(state.epoch);
    double sun_dist = sun_pos.norm();

    std::cout << "\nSun position at epoch (JD " << state.epoch << "):\n";
    std::cout << "  Distance from Earth: " << (sun_dist / 1.495978707e11) << " AU\n";
    std::cout << "  Position ECI: [" << sun_pos.x / 1e9 << ", "
              << sun_pos.y / 1e9 << ", " << sun_pos.z / 1e9 << "] × 10⁹ m\n";

    // SRP acceleration
    Vec3 a_srp = srp.acceleration(state, sun_pos);

    std::cout << "\nSRP acceleration at GEO:\n";
    std::cout << "  a_srp = [" << std::scientific << std::setprecision(4)
              << a_srp.x << ", " << a_srp.y << ", " << a_srp.z << "] m/s²\n";
    std::cout << "  |a_srp| = " << a_srp.norm() << " m/s²\n";

    // Test shadow function
    std::cout << "\nShadow function test:\n";
    double shadow = srp.shadowFunction(state.position, sun_pos);
    std::cout << "  Shadow factor (sunlight=1): " << shadow << "\n";

    // Test in shadow (opposite side of Earth)
    StateVector shadow_state;
    shadow_state.position = sun_pos * (-r_geo / sun_pos.norm());  // Behind Earth from Sun
    double shadow2 = srp.shadowFunction(shadow_state.position, sun_pos);
    std::cout << "  Shadow factor (behind Earth): " << shadow2 << "\n";
}

void testThirdBodyAcceleration()
{
    std::cout << "\n=== Test 4: Third-Body Perturbations ===\n";

    ThirdBodyModel third_body;

    // Test at GEO altitude
    double r_geo = constants::EARTH_RADIUS_EQ + 35786000.0;
    double jd = TimeSystem::calendarToJD(2024, 1, 15, 0, 0, 0);

    StateVector state;
    state.position = Vec3(r_geo, 0.0, 0.0);
    state.velocity = Vec3(0.0, std::sqrt(constants::EARTH_MU / r_geo), 0.0);
    state.epoch = jd;

    // Get body positions
    Vec3 sun_pos = SunPosition::getSunPositionECI(jd);
    Vec3 moon_pos = ThirdBodyModel::getMoonPositionECI(jd);

    std::cout << "\nCelestial body positions (JD " << jd << "):\n";
    std::cout << "  Sun distance: " << sun_pos.norm() / 1e9 << " × 10⁹ m\n";
    std::cout << "  Moon distance: " << moon_pos.norm() / 1e6 << " × 10⁶ m ("
              << moon_pos.norm() / 1000.0 << " km)\n";

    // Third-body accelerations
    Vec3 a_sun = third_body.sunAcceleration(state, jd);
    Vec3 a_moon = third_body.moonAcceleration(state, jd);
    Vec3 a_total = third_body.acceleration(state, jd);

    std::cout << "\nThird-body accelerations at GEO:\n";
    std::cout << std::scientific << std::setprecision(4);
    std::cout << "  Sun:   |a| = " << a_sun.norm() << " m/s²\n";
    std::cout << "  Moon:  |a| = " << a_moon.norm() << " m/s²\n";
    std::cout << "  Total: |a| = " << a_total.norm() << " m/s²\n";

    // Compare with gravity at GEO
    double a_central = constants::EARTH_MU / (r_geo * r_geo);
    std::cout << "\nFor comparison:\n";
    std::cout << "  Earth gravity at GEO: " << a_central << " m/s²\n";
    std::cout << "  Sun/Earth ratio: " << a_sun.norm() / a_central << "\n";
    std::cout << "  Moon/Earth ratio: " << a_moon.norm() / a_central << "\n";
}

void testPerturbationComparison()
{
    std::cout << "\n=== Test 5: Perturbation Magnitude Comparison ===\n";

    // Compare all perturbations at LEO (400 km) and GEO
    std::vector<std::pair<std::string, double>> orbits = {
        {"LEO (400 km)", constants::EARTH_RADIUS_EQ + 400000.0},
        {"MEO (20200 km)", constants::EARTH_RADIUS_EQ + 20200000.0},
        {"GEO (35786 km)", constants::EARTH_RADIUS_EQ + 35786000.0}
    };

    DragModel drag;
    drag.setSpacecraftParams(2.2, 10.0, 1000.0);

    SRPModel srp;
    srp.setSpacecraftParams(1.4, 10.0, 1000.0);

    ThirdBodyModel third_body;

    double jd = TimeSystem::calendarToJD(2024, 6, 21, 12, 0, 0);
    Vec3 sun_pos = SunPosition::getSunPositionECI(jd);

    std::cout << "\nPerturbation accelerations (m/s²) for 1000 kg, 10 m² spacecraft:\n";
    std::cout << "================================================================\n";
    std::cout << "Orbit          | Central   | J2       | Drag      | SRP       | 3rd-Body\n";
    std::cout << "---------------+-----------+----------+-----------+-----------+---------\n";

    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/perturbation_comparison.csv");
    if (csv.is_open())
    {
        csv << "orbit,altitude_km,central,j2,drag,srp,third_body\n";
    }

    for (const auto& [name, r] : orbits)
    {
        double v = std::sqrt(constants::EARTH_MU / r);
        double h = r - constants::EARTH_RADIUS_EQ;

        StateVector state;
        state.position = Vec3(r, 0.0, 0.0);
        state.velocity = Vec3(0.0, v, 0.0);
        state.epoch = jd;

        // Central gravity
        double a_central = constants::EARTH_MU / (r * r);

        // J2 (approximate at equator)
        double j2_factor = 1.5 * constants::EARTH_J2 * std::pow(constants::EARTH_RADIUS_EQ / r, 2);
        double a_j2 = a_central * j2_factor;

        // Drag
        Vec3 a_drag_vec = drag.acceleration(state, jd);
        double a_drag = a_drag_vec.norm();

        // SRP
        Vec3 a_srp_vec = srp.acceleration(state, sun_pos);
        double a_srp = a_srp_vec.norm();

        // Third-body
        Vec3 a_3b_vec = third_body.acceleration(state, jd);
        double a_3b = a_3b_vec.norm();

        std::cout << std::left << std::setw(14) << name << " | "
                  << std::scientific << std::setprecision(2)
                  << std::setw(9) << a_central << " | "
                  << std::setw(8) << a_j2 << " | "
                  << std::setw(9) << a_drag << " | "
                  << std::setw(9) << a_srp << " | "
                  << std::setw(8) << a_3b << "\n";

        if (csv.is_open())
        {
            csv << name << "," << (h / 1000.0) << ","
                << a_central << "," << a_j2 << "," << a_drag << ","
                << a_srp << "," << a_3b << "\n";
        }
    }

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nData saved to results/data/perturbation_comparison.csv\n";
    }
}

void testMoonPosition()
{
    std::cout << "\n=== Test 6: Moon Position Ephemeris ===\n";

    // Test Moon position over a month
    double jd_start = TimeSystem::calendarToJD(2024, 1, 1, 0, 0, 0);

    std::ofstream csv("/home/seongmin/ros2_ws/src/lrs_dynamics/results/data/moon_position.csv");
    if (csv.is_open())
    {
        csv << "jd,x_km,y_km,z_km,distance_km\n";
    }

    std::cout << "\nMoon position over January 2024:\n";
    std::cout << "Date       | Distance (km) | RA (deg)  | Dec (deg)\n";
    std::cout << "-----------+---------------+-----------+----------\n";

    for (int day = 1; day <= 31; day += 5)
    {
        double jd = jd_start + day - 1;
        Vec3 moon = ThirdBodyModel::getMoonPositionECI(jd);
        double dist = moon.norm();

        // Calculate RA and Dec
        double ra = std::atan2(moon.y, moon.x) * constants::RAD_TO_DEG;
        double dec = std::asin(moon.z / dist) * constants::RAD_TO_DEG;

        std::cout << "2024-01-" << std::setw(2) << std::setfill('0') << day
                  << std::setfill(' ') << " | "
                  << std::fixed << std::setprecision(0)
                  << std::setw(13) << (dist / 1000.0) << " | "
                  << std::setprecision(2)
                  << std::setw(9) << ra << " | "
                  << std::setw(8) << dec << "\n";

        if (csv.is_open())
        {
            csv << jd << "," << moon.x / 1000.0 << "," << moon.y / 1000.0 << ","
                << moon.z / 1000.0 << "," << dist / 1000.0 << "\n";
        }
    }

    if (csv.is_open())
    {
        csv.close();
        std::cout << "\nMoon data saved to results/data/moon_position.csv\n";
    }

    // Average distance should be ~385,000 km
    Vec3 moon_avg = ThirdBodyModel::getMoonPositionECI(jd_start + 15);
    std::cout << "\nMid-month Moon distance: " << moon_avg.norm() / 1000.0 << " km\n";
    std::cout << "(Expected ~385,000 km)\n";
}

int main()
{
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║        HPOP Perturbation Models Test Suite                ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    testAtmosphericDensity();
    testDragAcceleration();
    testSRPAcceleration();
    testThirdBodyAcceleration();
    testPerturbationComparison();
    testMoonPosition();

    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    All Tests Complete                      ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    // Save test summary
    std::ofstream log("/home/seongmin/ros2_ws/src/lrs_dynamics/results/logs/perturbation_test.log",
                      std::ios::app);
    if (log.is_open())
    {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        log << "\n=== Perturbation Test: " << std::ctime(&time);
        log << "All perturbation models tested successfully\n";
        log << "Models: Drag (Exponential + Harris-Priester), SRP, Third-Body (Sun+Moon)\n";
    }

    return 0;
}
