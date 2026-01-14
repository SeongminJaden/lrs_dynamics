/**
 * GGM05C Gravity Model Sample Data Extractor
 *
 * Outputs gravity acceleration data at various altitudes and latitudes
 * for analysis and visualization.
 */

#include "gazebo_leo_gravity/ggm_model.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <vector>

using namespace gazebo_leo_gravity;

// Constants
const double R_EARTH = 6378136.3;  // Earth equatorial radius [m]
const double GM = 3.986004415e14;  // [m^3/s^2]
const double DEG2RAD = M_PI / 180.0;

// Convert geodetic to ECEF
ignition::math::Vector3d geodeticToECEF(double lat_deg, double lon_deg, double alt_m)
{
    double lat = lat_deg * DEG2RAD;
    double lon = lon_deg * DEG2RAD;
    double r = R_EARTH + alt_m;

    return ignition::math::Vector3d(
        r * std::cos(lat) * std::cos(lon),
        r * std::cos(lat) * std::sin(lon),
        r * std::sin(lat)
    );
}

int main(int argc, char** argv)
{
    std::string ggm_file = "/home/seongmin/ros2_ws/src/lrs_dynamics/data/GGM05C.gfc";
    std::string output_file = "/home/seongmin/ros2_ws/src/lrs_dynamics/data/ggm05c_sample_data.csv";

    int nmax = 70;  // Degree/order for high precision

    if (argc > 1) nmax = std::atoi(argv[1]);

    std::cout << "=== GGM05C Sample Data Extractor ===" << std::endl;
    std::cout << "Loading GGM05C model (nmax=" << nmax << ")..." << std::endl;

    GGMModel model;
    if (!model.load(ggm_file, nmax))
    {
        std::cerr << "Failed to load GGM05C file!" << std::endl;
        return 1;
    }

    std::cout << "Loaded " << model.coeffCount() << " coefficients" << std::endl;
    std::cout << "GM = " << model.GM() << " m^3/s^2" << std::endl;
    std::cout << "a  = " << model.radius() << " m" << std::endl;

    // Open output file
    std::ofstream fout(output_file);
    if (!fout.is_open())
    {
        std::cerr << "Failed to open output file!" << std::endl;
        return 1;
    }

    // CSV Header
    fout << "lat_deg,lon_deg,alt_km,x_ecef_m,y_ecef_m,z_ecef_m,"
         << "ax_m_s2,ay_m_s2,az_m_s2,g_total_m_s2,"
         << "g_point_mass_m_s2,delta_g_m_s2,delta_g_mGal" << std::endl;

    std::cout << "\nGenerating sample data..." << std::endl;

    // ===========================================
    // 1. Altitude variation at equator (0°, 0°)
    // ===========================================
    std::cout << "1. Altitude sweep at equator..." << std::endl;
    for (double alt_km = 200; alt_km <= 2000; alt_km += 50)
    {
        auto pos = geodeticToECEF(0, 0, alt_km * 1000);
        auto acc = model.acceleration(pos);
        double g_total = acc.Length();
        double r = pos.Length();
        double g_point = GM / (r * r);
        double delta_g = g_total - g_point;
        double delta_g_mGal = delta_g * 1e5;  // Convert to mGal

        fout << std::fixed << std::setprecision(6)
             << 0 << "," << 0 << "," << alt_km << ","
             << pos.X() << "," << pos.Y() << "," << pos.Z() << ","
             << acc.X() << "," << acc.Y() << "," << acc.Z() << ","
             << g_total << "," << g_point << "," << delta_g << "," << delta_g_mGal
             << std::endl;
    }

    // ===========================================
    // 2. Latitude variation at 400km altitude
    // ===========================================
    std::cout << "2. Latitude sweep at 400km..." << std::endl;
    for (double lat = -90; lat <= 90; lat += 5)
    {
        auto pos = geodeticToECEF(lat, 0, 400000);
        auto acc = model.acceleration(pos);
        double g_total = acc.Length();
        double r = pos.Length();
        double g_point = GM / (r * r);
        double delta_g = g_total - g_point;
        double delta_g_mGal = delta_g * 1e5;

        fout << std::fixed << std::setprecision(6)
             << lat << "," << 0 << "," << 400 << ","
             << pos.X() << "," << pos.Y() << "," << pos.Z() << ","
             << acc.X() << "," << acc.Y() << "," << acc.Z() << ","
             << g_total << "," << g_point << "," << delta_g << "," << delta_g_mGal
             << std::endl;
    }

    // ===========================================
    // 3. Global grid at ISS altitude (400km)
    // ===========================================
    std::cout << "3. Global grid at 400km..." << std::endl;
    for (double lat = -90; lat <= 90; lat += 10)
    {
        for (double lon = -180; lon < 180; lon += 10)
        {
            auto pos = geodeticToECEF(lat, lon, 400000);
            auto acc = model.acceleration(pos);
            double g_total = acc.Length();
            double r = pos.Length();
            double g_point = GM / (r * r);
            double delta_g = g_total - g_point;
            double delta_g_mGal = delta_g * 1e5;

            fout << std::fixed << std::setprecision(6)
                 << lat << "," << lon << "," << 400 << ","
                 << pos.X() << "," << pos.Y() << "," << pos.Z() << ","
                 << acc.X() << "," << acc.Y() << "," << acc.Z() << ","
                 << g_total << "," << g_point << "," << delta_g << "," << delta_g_mGal
                 << std::endl;
        }
    }

    // ===========================================
    // 4. LEO orbit simulation points
    // ===========================================
    std::cout << "4. LEO orbit track (ISS-like)..." << std::endl;
    double inc = 51.6 * DEG2RAD;  // ISS inclination
    double alt = 420000;  // 420 km
    for (double angle = 0; angle < 360; angle += 1)
    {
        double theta = angle * DEG2RAD;
        double lat = std::asin(std::sin(inc) * std::sin(theta)) / DEG2RAD;
        double lon = std::atan2(std::cos(inc) * std::sin(theta), std::cos(theta)) / DEG2RAD;

        auto pos = geodeticToECEF(lat, lon, alt);
        auto acc = model.acceleration(pos);
        double g_total = acc.Length();
        double r = pos.Length();
        double g_point = GM / (r * r);
        double delta_g = g_total - g_point;
        double delta_g_mGal = delta_g * 1e5;

        fout << std::fixed << std::setprecision(6)
             << lat << "," << lon << "," << alt/1000.0 << ","
             << pos.X() << "," << pos.Y() << "," << pos.Z() << ","
             << acc.X() << "," << acc.Y() << "," << acc.Z() << ","
             << g_total << "," << g_point << "," << delta_g << "," << delta_g_mGal
             << std::endl;
    }

    fout.close();

    std::cout << "\n=== Sample Data Summary ===" << std::endl;
    std::cout << "Output file: " << output_file << std::endl;

    // Print some key statistics
    std::cout << "\n=== Key Statistics ===" << std::endl;

    // Point mass vs GGM05C comparison at 400km equator
    auto pos_eq = geodeticToECEF(0, 0, 400000);
    auto acc_eq = model.acceleration(pos_eq);
    double g_ggm = acc_eq.Length();
    double r_eq = pos_eq.Length();
    double g_point = GM / (r_eq * r_eq);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nAt 400km altitude, equator:" << std::endl;
    std::cout << "  Point mass gravity: " << g_point << " m/s^2" << std::endl;
    std::cout << "  GGM05C gravity:     " << g_ggm << " m/s^2" << std::endl;
    std::cout << "  Difference:         " << (g_ggm - g_point) * 1e5 << " mGal" << std::endl;
    std::cout << "  Relative error:     " << std::abs(g_ggm - g_point) / g_point * 1e6 << " ppm" << std::endl;

    // Comparison at pole
    auto pos_pole = geodeticToECEF(90, 0, 400000);
    auto acc_pole = model.acceleration(pos_pole);
    double g_pole = acc_pole.Length();
    double r_pole = pos_pole.Length();
    double g_point_pole = GM / (r_pole * r_pole);

    std::cout << "\nAt 400km altitude, north pole:" << std::endl;
    std::cout << "  Point mass gravity: " << g_point_pole << " m/s^2" << std::endl;
    std::cout << "  GGM05C gravity:     " << g_pole << " m/s^2" << std::endl;
    std::cout << "  Difference:         " << (g_pole - g_point_pole) * 1e5 << " mGal" << std::endl;

    std::cout << "\nEquator vs Pole difference: " << (g_pole - g_ggm) * 1e5 << " mGal" << std::endl;

    return 0;
}
