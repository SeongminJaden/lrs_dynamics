/**
 * @file test_export.cpp
 * @brief Test for HPOP export functionality
 */

#include <iostream>
#include <cmath>
#include <hpop_export/csv_exporter.hpp>
#include <hpop_export/oem_exporter.hpp>

using namespace hpop_export;

// Constants
const double GM_EARTH = 398600.4418e9;  // [m^3/s^2]
const double EARTH_RADIUS = 6378137.0;  // [m]

/**
 * @brief Generate test ephemeris data for a circular orbit
 */
std::vector<EphemerisPoint> generateTestEphemeris(
    double altitude_km, double inclination_deg, int num_points)
{
    std::vector<EphemerisPoint> ephemeris;

    double sma = EARTH_RADIUS + altitude_km * 1000.0;  // [m]
    double inc = inclination_deg * M_PI / 180.0;       // [rad]
    double period = 2.0 * M_PI * std::sqrt(sma * sma * sma / GM_EARTH);  // [s]

    // Starting Julian Date (J2000.0 + 1 day)
    double jd_start = 2451545.0 + 1.0;

    std::cout << "Generating orbit ephemeris:" << std::endl;
    std::cout << "  Altitude: " << altitude_km << " km" << std::endl;
    std::cout << "  SMA: " << sma / 1000.0 << " km" << std::endl;
    std::cout << "  Period: " << period / 60.0 << " minutes" << std::endl;
    std::cout << "  Inclination: " << inclination_deg << " deg" << std::endl;

    double mean_motion = std::sqrt(GM_EARTH / (sma * sma * sma));  // [rad/s]
    double velocity = std::sqrt(GM_EARTH / sma);  // [m/s]

    for (int i = 0; i < num_points; i++)
    {
        double t = i * period / num_points;  // Time from epoch [s]
        double true_anomaly = mean_motion * t;  // For circular orbit

        EphemerisPoint point;
        point.epoch_jd = jd_start + t / 86400.0;

        // Position in orbital plane
        double r = sma;  // For circular orbit
        double x_orb = r * std::cos(true_anomaly);
        double y_orb = r * std::sin(true_anomaly);

        // Rotate by inclination (simplified - RAAN = 0, AoP = 0)
        point.x = x_orb;
        point.y = y_orb * std::cos(inc);
        point.z = y_orb * std::sin(inc);

        // Velocity in orbital plane
        double vx_orb = -velocity * std::sin(true_anomaly);
        double vy_orb = velocity * std::cos(true_anomaly);

        point.vx = vx_orb;
        point.vy = vy_orb * std::cos(inc);
        point.vz = vy_orb * std::sin(inc);

        // Orbital elements
        point.semi_major_axis = sma;
        point.eccentricity = 0.0;
        point.inclination = inc;
        point.raan = 0.0;
        point.arg_periapsis = 0.0;
        point.true_anomaly = true_anomaly;

        point.satellite_id = "TEST_SAT";
        point.norad_id = 99999;

        ephemeris.push_back(point);
    }

    return ephemeris;
}

/**
 * @brief Test CSV export
 */
bool testCsvExport()
{
    std::cout << "\n=== Testing CSV Export ===" << std::endl;

    auto ephemeris = generateTestEphemeris(400.0, 51.6, 100);  // ISS-like orbit

    CsvExporter exporter;
    CsvExportOptions options;
    options.include_header = true;
    options.include_elements = true;
    options.date_format = "ISO8601";

    // Export to file
    std::string filename = "/tmp/hpop_test_export.csv";
    bool success = exporter.exportToFile(filename, ephemeris, options);

    if (success)
    {
        std::cout << "CSV export successful: " << filename << std::endl;
        std::cout << "Exported " << ephemeris.size() << " records" << std::endl;

        // Print first few lines
        std::cout << "\nFirst 3 records (preview):" << std::endl;
        options.include_elements = false;  // Shorter preview
        std::string preview;
        int count = 0;
        for (const auto& point : ephemeris)
        {
            if (count++ >= 3) break;
            std::cout << "  JD=" << std::fixed << std::setprecision(6) << point.epoch_jd
                     << " X=" << std::setprecision(1) << point.x / 1000.0 << "km"
                     << " Y=" << point.y / 1000.0 << "km"
                     << " Z=" << point.z / 1000.0 << "km" << std::endl;
        }
    }
    else
    {
        std::cout << "CSV export failed: " << exporter.getLastError() << std::endl;
    }

    return success;
}

/**
 * @brief Test OEM export
 */
bool testOemExport()
{
    std::cout << "\n=== Testing OEM Export ===" << std::endl;

    auto csv_ephemeris = generateTestEphemeris(550.0, 53.0, 50);  // Starlink-like

    // Convert to OEM format
    std::vector<OemState> states;
    for (const auto& point : csv_ephemeris)
    {
        OemState state;
        state.epoch_jd = point.epoch_jd;
        state.x = point.x / 1000.0;   // m -> km
        state.y = point.y / 1000.0;
        state.z = point.z / 1000.0;
        state.vx = point.vx / 1000.0; // m/s -> km/s
        state.vy = point.vy / 1000.0;
        state.vz = point.vz / 1000.0;
        states.push_back(state);
    }

    OemExporter exporter;
    OemExportOptions options;
    options.originator = "HPOP_TEST";
    options.object_name = "STARLINK-TEST";
    options.object_id = "2023-001A";
    options.ref_frame = "EME2000";

    // Export to file
    std::string filename = "/tmp/hpop_test_export.oem";
    bool success = exporter.exportToFile(filename, states, options);

    if (success)
    {
        std::cout << "OEM export successful: " << filename << std::endl;
        std::cout << "Exported " << states.size() << " records" << std::endl;

        // Print OEM header preview
        std::cout << "\nOEM content preview:" << std::endl;
        std::string content = exporter.exportToString(states, options);
        size_t preview_len = std::min(content.size(), size_t(500));
        std::cout << content.substr(0, preview_len) << "..." << std::endl;
    }
    else
    {
        std::cout << "OEM export failed: " << exporter.getLastError() << std::endl;
    }

    return success;
}

/**
 * @brief Test multiple format options
 */
void testFormatOptions()
{
    std::cout << "\n=== Testing Format Options ===" << std::endl;

    auto ephemeris = generateTestEphemeris(700.0, 98.0, 10);  // Sun-sync

    CsvExporter csv_exporter;

    // Test different date formats
    std::cout << "\nDate format comparison:" << std::endl;

    CsvExportOptions options_jd;
    options_jd.date_format = "JD";
    options_jd.include_elements = false;
    options_jd.include_velocity = false;
    std::string jd_output = csv_exporter.exportToString({ephemeris[0]}, options_jd);
    std::cout << "  JD format: " << jd_output.substr(0, jd_output.find('\n', 50)) << std::endl;

    CsvExportOptions options_iso;
    options_iso.date_format = "ISO8601";
    options_iso.include_elements = false;
    options_iso.include_velocity = false;
    std::string iso_output = csv_exporter.exportToString({ephemeris[0]}, options_iso);
    std::cout << "  ISO8601: " << iso_output.substr(0, iso_output.find('\n', 50)) << std::endl;

    CsvExportOptions options_mjd;
    options_mjd.date_format = "MJD";
    options_mjd.include_elements = false;
    options_mjd.include_velocity = false;
    std::string mjd_output = csv_exporter.exportToString({ephemeris[0]}, options_mjd);
    std::cout << "  MJD format: " << mjd_output.substr(0, mjd_output.find('\n', 50)) << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  HPOP Export Module Test" << std::endl;
    std::cout << "========================================" << std::endl;

    bool all_passed = true;

    all_passed &= testCsvExport();
    all_passed &= testOemExport();
    testFormatOptions();

    std::cout << "\n========================================" << std::endl;
    if (all_passed)
    {
        std::cout << "  All tests PASSED!" << std::endl;
    }
    else
    {
        std::cout << "  Some tests FAILED!" << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return all_passed ? 0 : 1;
}
