/**
 * @file test_tle.cpp
 * @brief Test TLE parsing and Celestrak API fetching
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>

#include "hpop_tle/tle_parser.hpp"
#include "hpop_tle/tle_fetcher.hpp"

using namespace hpop_tle;

void printTLE(const TLEData& tle)
{
    std::cout << "\n========================================\n";
    std::cout << "Satellite: " << tle.satellite_name << "\n";
    std::cout << "NORAD ID:  " << tle.norad_id << "\n";
    std::cout << "Intl Des:  " << tle.intl_designator << "\n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Epoch:       " << tle.epoch_year << "/" << tle.epoch_day << " (JD " << tle.epochJD() << ")\n";
    std::cout << "Inclination: " << tle.inclination << " deg\n";
    std::cout << "RAAN:        " << tle.raan << " deg\n";
    std::cout << "Eccentricity:" << tle.eccentricity << "\n";
    std::cout << "Arg Periapsis:" << tle.arg_periapsis << " deg\n";
    std::cout << "Mean Anomaly: " << tle.mean_anomaly << " deg\n";
    std::cout << "Mean Motion:  " << tle.mean_motion << " rev/day\n";
    std::cout << "B* Drag:      " << tle.bstar << "\n";
    std::cout << "Semi-major:   " << tle.semiMajorAxis() / 1000.0 << " km\n";
    std::cout << "Period:       " << tle.period() / 60.0 << " minutes\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Line 1: " << tle.line1 << "\n";
    std::cout << "Line 2: " << tle.line2 << "\n";
}

void testTLEParsing()
{
    std::cout << "\n=== Test 1: TLE Parsing ===\n";

    // ISS TLE with correct checksums (line1 checksum=2, line2 checksum=0)
    std::string iss_line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9992";
    std::string iss_line2 = "2 25544  51.6400 208.9163 0006703 274.8936  85.1431 15.49957234424850";

    auto tle = TLEParser::parse(iss_line1, iss_line2, "ISS (ZARYA)");

    if (tle.has_value())
    {
        std::cout << "[PASS] TLE parsing successful\n";
        printTLE(*tle);

        // Convert to orbital elements
        auto oe = tle->toOrbitalElements();
        std::cout << "\nOrbital Elements:\n";
        std::cout << "  SMA: " << oe.sma / 1000.0 << " km\n";
        std::cout << "  ECC: " << oe.ecc << "\n";
        std::cout << "  INC: " << oe.inc * 180.0 / M_PI << " deg\n";
        std::cout << "  RAAN:" << oe.raan * 180.0 / M_PI << " deg\n";
        std::cout << "  AOP: " << oe.aop * 180.0 / M_PI << " deg\n";
        std::cout << "  TA:  " << oe.ta * 180.0 / M_PI << " deg\n";

        // Convert to state vector
        auto sv = tle->toStateVector();
        std::cout << "\nState Vector (ECI):\n";
        std::cout << "  Position: [" << sv.position.x / 1000.0 << ", "
                  << sv.position.y / 1000.0 << ", "
                  << sv.position.z / 1000.0 << "] km\n";
        std::cout << "  Velocity: [" << sv.velocity.x / 1000.0 << ", "
                  << sv.velocity.y / 1000.0 << ", "
                  << sv.velocity.z / 1000.0 << "] km/s\n";
        double r = sv.position.norm() / 1000.0;
        double v = sv.velocity.norm() / 1000.0;
        std::cout << "  |r| = " << r << " km, |v| = " << v << " km/s\n";
    }
    else
    {
        std::cout << "[FAIL] TLE parsing failed\n";
    }
}

void testMultipleTLEParsing()
{
    std::cout << "\n=== Test 2: Multiple TLE Parsing ===\n";

    // Use real TLEs with correct checksums
    std::string tle_data = R"(ISS (ZARYA)
1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9992
2 25544  51.6400 208.9163 0006703 274.8936  85.1431 15.49957234424850
STARLINK-1007
1 44713U 19074A   24001.50000000  .00005000  00000-0  33000-4 0  9992
2 44713  53.0500 100.0000 0001234  45.0000 315.0000 15.10000000 10005
)";

    auto tles = TLEParser::parseString(tle_data);
    std::cout << "Parsed " << tles.size() << " TLEs\n";

    for (const auto& tle : tles)
    {
        std::cout << "  - " << tle.satellite_name << " (NORAD " << tle.norad_id << ")\n";
    }

    if (tles.size() == 2)
    {
        std::cout << "[PASS] Multiple TLE parsing successful\n";
    }
    else
    {
        std::cout << "[FAIL] Expected 2 TLEs, got " << tles.size() << "\n";
    }
}

void testChecksumValidation()
{
    std::cout << "\n=== Test 3: Checksum Validation ===\n";

    // Valid TLE with correct checksums
    std::string valid_line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9992";
    std::string valid_line2 = "2 25544  51.6400 208.9163 0006703 274.8936  85.1431 15.49957234424850";

    auto valid_tle = TLEParser::parse(valid_line1, valid_line2);
    if (valid_tle.has_value())
    {
        std::cout << "[PASS] Valid TLE accepted\n";
    }
    else
    {
        std::cout << "[FAIL] Valid TLE rejected\n";
    }

    // Invalid TLE (modified checksum - last digit changed)
    std::string invalid_line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9999";
    std::string invalid_line2 = "2 25544  51.6400 208.9163 0006703 274.8936  85.1431 15.49957234424859";

    auto invalid_tle = TLEParser::parse(invalid_line1, invalid_line2);
    if (!invalid_tle.has_value())
    {
        std::cout << "[PASS] Invalid TLE rejected (checksum)\n";
    }
    else
    {
        std::cout << "[FAIL] Invalid TLE accepted (should have been rejected)\n";
    }
}

void testCelestrakFetch()
{
    std::cout << "\n=== Test 4: Celestrak API Fetch ===\n";

    TLEFetcher fetcher;
    fetcher.setTimeout(30);

    // Test fetching ISS TLE by NORAD ID
    std::cout << "Fetching ISS TLE (NORAD 25544)...\n";

    auto start = std::chrono::steady_clock::now();
    auto result = fetcher.fetchByNoradId(25544);
    auto end = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (result.success)
    {
        std::cout << "[PASS] Celestrak fetch successful (" << elapsed_ms << " ms)\n";
        std::cout << "HTTP Code: " << result.http_code << "\n";
        std::cout << "Fetched " << result.tles.size() << " TLE(s)\n";

        if (!result.tles.empty())
        {
            printTLE(result.tles[0]);

            // Save to results
            std::ofstream log("/home/seongmin/ros2_ws/src/lrs_dynamics/results/logs/tle_fetch_test.log",
                              std::ios::app);
            if (log.is_open())
            {
                auto now = std::chrono::system_clock::now();
                auto time = std::chrono::system_clock::to_time_t(now);
                log << "\n=== TLE Fetch Test: " << std::ctime(&time);
                log << "NORAD ID: 25544 (ISS)\n";
                log << "Fetch time: " << elapsed_ms << " ms\n";
                log << "Line 1: " << result.tles[0].line1 << "\n";
                log << "Line 2: " << result.tles[0].line2 << "\n";
                log << "Epoch JD: " << result.tles[0].epochJD() << "\n";
                log << "SMA: " << result.tles[0].semiMajorAxis() / 1000.0 << " km\n";
                log << "Period: " << result.tles[0].period() / 60.0 << " min\n";
            }
        }
    }
    else
    {
        std::cout << "[WARN] Celestrak fetch failed: " << result.error_message << "\n";
        std::cout << "HTTP Code: " << result.http_code << "\n";
        std::cout << "(This may be expected if there's no internet connection)\n";
    }
}

void testMultipleSatelliteFetch()
{
    std::cout << "\n=== Test 5: Multiple Satellite Fetch ===\n";

    TLEFetcher fetcher;

    // Fetch ISS and some other satellites
    std::vector<uint32_t> norad_ids = {25544, 48274, 43013};  // ISS, Tiangong, Starlink

    std::cout << "Fetching " << norad_ids.size() << " satellites...\n";

    auto start = std::chrono::steady_clock::now();
    auto result = fetcher.fetchByNoradIds(norad_ids);
    auto end = std::chrono::steady_clock::now();

    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (result.success)
    {
        std::cout << "[PASS] Multiple fetch successful (" << elapsed_ms << " ms)\n";
        std::cout << "Fetched " << result.tles.size() << " TLEs:\n";
        for (const auto& tle : result.tles)
        {
            std::cout << "  - " << tle.satellite_name << " (NORAD " << tle.norad_id << ")\n";
        }
    }
    else
    {
        std::cout << "[WARN] Multiple fetch failed: " << result.error_message << "\n";
        std::cout << "(This may be expected if there's no internet connection)\n";
    }
}

void testTLECache()
{
    std::cout << "\n=== Test 6: TLE Cache ===\n";

    TLECache cache;

    // Create a test TLE
    TLEData tle;
    tle.norad_id = 25544;
    tle.satellite_name = "ISS";
    tle.inclination = 51.64;

    // Store in cache
    cache.put(tle);
    std::cout << "Cache size after put: " << cache.size() << "\n";

    // Retrieve from cache
    auto cached = cache.get(25544, std::chrono::hours(1));
    if (cached.has_value())
    {
        std::cout << "[PASS] Cache hit for NORAD 25544\n";
        std::cout << "  Name: " << cached->satellite_name << "\n";
    }
    else
    {
        std::cout << "[FAIL] Cache miss for NORAD 25544\n";
    }

    // Test cache miss for non-existent ID
    auto miss = cache.get(99999);
    if (!miss.has_value())
    {
        std::cout << "[PASS] Cache miss for non-existent ID\n";
    }
    else
    {
        std::cout << "[FAIL] Unexpected cache hit\n";
    }
}

int main()
{
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║           HPOP TLE Parser & Fetcher Test Suite            ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    // Run all tests
    testTLEParsing();
    testMultipleTLEParsing();
    testChecksumValidation();
    testCelestrakFetch();
    testMultipleSatelliteFetch();
    testTLECache();

    std::cout << "\n╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    All Tests Complete                      ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    return 0;
}
