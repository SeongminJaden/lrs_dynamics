#pragma once

#include <string>
#include <vector>
#include <optional>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdexcept>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/time_system.hpp"

namespace hpop_tle
{

/// Two-Line Element Set data structure
struct TLEData
{
    // Line 0 (optional name)
    std::string satellite_name;

    // Line 1 fields
    uint32_t norad_id{0};
    char classification{'U'};
    std::string intl_designator;
    int epoch_year{0};
    double epoch_day{0.0};
    double mean_motion_dot{0.0};      // First derivative / 2
    double mean_motion_ddot{0.0};     // Second derivative / 6
    double bstar{0.0};
    uint8_t ephemeris_type{0};
    uint32_t element_set_number{0};

    // Line 2 fields
    double inclination{0.0};          // [deg]
    double raan{0.0};                 // [deg]
    double eccentricity{0.0};         // Decimal point assumed
    double arg_periapsis{0.0};        // [deg]
    double mean_anomaly{0.0};         // [deg]
    double mean_motion{0.0};          // [rev/day]
    uint32_t revolution_number{0};

    // Raw lines
    std::string line1;
    std::string line2;

    // Derived quantities
    double epochJD() const
    {
        return hpop_core::TimeSystem::tleEpochToJD(epoch_year, epoch_day);
    }

    double semiMajorAxis() const
    {
        // n = sqrt(mu/a^3), n in rad/s
        double n = mean_motion * hpop_core::constants::TWO_PI / hpop_core::constants::SECONDS_PER_DAY;
        return std::cbrt(hpop_core::constants::EARTH_MU / (n * n));
    }

    double period() const
    {
        return hpop_core::constants::SECONDS_PER_DAY / mean_motion;
    }

    // Convert to orbital elements (radians)
    hpop_core::OrbitalElements toOrbitalElements() const
    {
        hpop_core::OrbitalElements oe;
        oe.sma = semiMajorAxis();
        oe.ecc = eccentricity;
        oe.inc = inclination * hpop_core::constants::DEG_TO_RAD;
        oe.raan = raan * hpop_core::constants::DEG_TO_RAD;
        oe.aop = arg_periapsis * hpop_core::constants::DEG_TO_RAD;

        // Convert mean anomaly to true anomaly
        double M = mean_anomaly * hpop_core::constants::DEG_TO_RAD;
        double E = meanToEccentricAnomaly(M, eccentricity);
        oe.ta = eccentricToTrueAnomaly(E, eccentricity);

        oe.epoch = epochJD();
        return oe;
    }

    // Convert to state vector
    hpop_core::StateVector toStateVector() const
    {
        return toOrbitalElements().toStateVector();
    }

private:
    // Kepler's equation solver: M = E - e*sin(E)
    static double meanToEccentricAnomaly(double M, double e, double tol = 1e-12)
    {
        double E = M;  // Initial guess
        for (int i = 0; i < 50; ++i)
        {
            double dE = (M - E + e * std::sin(E)) / (1.0 - e * std::cos(E));
            E += dE;
            if (std::abs(dE) < tol) break;
        }
        return E;
    }

    // Convert eccentric anomaly to true anomaly
    static double eccentricToTrueAnomaly(double E, double e)
    {
        double cos_E = std::cos(E);
        double sin_E = std::sin(E);
        double cos_nu = (cos_E - e) / (1.0 - e * cos_E);
        double sin_nu = std::sqrt(1.0 - e * e) * sin_E / (1.0 - e * cos_E);
        return std::atan2(sin_nu, cos_nu);
    }
};

/// TLE Parser class
class TLEParser
{
public:
    /// Parse TLE from two lines (with optional name line)
    static std::optional<TLEData> parse(const std::string& line1,
                                         const std::string& line2,
                                         const std::string& name = "")
    {
        TLEData tle;
        tle.satellite_name = name;
        tle.line1 = line1;
        tle.line2 = line2;

        try
        {
            // Validate line lengths
            if (line1.length() < 69 || line2.length() < 69)
                return std::nullopt;

            // Validate line numbers
            if (line1[0] != '1' || line2[0] != '2')
                return std::nullopt;

            // Parse Line 1
            tle.norad_id = std::stoul(line1.substr(2, 5));
            tle.classification = line1[7];
            tle.intl_designator = trim(line1.substr(9, 8));

            tle.epoch_year = std::stoi(line1.substr(18, 2));
            tle.epoch_day = std::stod(line1.substr(20, 12));

            tle.mean_motion_dot = std::stod(line1.substr(33, 10));

            // Parse mean_motion_ddot (scientific notation without 'E')
            std::string ddot_str = line1.substr(44, 8);
            tle.mean_motion_ddot = parseExponent(ddot_str);

            // Parse bstar (scientific notation without 'E')
            std::string bstar_str = line1.substr(53, 8);
            tle.bstar = parseExponent(bstar_str);

            tle.ephemeris_type = std::stoi(line1.substr(62, 1));
            tle.element_set_number = std::stoul(line1.substr(64, 4));

            // Parse Line 2
            uint32_t norad_id2 = std::stoul(line2.substr(2, 5));
            if (norad_id2 != tle.norad_id)
                return std::nullopt;  // NORAD IDs don't match

            tle.inclination = std::stod(line2.substr(8, 8));
            tle.raan = std::stod(line2.substr(17, 8));

            // Eccentricity (decimal point assumed)
            std::string ecc_str = "0." + line2.substr(26, 7);
            tle.eccentricity = std::stod(ecc_str);

            tle.arg_periapsis = std::stod(line2.substr(34, 8));
            tle.mean_anomaly = std::stod(line2.substr(43, 8));
            tle.mean_motion = std::stod(line2.substr(52, 11));
            tle.revolution_number = std::stoul(line2.substr(63, 5));

            // Validate checksum
            if (!validateChecksum(line1) || !validateChecksum(line2))
                return std::nullopt;

            return tle;
        }
        catch (const std::exception& e)
        {
            return std::nullopt;
        }
    }

    /// Parse TLE from three lines (name + line1 + line2)
    static std::optional<TLEData> parseThreeLines(const std::string& line0,
                                                   const std::string& line1,
                                                   const std::string& line2)
    {
        return parse(line1, line2, trim(line0));
    }

    /// Parse multiple TLEs from a file
    static std::vector<TLEData> parseFile(const std::string& filename)
    {
        std::vector<TLEData> tles;
        std::ifstream file(filename);

        if (!file.is_open())
            return tles;

        std::vector<std::string> lines;
        std::string line;

        while (std::getline(file, line))
        {
            if (line.empty()) continue;
            lines.push_back(line);

            // Check if we have a complete TLE (2 or 3 lines)
            if (lines.size() >= 2)
            {
                // Check if last two lines are TLE lines
                const std::string& lastLine = lines.back();
                const std::string& prevLine = lines[lines.size() - 2];

                if (!lastLine.empty() && lastLine[0] == '2' &&
                    !prevLine.empty() && prevLine[0] == '1')
                {
                    std::string name;
                    if (lines.size() >= 3)
                    {
                        const std::string& nameLine = lines[lines.size() - 3];
                        if (!nameLine.empty() && nameLine[0] != '1' && nameLine[0] != '2')
                            name = nameLine;
                    }

                    auto tle = parse(prevLine, lastLine, name);
                    if (tle)
                        tles.push_back(*tle);

                    lines.clear();
                }
            }
        }

        return tles;
    }

    /// Parse TLE from string containing multiple lines
    static std::vector<TLEData> parseString(const std::string& data)
    {
        std::vector<TLEData> tles;
        std::istringstream stream(data);
        std::vector<std::string> lines;
        std::string line;

        while (std::getline(stream, line))
        {
            // Remove carriage return if present
            if (!line.empty() && line.back() == '\r')
                line.pop_back();

            if (line.empty()) continue;
            lines.push_back(line);

            if (lines.size() >= 2)
            {
                const std::string& lastLine = lines.back();
                const std::string& prevLine = lines[lines.size() - 2];

                if (!lastLine.empty() && lastLine[0] == '2' &&
                    !prevLine.empty() && prevLine[0] == '1')
                {
                    std::string name;
                    if (lines.size() >= 3)
                    {
                        const std::string& nameLine = lines[lines.size() - 3];
                        if (!nameLine.empty() && nameLine[0] != '1' && nameLine[0] != '2')
                            name = nameLine;
                    }

                    auto tle = parse(prevLine, lastLine, name);
                    if (tle)
                        tles.push_back(*tle);

                    lines.clear();
                }
            }
        }

        return tles;
    }

private:
    static std::string trim(const std::string& str)
    {
        size_t start = str.find_first_not_of(" \t\r\n");
        size_t end = str.find_last_not_of(" \t\r\n");
        return (start == std::string::npos) ? "" : str.substr(start, end - start + 1);
    }

    // Parse TLE-style scientific notation (e.g., " 12345-6" = 0.12345e-6)
    static double parseExponent(const std::string& str)
    {
        std::string s = trim(str);
        if (s.empty()) return 0.0;

        // Find exponent position
        size_t exp_pos = s.find_last_of("+-");
        if (exp_pos == std::string::npos || exp_pos == 0)
        {
            // No exponent, treat as mantissa
            if (s[0] == '-' || s[0] == '+' || s[0] == '.')
                return std::stod(s);
            return std::stod("0." + s);
        }

        // Split into mantissa and exponent
        std::string mantissa = s.substr(0, exp_pos);
        std::string exponent = s.substr(exp_pos);

        // Handle mantissa
        double mant_val;
        if (mantissa[0] == '-')
        {
            mant_val = -std::stod("0." + mantissa.substr(1));
        }
        else if (mantissa[0] == '+')
        {
            mant_val = std::stod("0." + mantissa.substr(1));
        }
        else
        {
            mant_val = std::stod("0." + mantissa);
        }

        int exp_val = std::stoi(exponent);
        return mant_val * std::pow(10.0, exp_val);
    }

    static bool validateChecksum(const std::string& line)
    {
        if (line.length() < 69) return false;

        int checksum = 0;
        for (size_t i = 0; i < 68; ++i)
        {
            char c = line[i];
            if (c >= '0' && c <= '9')
                checksum += (c - '0');
            else if (c == '-')
                checksum += 1;
        }

        int expected = line[68] - '0';
        return (checksum % 10) == expected;
    }
};

} // namespace hpop_tle
