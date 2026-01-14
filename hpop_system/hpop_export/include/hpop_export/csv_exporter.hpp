/**
 * @file csv_exporter.hpp
 * @brief CSV ephemeris exporter for HPOP
 */

#ifndef HPOP_EXPORT_CSV_EXPORTER_HPP
#define HPOP_EXPORT_CSV_EXPORTER_HPP

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace hpop_export
{

/**
 * @brief Ephemeris data point for export
 */
struct EphemerisPoint
{
    double epoch_jd;           // Julian Date
    double x, y, z;            // Position [m]
    double vx, vy, vz;         // Velocity [m/s]

    // Optional orbital elements
    double semi_major_axis;    // [m]
    double eccentricity;
    double inclination;        // [rad]
    double raan;               // [rad]
    double arg_periapsis;      // [rad]
    double true_anomaly;       // [rad]

    // Optional metadata
    std::string satellite_id;
    uint32_t norad_id;
};

/**
 * @brief CSV export options
 */
struct CsvExportOptions
{
    bool include_header = true;
    bool include_elements = true;
    bool include_velocity = true;
    int position_precision = 3;      // Decimal places for position [m]
    int velocity_precision = 6;      // Decimal places for velocity [m/s]
    int element_precision = 10;      // Decimal places for elements
    char delimiter = ',';
    std::string date_format = "JD";  // JD, ISO8601, MJD
};

/**
 * @brief CSV ephemeris exporter
 */
class CsvExporter
{
public:
    CsvExporter() = default;

    /**
     * @brief Export ephemeris to CSV file
     * @param filename Output filename
     * @param ephemeris Vector of ephemeris points
     * @param options Export options
     * @return true if successful
     */
    bool exportToFile(const std::string& filename,
                      const std::vector<EphemerisPoint>& ephemeris,
                      const CsvExportOptions& options = CsvExportOptions())
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            last_error_ = "Failed to open file: " + filename;
            return false;
        }

        // Write header
        if (options.include_header)
        {
            writeHeader(file, options);
        }

        // Write data
        for (const auto& point : ephemeris)
        {
            writeLine(file, point, options);
        }

        file.close();
        return true;
    }

    /**
     * @brief Export ephemeris to string
     */
    std::string exportToString(const std::vector<EphemerisPoint>& ephemeris,
                               const CsvExportOptions& options = CsvExportOptions())
    {
        std::ostringstream ss;

        if (options.include_header)
        {
            writeHeader(ss, options);
        }

        for (const auto& point : ephemeris)
        {
            writeLine(ss, point, options);
        }

        return ss.str();
    }

    /**
     * @brief Get last error message
     */
    std::string getLastError() const { return last_error_; }

private:
    std::string last_error_;

    template<typename Stream>
    void writeHeader(Stream& stream, const CsvExportOptions& options)
    {
        char d = options.delimiter;

        stream << "Epoch";
        stream << d << "X_m" << d << "Y_m" << d << "Z_m";

        if (options.include_velocity)
        {
            stream << d << "VX_m_s" << d << "VY_m_s" << d << "VZ_m_s";
        }

        if (options.include_elements)
        {
            stream << d << "SMA_m" << d << "ECC" << d << "INC_rad";
            stream << d << "RAAN_rad" << d << "AOP_rad" << d << "TA_rad";
        }

        stream << d << "SatelliteID" << d << "NoradID";
        stream << "\n";
    }

    template<typename Stream>
    void writeLine(Stream& stream, const EphemerisPoint& point,
                   const CsvExportOptions& options)
    {
        char d = options.delimiter;

        // Epoch
        if (options.date_format == "ISO8601")
        {
            stream << jdToIso8601(point.epoch_jd);
        }
        else if (options.date_format == "MJD")
        {
            stream << std::fixed << std::setprecision(10) << (point.epoch_jd - 2400000.5);
        }
        else // JD
        {
            stream << std::fixed << std::setprecision(10) << point.epoch_jd;
        }

        // Position
        stream << std::fixed << std::setprecision(options.position_precision);
        stream << d << point.x << d << point.y << d << point.z;

        // Velocity
        if (options.include_velocity)
        {
            stream << std::fixed << std::setprecision(options.velocity_precision);
            stream << d << point.vx << d << point.vy << d << point.vz;
        }

        // Orbital elements
        if (options.include_elements)
        {
            stream << std::fixed << std::setprecision(options.element_precision);
            stream << d << point.semi_major_axis;
            stream << d << point.eccentricity;
            stream << d << point.inclination;
            stream << d << point.raan;
            stream << d << point.arg_periapsis;
            stream << d << point.true_anomaly;
        }

        // Metadata
        stream << d << point.satellite_id;
        stream << d << point.norad_id;
        stream << "\n";
    }

    /**
     * @brief Convert Julian Date to ISO 8601 string
     */
    std::string jdToIso8601(double jd)
    {
        // Convert JD to calendar date
        double z = std::floor(jd + 0.5);
        double f = jd + 0.5 - z;

        double a;
        if (z < 2299161)
        {
            a = z;
        }
        else
        {
            double alpha = std::floor((z - 1867216.25) / 36524.25);
            a = z + 1 + alpha - std::floor(alpha / 4);
        }

        double b = a + 1524;
        double c = std::floor((b - 122.1) / 365.25);
        double d = std::floor(365.25 * c);
        double e = std::floor((b - d) / 30.6001);

        int day = static_cast<int>(b - d - std::floor(30.6001 * e));
        int month = static_cast<int>((e < 14) ? e - 1 : e - 13);
        int year = static_cast<int>((month > 2) ? c - 4716 : c - 4715);

        // Time of day
        double day_fraction = f;
        int hour = static_cast<int>(day_fraction * 24);
        day_fraction = day_fraction * 24 - hour;
        int minute = static_cast<int>(day_fraction * 60);
        day_fraction = day_fraction * 60 - minute;
        double second = day_fraction * 60;

        std::ostringstream ss;
        ss << std::setfill('0');
        ss << year << "-" << std::setw(2) << month << "-" << std::setw(2) << day;
        ss << "T" << std::setw(2) << hour << ":" << std::setw(2) << minute;
        ss << ":" << std::fixed << std::setprecision(3) << std::setw(6) << second << "Z";

        return ss.str();
    }
};

} // namespace hpop_export

#endif // HPOP_EXPORT_CSV_EXPORTER_HPP
