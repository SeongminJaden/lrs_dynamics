#pragma once

#include <cmath>
#include <ctime>
#include <string>
#include "hpop_core/constants.hpp"

namespace hpop_core
{

/// Time system utilities for astrodynamics computations
class TimeSystem
{
public:
    /// Convert calendar date to Julian Date
    static double calendarToJD(int year, int month, int day,
                               int hour = 0, int minute = 0, double second = 0.0)
    {
        // Using algorithm from Vallado "Fundamentals of Astrodynamics"
        double jd = 367.0 * year
                  - std::floor(7.0 * (year + std::floor((month + 9.0) / 12.0)) / 4.0)
                  + std::floor(275.0 * month / 9.0)
                  + day + 1721013.5
                  + ((second / 60.0 + minute) / 60.0 + hour) / 24.0;
        return jd;
    }

    /// Convert Julian Date to calendar date
    static void jdToCalendar(double jd, int& year, int& month, int& day,
                            int& hour, int& minute, double& second)
    {
        double z = std::floor(jd + 0.5);
        double frac = jd + 0.5 - z;

        double alpha = std::floor((z - 1867216.25) / 36524.25);
        double a = z + 1 + alpha - std::floor(alpha / 4.0);
        double b = a + 1524;
        double c = std::floor((b - 122.1) / 365.25);
        double d = std::floor(365.25 * c);
        double e = std::floor((b - d) / 30.6001);

        day = static_cast<int>(b - d - std::floor(30.6001 * e));
        month = static_cast<int>((e < 14) ? (e - 1) : (e - 13));
        year = static_cast<int>((month > 2) ? (c - 4716) : (c - 4715));

        double time = frac * 24.0;
        hour = static_cast<int>(time);
        time = (time - hour) * 60.0;
        minute = static_cast<int>(time);
        second = (time - minute) * 60.0;
    }

    /// Convert Julian Date to Modified Julian Date
    static double jdToMJD(double jd)
    {
        return jd - 2400000.5;
    }

    /// Convert Modified Julian Date to Julian Date
    static double mjdToJD(double mjd)
    {
        return mjd + 2400000.5;
    }

    /// Get Julian Date for J2000 epoch
    static double j2000JD()
    {
        return constants::J2000_JD;
    }

    /// Get Julian centuries from J2000
    static double julianCenturiesFromJ2000(double jd)
    {
        return (jd - constants::J2000_JD) / constants::DAYS_PER_JULIAN_CENTURY;
    }

    /// Get current Julian Date
    static double currentJD()
    {
        std::time_t now = std::time(nullptr);
        std::tm* utc = std::gmtime(&now);
        return calendarToJD(utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
                           utc->tm_hour, utc->tm_min, static_cast<double>(utc->tm_sec));
    }

    /// Convert TLE epoch (year, day) to Julian Date
    static double tleEpochToJD(int year, double day)
    {
        // TLE uses 2-digit year: 00-56 -> 2000-2056, 57-99 -> 1957-1999
        if (year < 57)
            year += 2000;
        else
            year += 1900;

        // January 1 of the year
        double jd = calendarToJD(year, 1, 1);

        // Add days (day 1 = Jan 1)
        return jd + day - 1.0;
    }

    /// Greenwich Mean Sidereal Time (GMST) in radians
    static double gmst(double jd)
    {
        double T = julianCenturiesFromJ2000(jd);

        // GMST in seconds at 0h UT
        double gmst_sec = 67310.54841
                        + (876600.0 * 3600.0 + 8640184.812866) * T
                        + 0.093104 * T * T
                        - 6.2e-6 * T * T * T;

        // Convert to radians
        double gmst_rad = std::fmod(gmst_sec, constants::SECONDS_PER_DAY)
                        / constants::SECONDS_PER_DAY * constants::TWO_PI;

        return utils::normalizeAngle(gmst_rad);
    }

    /// Greenwich Apparent Sidereal Time (includes nutation)
    static double gast(double jd)
    {
        // Simplified: GAST ≈ GMST for most applications
        // Full implementation would include nutation terms
        return gmst(jd);
    }

    /// Convert Unix timestamp to Julian Date
    static double unixToJD(double unix_time)
    {
        return unix_time / constants::SECONDS_PER_DAY + 2440587.5;
    }

    /// Convert Julian Date to Unix timestamp
    static double jdToUnix(double jd)
    {
        return (jd - 2440587.5) * constants::SECONDS_PER_DAY;
    }

    /// Format Julian Date as ISO 8601 string
    static std::string jdToISO8601(double jd)
    {
        int year, month, day, hour, minute;
        double second;
        jdToCalendar(jd, year, month, day, hour, minute, second);

        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%06.3fZ",
                     year, month, day, hour, minute, second);
        return std::string(buffer);
    }
};

} // namespace hpop_core
