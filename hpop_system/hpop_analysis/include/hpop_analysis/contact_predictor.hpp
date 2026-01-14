/**
 * @file contact_predictor.hpp
 * @brief Ground station contact prediction for satellites
 *
 * Calculates visibility windows between satellites and ground stations.
 * Considers Earth's rotation, satellite orbit, and terrain masking.
 */

#ifndef HPOP_ANALYSIS_CONTACT_PREDICTOR_HPP
#define HPOP_ANALYSIS_CONTACT_PREDICTOR_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "hpop_core/constants.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"

namespace hpop_analysis
{

/**
 * @brief Ground station definition
 */
struct GroundStation
{
    std::string id;
    std::string name;
    double latitude;           // [rad]
    double longitude;          // [rad]
    double altitude;           // [m] above ellipsoid
    double min_elevation;      // [rad] minimum elevation angle

    GroundStation() = default;

    GroundStation(const std::string& id_, const std::string& name_,
                  double lat_deg, double lon_deg, double alt_m = 0.0,
                  double min_el_deg = 5.0)
        : id(id_), name(name_)
        , latitude(lat_deg * hpop_core::constants::DEG_TO_RAD)
        , longitude(lon_deg * hpop_core::constants::DEG_TO_RAD)
        , altitude(alt_m)
        , min_elevation(min_el_deg * hpop_core::constants::DEG_TO_RAD)
    {}

    /**
     * @brief Get ground station position in ECEF
     */
    hpop_core::Vec3 getPositionECEF() const
    {
        hpop_core::GeodeticCoord geo;
        geo.latitude = latitude;
        geo.longitude = longitude;
        geo.altitude = altitude;
        return hpop_core::CoordinateFrames::geodeticToEcef(geo);
    }

    /**
     * @brief Get ground station position in ECI at given time
     */
    hpop_core::Vec3 getPositionECI(double jd) const
    {
        hpop_core::Vec3 ecef = getPositionECEF();
        // Use StateVector wrapper for conversion
        hpop_core::StateVector ecef_sv;
        ecef_sv.position = ecef;
        ecef_sv.velocity = hpop_core::Vec3(0, 0, 0);
        ecef_sv.epoch = jd;
        return hpop_core::CoordinateFrames::ecefToEci(ecef_sv, jd).position;
    }
};

/**
 * @brief Contact window between satellite and ground station
 */
struct ContactWindow
{
    std::string satellite_id;
    std::string station_id;
    double aos_jd;             // Acquisition of Signal (Julian Date)
    double los_jd;             // Loss of Signal (Julian Date)
    double tca_jd;             // Time of Closest Approach
    double max_elevation;      // [rad] maximum elevation angle
    double aos_azimuth;        // [rad] azimuth at AOS
    double los_azimuth;        // [rad] azimuth at LOS

    double getDurationMinutes() const
    {
        return (los_jd - aos_jd) * 1440.0;  // JD to minutes
    }

    double getDurationSeconds() const
    {
        return (los_jd - aos_jd) * 86400.0;  // JD to seconds
    }
};

/**
 * @brief Contact predictor class
 */
class ContactPredictor
{
public:
    ContactPredictor() = default;

    /**
     * @brief Add a ground station to the predictor
     */
    void addGroundStation(const GroundStation& station)
    {
        stations_.push_back(station);
    }

    /**
     * @brief Clear all ground stations
     */
    void clearGroundStations()
    {
        stations_.clear();
    }

    /**
     * @brief Get all registered ground stations
     */
    const std::vector<GroundStation>& getGroundStations() const
    {
        return stations_;
    }

    /**
     * @brief Calculate elevation and azimuth from ground station to satellite
     *
     * @param gs Ground station
     * @param sat_eci Satellite position in ECI [m]
     * @param jd Julian date
     * @param elevation Output elevation angle [rad]
     * @param azimuth Output azimuth angle [rad]
     * @return true if satellite is above horizon
     */
    static bool calculateElevationAzimuth(const GroundStation& gs,
                                          const hpop_core::Vec3& sat_eci,
                                          double jd,
                                          double& elevation,
                                          double& azimuth)
    {
        using namespace hpop_core;

        // Convert satellite position to ECEF (use StateVector wrapper)
        StateVector eci_sv;
        eci_sv.position = sat_eci;
        eci_sv.velocity = Vec3(0, 0, 0);
        eci_sv.epoch = jd;
        Vec3 sat_ecef = CoordinateFrames::eciToEcef(eci_sv, jd).position;

        // Get ground station ECEF position
        Vec3 gs_ecef = gs.getPositionECEF();

        // Vector from ground station to satellite in ECEF
        Vec3 range_ecef = sat_ecef - gs_ecef;

        // Convert to local ENU (East-North-Up) frame
        double sin_lat = std::sin(gs.latitude);
        double cos_lat = std::cos(gs.latitude);
        double sin_lon = std::sin(gs.longitude);
        double cos_lon = std::cos(gs.longitude);

        // Rotation matrix ECEF to ENU
        double east = -sin_lon * range_ecef.x + cos_lon * range_ecef.y;
        double north = -sin_lat * cos_lon * range_ecef.x - sin_lat * sin_lon * range_ecef.y
                       + cos_lat * range_ecef.z;
        double up = cos_lat * cos_lon * range_ecef.x + cos_lat * sin_lon * range_ecef.y
                    + sin_lat * range_ecef.z;

        // Calculate range
        double range = std::sqrt(east * east + north * north + up * up);

        // Elevation angle (angle from horizontal plane)
        elevation = std::asin(up / range);

        // Azimuth angle (clockwise from North)
        azimuth = std::atan2(east, north);
        if (azimuth < 0) azimuth += 2.0 * constants::PI;

        return elevation > 0;  // Above horizon
    }

    /**
     * @brief Check if satellite is visible from ground station
     */
    static bool isVisible(const GroundStation& gs,
                          const hpop_core::Vec3& sat_eci,
                          double jd)
    {
        double el, az;
        if (!calculateElevationAzimuth(gs, sat_eci, jd, el, az))
            return false;
        return el >= gs.min_elevation;
    }

    /**
     * @brief Predict contact windows for a satellite trajectory
     *
     * @param satellite_id Satellite identifier
     * @param trajectory Vector of (JD, StateVector) pairs
     * @return Vector of contact windows for all ground stations
     */
    std::vector<ContactWindow> predictContacts(
        const std::string& satellite_id,
        const std::vector<std::pair<double, hpop_core::StateVector>>& trajectory) const
    {
        std::vector<ContactWindow> contacts;

        for (const auto& station : stations_)
        {
            auto station_contacts = predictContactsForStation(
                satellite_id, station, trajectory);
            contacts.insert(contacts.end(),
                           station_contacts.begin(),
                           station_contacts.end());
        }

        // Sort by AOS time
        std::sort(contacts.begin(), contacts.end(),
                  [](const ContactWindow& a, const ContactWindow& b) {
                      return a.aos_jd < b.aos_jd;
                  });

        return contacts;
    }

    /**
     * @brief Predict contacts for a specific ground station
     */
    std::vector<ContactWindow> predictContactsForStation(
        const std::string& satellite_id,
        const GroundStation& station,
        const std::vector<std::pair<double, hpop_core::StateVector>>& trajectory) const
    {
        std::vector<ContactWindow> contacts;

        if (trajectory.size() < 2) return contacts;

        bool in_contact = false;
        ContactWindow current_contact;
        double max_el = -hpop_core::constants::PI;
        double tca_jd = 0.0;

        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            double jd = trajectory[i].first;
            const auto& state = trajectory[i].second;

            double el, az;
            bool above_horizon = calculateElevationAzimuth(station, state.position, jd, el, az);
            bool visible = above_horizon && (el >= station.min_elevation);

            if (visible && !in_contact)
            {
                // Start of contact
                in_contact = true;
                current_contact = ContactWindow();
                current_contact.satellite_id = satellite_id;
                current_contact.station_id = station.id;
                current_contact.aos_jd = jd;
                current_contact.aos_azimuth = az;
                max_el = el;
                tca_jd = jd;
            }
            else if (visible && in_contact)
            {
                // Continuing contact - track maximum elevation
                if (el > max_el)
                {
                    max_el = el;
                    tca_jd = jd;
                }
            }
            else if (!visible && in_contact)
            {
                // End of contact
                in_contact = false;
                current_contact.los_jd = jd;
                current_contact.los_azimuth = az;
                current_contact.max_elevation = max_el;
                current_contact.tca_jd = tca_jd;
                contacts.push_back(current_contact);
            }
        }

        // Handle contact that extends beyond trajectory end
        if (in_contact)
        {
            current_contact.los_jd = trajectory.back().first;
            double el, az;
            calculateElevationAzimuth(station, trajectory.back().second.position,
                                      trajectory.back().first, el, az);
            current_contact.los_azimuth = az;
            current_contact.max_elevation = max_el;
            current_contact.tca_jd = tca_jd;
            contacts.push_back(current_contact);
        }

        return contacts;
    }

    /**
     * @brief Calculate ground track (sub-satellite point)
     */
    static std::pair<double, double> getSubSatellitePoint(
        const hpop_core::Vec3& sat_eci,
        double jd)
    {
        using namespace hpop_core;

        // Convert satellite position to ECEF (use StateVector wrapper)
        StateVector eci_sv;
        eci_sv.position = sat_eci;
        eci_sv.velocity = Vec3(0, 0, 0);
        eci_sv.epoch = jd;
        Vec3 sat_ecef = CoordinateFrames::eciToEcef(eci_sv, jd).position;
        auto geodetic = CoordinateFrames::ecefToGeodetic(sat_ecef);

        return {geodetic.latitude * constants::RAD_TO_DEG,
                geodetic.longitude * constants::RAD_TO_DEG};
    }

    /**
     * @brief Calculate ground track for a trajectory
     */
    static std::vector<std::tuple<double, double, double>> calculateGroundTrack(
        const std::vector<std::pair<double, hpop_core::StateVector>>& trajectory)
    {
        std::vector<std::tuple<double, double, double>> ground_track;
        ground_track.reserve(trajectory.size());

        for (const auto& [jd, state] : trajectory)
        {
            auto [lat, lon] = getSubSatellitePoint(state.position, jd);
            ground_track.emplace_back(jd, lat, lon);
        }

        return ground_track;
    }

private:
    std::vector<GroundStation> stations_;
};

/**
 * @brief Common ground stations database
 */
namespace ground_stations
{

inline GroundStation DAEJEON()
{
    return GroundStation("DAEJEON", "Daejeon Ground Station", 36.35, 127.38, 100.0, 5.0);
}

inline GroundStation SVALBARD()
{
    return GroundStation("SVALBARD", "Svalbard Satellite Station", 78.23, 15.39, 450.0, 3.0);
}

inline GroundStation MCMURDO()
{
    return GroundStation("MCMURDO", "McMurdo Ground Station", -77.85, 166.67, 24.0, 5.0);
}

inline GroundStation CANBERRA()
{
    return GroundStation("CANBERRA", "Canberra DSN", -35.40, 148.98, 680.0, 5.0);
}

inline GroundStation MADRID()
{
    return GroundStation("MADRID", "Madrid DSN", 40.43, -4.25, 834.0, 5.0);
}

inline GroundStation GOLDSTONE()
{
    return GroundStation("GOLDSTONE", "Goldstone DSN", 35.43, -116.89, 1001.0, 5.0);
}

inline GroundStation KIRUNA()
{
    return GroundStation("KIRUNA", "Kiruna Ground Station", 67.86, 21.06, 440.0, 5.0);
}

inline GroundStation KOUROU()
{
    return GroundStation("KOUROU", "Kourou Ground Station", 5.17, -52.68, 25.0, 5.0);
}

} // namespace ground_stations

} // namespace hpop_analysis

#endif // HPOP_ANALYSIS_CONTACT_PREDICTOR_HPP
