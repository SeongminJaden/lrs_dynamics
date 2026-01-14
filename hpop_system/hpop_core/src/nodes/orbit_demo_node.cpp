/**
 * @file orbit_demo_node.cpp
 * @brief Demo node that publishes simulated satellite orbits for RViz visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <hpop_msgs/msg/satellite_state.hpp>
#include <hpop_msgs/msg/orbital_elements.hpp>
#include <cmath>
#include <vector>
#include <string>

namespace hpop_core
{

struct SatelliteConfig
{
    std::string id;
    std::string name;
    uint32_t norad_id;
    double semi_major_axis;  // m
    double eccentricity;
    double inclination;      // rad
    double raan;             // rad
    double arg_perigee;      // rad
    double mean_anomaly;     // rad (at t=0)
};

class OrbitDemoNode : public rclcpp::Node
{
public:
    OrbitDemoNode() : Node("orbit_demo_node"), simulation_time_(0.0)
    {
        // Initialize satellites
        initializeSatellites();

        // Create publisher
        state_pub_ = this->create_publisher<hpop_msgs::msg::SatelliteState>(
            "/hpop/satellite_state", 10);

        // Create timer (10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OrbitDemoNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Orbit Demo Node started with %zu satellites",
                    satellites_.size());
        RCLCPP_INFO(this->get_logger(), "Publishing to /hpop/satellite_state at 10 Hz");
        RCLCPP_INFO(this->get_logger(), "Time scale: 100x real-time");
    }

private:
    void initializeSatellites()
    {
        // ISS - Low Earth Orbit
        satellites_.push_back({
            "ISS",
            "ISS (ZARYA)",
            25544,
            6.778e6,    // 400 km altitude
            0.0001,
            51.64 * M_PI / 180.0,
            30.0 * M_PI / 180.0,
            0.0,
            0.0
        });

        // Starlink - LEO constellation
        satellites_.push_back({
            "STARLINK-1234",
            "STARLINK-1234",
            48274,
            6.928e6,    // 550 km altitude
            0.0001,
            53.0 * M_PI / 180.0,
            60.0 * M_PI / 180.0,
            0.0,
            120.0 * M_PI / 180.0
        });

        // Sentinel-6A - Medium altitude
        satellites_.push_back({
            "SENTINEL-6A",
            "SENTINEL-6A",
            46984,
            7.714e6,    // 1336 km altitude
            0.0001,
            66.0 * M_PI / 180.0,
            120.0 * M_PI / 180.0,
            0.0,
            240.0 * M_PI / 180.0
        });

        // GPS satellite - MEO
        satellites_.push_back({
            "GPS-IIR-10",
            "GPS BIIR-10 (PRN 20)",
            28361,
            26.56e6,    // 20,200 km altitude
            0.005,
            55.0 * M_PI / 180.0,
            0.0,
            0.0,
            90.0 * M_PI / 180.0
        });

        // Molniya-type HEO
        satellites_.push_back({
            "MOLNIYA-SIM",
            "MOLNIYA SIMULATION",
            99999,
            26.554e6,   // Semi-major axis for 12-hour orbit
            0.74,       // High eccentricity
            63.4 * M_PI / 180.0,  // Critical inclination
            -90.0 * M_PI / 180.0,
            270.0 * M_PI / 180.0,
            0.0
        });
    }

    void timerCallback()
    {
        // Time scale: 100x real-time
        double dt = 0.1 * 100.0;  // 10 seconds per tick
        simulation_time_ += dt;

        const double mu = 3.986004418e14;  // Earth GM

        for (auto& sat : satellites_)
        {
            // Calculate orbital period
            double n = std::sqrt(mu / (sat.semi_major_axis * sat.semi_major_axis * sat.semi_major_axis));

            // Update mean anomaly
            double M = sat.mean_anomaly + n * simulation_time_;
            M = std::fmod(M, 2.0 * M_PI);

            // Solve Kepler's equation (simple iteration)
            double E = M;
            for (int i = 0; i < 10; ++i)
            {
                E = M + sat.eccentricity * std::sin(E);
            }

            // True anomaly
            double nu = 2.0 * std::atan2(
                std::sqrt(1.0 + sat.eccentricity) * std::sin(E / 2.0),
                std::sqrt(1.0 - sat.eccentricity) * std::cos(E / 2.0)
            );

            // Radius
            double r = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E));

            // Position in orbital plane
            double x_orb = r * std::cos(nu);
            double y_orb = r * std::sin(nu);

            // Velocity in orbital plane
            double h = std::sqrt(mu * sat.semi_major_axis * (1.0 - sat.eccentricity * sat.eccentricity));
            double vx_orb = -mu / h * std::sin(nu);
            double vy_orb = mu / h * (sat.eccentricity + std::cos(nu));

            // Rotation matrices
            double cos_O = std::cos(sat.raan);
            double sin_O = std::sin(sat.raan);
            double cos_i = std::cos(sat.inclination);
            double sin_i = std::sin(sat.inclination);
            double cos_w = std::cos(sat.arg_perigee);
            double sin_w = std::sin(sat.arg_perigee);

            // Transform to ECI
            double x = (cos_O * cos_w - sin_O * sin_w * cos_i) * x_orb +
                       (-cos_O * sin_w - sin_O * cos_w * cos_i) * y_orb;
            double y = (sin_O * cos_w + cos_O * sin_w * cos_i) * x_orb +
                       (-sin_O * sin_w + cos_O * cos_w * cos_i) * y_orb;
            double z = (sin_w * sin_i) * x_orb + (cos_w * sin_i) * y_orb;

            double vx = (cos_O * cos_w - sin_O * sin_w * cos_i) * vx_orb +
                        (-cos_O * sin_w - sin_O * cos_w * cos_i) * vy_orb;
            double vy = (sin_O * cos_w + cos_O * sin_w * cos_i) * vx_orb +
                        (-sin_O * sin_w + cos_O * cos_w * cos_i) * vy_orb;
            double vz = (sin_w * sin_i) * vx_orb + (cos_w * sin_i) * vy_orb;

            // Create message
            auto msg = hpop_msgs::msg::SatelliteState();
            msg.header.stamp = this->now();
            msg.header.frame_id = "earth";
            msg.satellite_id = sat.id;
            msg.name = sat.name;
            msg.norad_id = sat.norad_id;

            // Position (meters)
            msg.position.x = x;
            msg.position.y = y;
            msg.position.z = z;

            // Velocity (m/s)
            msg.velocity.x = vx;
            msg.velocity.y = vy;
            msg.velocity.z = vz;

            // Orbital elements (in radians)
            msg.elements.semi_major_axis = sat.semi_major_axis;
            msg.elements.eccentricity = sat.eccentricity;
            msg.elements.inclination = sat.inclination;
            msg.elements.raan = sat.raan;
            msg.elements.arg_periapsis = sat.arg_perigee;
            msg.elements.true_anomaly = nu;

            // Physical properties
            msg.mass = 1000.0;
            msg.cross_section_drag = 10.0;
            msg.drag_coefficient = 2.2;
            msg.reflectivity_coeff = 1.3;

            state_pub_->publish(msg);
        }

        // Log status every 60 seconds of simulation time
        if (static_cast<int>(simulation_time_) % 600 == 0 &&
            static_cast<int>(simulation_time_) != last_log_time_)
        {
            last_log_time_ = static_cast<int>(simulation_time_);
            RCLCPP_INFO(this->get_logger(), "Simulation time: %.1f minutes",
                        simulation_time_ / 60.0);
        }
    }

    std::vector<SatelliteConfig> satellites_;
    rclcpp::Publisher<hpop_msgs::msg::SatelliteState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double simulation_time_;
    int last_log_time_ = 0;
};

}  // namespace hpop_core

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hpop_core::OrbitDemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
