/**
 * @file orbit_demo_node.cpp
 * @brief Demo node that publishes simulated satellite orbits for RViz visualization
 *
 * Propagation is controlled via services:
 * - /hpop/start_propagation: Start orbit propagation
 * - /hpop/stop_propagation: Stop orbit propagation
 * - /hpop/reset_propagation: Reset simulation time to 0
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <hpop_msgs/msg/satellite_state.hpp>
#include <hpop_msgs/msg/orbital_elements.hpp>
#include <hpop_msgs/msg/maneuver_plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo_msgs/srv/set_entity_state.hpp>
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

    // Additional velocity offset from delta-v maneuvers
    double delta_v_accumulated = 0.0;  // Accumulated in-track delta-v effect
};

class OrbitDemoNode : public rclcpp::Node
{
public:
    OrbitDemoNode() : Node("orbit_demo_node"), simulation_time_(0.0), propagating_(false)
    {
        // Initialize satellites
        initializeSatellites();

        // Create publishers
        state_pub_ = this->create_publisher<hpop_msgs::msg::SatelliteState>(
            "/hpop/satellite_state", 10);

        // Orbit path publishers
        chaser_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/hpop/chaser_orbit_path", 10);
        target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/hpop/target_orbit_path", 10);

        // Satellite markers publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/hpop/satellite_markers", 10);

        // Earth marker publisher
        earth_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/earth_marker", 10);

        // Scaled visualization publishers (for orbit view RViz)
        viz_chaser_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/visualization/chaser_orbit", 10);
        viz_target_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/visualization/target_orbit", 10);
        viz_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization/satellite_markers", 10);
        viz_earth_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization/earth_marker", 10);

        // LVLH (Local Vertical Local Horizontal) relative motion visualization
        // Shows Target position relative to Chaser for rendezvous verification
        lvlh_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lvlh/satellite_markers", 10);
        lvlh_distance_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/lvlh/distance_text", 10);
        lvlh_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/lvlh/relative_path", 10);

        // TF broadcaster for world frame
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Gazebo set entity state client
        gazebo_set_state_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
            "/gazebo/set_entity_state");

        // Create services for propagation control
        start_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hpop/start_propagation",
            std::bind(&OrbitDemoNode::startPropagationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hpop/stop_propagation",
            std::bind(&OrbitDemoNode::stopPropagationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/hpop/reset_propagation",
            std::bind(&OrbitDemoNode::resetPropagationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Delta-V command subscriber (for rendezvous maneuvers)
        deltav_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/hpop/apply_delta_v", 10,
            std::bind(&OrbitDemoNode::deltaVCallback, this, std::placeholders::_1));

        // Create timer (20 Hz for smoother visualization)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OrbitDemoNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Orbit Demo Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Satellites: %zu (TARGET, CHASER)", satellites_.size());
        RCLCPP_INFO(this->get_logger(), "  Publisher: /hpop/satellite_state");
        RCLCPP_INFO(this->get_logger(), "  Time scale: 50x real-time (smooth)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Control services:");
        RCLCPP_INFO(this->get_logger(), "  - /hpop/start_propagation");
        RCLCPP_INFO(this->get_logger(), "  - /hpop/stop_propagation");
        RCLCPP_INFO(this->get_logger(), "  - /hpop/reset_propagation");
        RCLCPP_INFO(this->get_logger(), "  - /hpop/apply_delta_v (topic)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "LVLH Relative Motion Visualization:");
        RCLCPP_INFO(this->get_logger(), "  - /lvlh/satellite_markers");
        RCLCPP_INFO(this->get_logger(), "  - /lvlh/distance_text");
        RCLCPP_INFO(this->get_logger(), "  - /lvlh/relative_path");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Waiting for start command...");

        // Publish initial state (satellites at initial positions)
        publishSatelliteStates();
    }

private:
    void startPropagationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (propagating_) {
            response->success = false;
            response->message = "Propagation already running";
        } else {
            propagating_ = true;
            response->success = true;
            response->message = "Orbit propagation started";
            RCLCPP_INFO(this->get_logger(), ">>> Orbit propagation STARTED <<<");
        }
    }

    void stopPropagationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!propagating_) {
            response->success = false;
            response->message = "Propagation not running";
        } else {
            propagating_ = false;
            response->success = true;
            response->message = "Orbit propagation stopped";
            RCLCPP_INFO(this->get_logger(), ">>> Orbit propagation STOPPED <<<");
        }
    }

    void resetPropagationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        simulation_time_ = 0.0;
        last_log_time_ = 0;
        lvlh_path_history_.clear();  // Clear LVLH path history
        // Reset satellite orbital elements to initial values
        initializeSatellites();
        response->success = true;
        response->message = "Simulation time reset to 0";
        RCLCPP_INFO(this->get_logger(), ">>> Simulation time RESET to 0 <<<");
        // Publish initial state
        publishSatelliteStates();
    }

    void initializeSatellites()
    {
        // Rendezvous Mission: Target and Chaser satellites only

        // TARGET satellite - ISS orbit position (400 km altitude)
        // This is the debris/disabled satellite to capture
        satellites_.push_back({
            "TARGET",
            "Target Satellite (Debris)",
            99001,
            6.778e6,    // 400 km altitude (ISS-like orbit)
            0.0001,
            51.64 * M_PI / 180.0,  // ISS inclination
            30.0 * M_PI / 180.0,   // RAAN
            0.0,                    // Arg of perigee
            0.0                     // Mean anomaly at t=0
        });

        // CHASER satellite - 1500 km behind Target in same orbit
        // Arc distance = radius * angle
        // 1500 km / 6778 km ≈ 0.221 radians ≈ 12.7 degrees
        double phase_offset = 1500.0e3 / 6.778e6;  // ~0.221 radians behind

        satellites_.push_back({
            "CHASER",
            "Chaser Satellite (Active)",
            99002,
            6.778e6,    // Same orbit as Target
            0.0001,
            51.64 * M_PI / 180.0,  // Same inclination
            30.0 * M_PI / 180.0,   // Same RAAN
            0.0,                    // Same arg of perigee
            -phase_offset           // Behind Target by 1500km
        });
    }

    void timerCallback()
    {
        // Only advance time if propagating
        if (propagating_) {
            // Time scale: 50x real-time for smoother visualization
            double dt = 0.05 * 50.0;  // 2.5 seconds per tick (20Hz * 50x = 1000 sim sec/real sec... actually 2.5s sim per tick)
            simulation_time_ += dt;
        }

        // Always publish satellite states (even when paused)
        publishSatelliteStates();
        publishOrbitPaths();
        publishEarthMarker();
        publishWorldTF();
        publishScaledVisualization();
        publishLVLHVisualization();

        // Log status every 60 seconds of simulation time (only when propagating)
        if (propagating_ &&
            static_cast<int>(simulation_time_) % 600 == 0 &&
            static_cast<int>(simulation_time_) != last_log_time_)
        {
            last_log_time_ = static_cast<int>(simulation_time_);
            RCLCPP_INFO(this->get_logger(), "Simulation time: %.1f minutes (%.1f orbits)",
                        simulation_time_ / 60.0,
                        simulation_time_ / 5554.0);  // ISS orbital period ~92.6 min
        }
    }

    void deltaVCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // Apply delta-v to CHASER satellite
        // msg->x = radial, msg->y = in-track (prograde), msg->z = cross-track
        // Positive in-track = prograde = raises orbit = slows down
        // Negative in-track = retrograde = lowers orbit = speeds up (for catching up)

        const double mu = 3.986004418e14;  // Earth GM

        for (auto& sat : satellites_) {
            if (sat.id == "CHASER") {
                // In-track delta-v changes the semi-major axis
                // For small delta-v: delta_a = 2 * a * delta_v / v_circular
                double v_circular = std::sqrt(mu / sat.semi_major_axis);
                double delta_a = 2.0 * sat.semi_major_axis * msg->y / v_circular;

                // Apply the change (negative delta_a = lower orbit = faster = catch up)
                sat.semi_major_axis += delta_a;

                // Accumulate delta-v for tracking
                sat.delta_v_accumulated += msg->y;

                RCLCPP_INFO(this->get_logger(),
                    "Delta-V applied to CHASER: in-track=%.2f m/s, delta_a=%.1f m, new_a=%.1f km",
                    msg->y, delta_a, sat.semi_major_axis / 1000.0);

                // Also store radial component for phasing adjustments
                if (std::abs(msg->x) > 0.001) {
                    // Radial delta-v changes eccentricity
                    double delta_e = msg->x / v_circular;
                    sat.eccentricity += delta_e;
                    if (sat.eccentricity < 0) sat.eccentricity = 0.0001;
                    RCLCPP_INFO(this->get_logger(), "  Radial delta-v: %.2f m/s, new_e=%.6f",
                        msg->x, sat.eccentricity);
                }
            }
        }
    }

    void publishSatelliteStates()
    {
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
            msg.header.frame_id = "world";
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

            // Note: Gazebo uses Floating Origin (Chaser at 0,0,0)
            // Gazebo model positions are NOT updated here
            // RViz shows real orbit coordinates via TF and Path messages
            // Gazebo shows docking simulation with relative positions
        }
    }

    void publishOrbitPaths()
    {
        const double mu = 3.986004418e14;
        const int num_points = 100;  // Points per orbit

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t sat_idx = 0; sat_idx < satellites_.size(); ++sat_idx)
        {
            auto& sat = satellites_[sat_idx];
            nav_msgs::msg::Path path;
            path.header.stamp = this->now();
            path.header.frame_id = "world";

            // Generate full orbit path
            for (int i = 0; i <= num_points; ++i)
            {
                double M = 2.0 * M_PI * i / num_points;

                // Solve Kepler's equation
                double E = M;
                for (int j = 0; j < 10; ++j)
                    E = M + sat.eccentricity * std::sin(E);

                double nu = 2.0 * std::atan2(
                    std::sqrt(1.0 + sat.eccentricity) * std::sin(E / 2.0),
                    std::sqrt(1.0 - sat.eccentricity) * std::cos(E / 2.0));

                double r = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E));
                double x_orb = r * std::cos(nu);
                double y_orb = r * std::sin(nu);

                // Transform to ECI
                double cos_O = std::cos(sat.raan);
                double sin_O = std::sin(sat.raan);
                double cos_i = std::cos(sat.inclination);
                double sin_i = std::sin(sat.inclination);
                double cos_w = std::cos(sat.arg_perigee);
                double sin_w = std::sin(sat.arg_perigee);

                double x = (cos_O * cos_w - sin_O * sin_w * cos_i) * x_orb +
                           (-cos_O * sin_w - sin_O * cos_w * cos_i) * y_orb;
                double y = (sin_O * cos_w + cos_O * sin_w * cos_i) * x_orb +
                           (-sin_O * sin_w + cos_O * cos_w * cos_i) * y_orb;
                double z = (sin_w * sin_i) * x_orb + (cos_w * sin_i) * y_orb;

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = z;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            // Publish path
            if (sat.id == "CHASER")
                chaser_path_pub_->publish(path);
            else
                target_path_pub_->publish(path);

            // Create satellite marker (sphere)
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "world";
            marker.ns = "satellites";
            marker.id = sat_idx;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Get current position from last published state
            double n = std::sqrt(mu / (sat.semi_major_axis * sat.semi_major_axis * sat.semi_major_axis));
            double M_curr = sat.mean_anomaly + n * simulation_time_;
            double E_curr = M_curr;
            for (int j = 0; j < 10; ++j)
                E_curr = M_curr + sat.eccentricity * std::sin(E_curr);

            double nu_curr = 2.0 * std::atan2(
                std::sqrt(1.0 + sat.eccentricity) * std::sin(E_curr / 2.0),
                std::sqrt(1.0 - sat.eccentricity) * std::cos(E_curr / 2.0));

            double r_curr = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E_curr));
            double x_orb = r_curr * std::cos(nu_curr);
            double y_orb = r_curr * std::sin(nu_curr);

            double cos_O = std::cos(sat.raan);
            double sin_O = std::sin(sat.raan);
            double cos_i = std::cos(sat.inclination);
            double sin_i = std::sin(sat.inclination);
            double cos_w = std::cos(sat.arg_perigee);
            double sin_w = std::sin(sat.arg_perigee);

            marker.pose.position.x = (cos_O * cos_w - sin_O * sin_w * cos_i) * x_orb +
                                     (-cos_O * sin_w - sin_O * cos_w * cos_i) * y_orb;
            marker.pose.position.y = (sin_O * cos_w + cos_O * sin_w * cos_i) * x_orb +
                                     (-sin_O * sin_w + cos_O * cos_w * cos_i) * y_orb;
            marker.pose.position.z = (sin_w * sin_i) * x_orb + (cos_w * sin_i) * y_orb;
            marker.pose.orientation.w = 1.0;

            // Marker size (visible at orbit scale)
            marker.scale.x = 200000.0;  // 200km sphere for visibility
            marker.scale.y = 200000.0;
            marker.scale.z = 200000.0;

            // Color: green for chaser, orange for target
            if (sat.id == "CHASER") {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
            }
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    void publishWorldTF()
    {
        // Publish world -> earth transform continuously
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "world";
        t.child_frame_id = "earth";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    void publishEarthMarker()
    {
        visualization_msgs::msg::Marker earth;
        earth.header.stamp = this->now();
        earth.header.frame_id = "world";
        earth.ns = "earth";
        earth.id = 0;
        earth.type = visualization_msgs::msg::Marker::SPHERE;
        earth.action = visualization_msgs::msg::Marker::ADD;

        // Earth at origin
        earth.pose.position.x = 0;
        earth.pose.position.y = 0;
        earth.pose.position.z = 0;
        earth.pose.orientation.w = 1.0;

        // Earth radius: 6,371 km (scaled for visibility)
        double earth_radius = 6371000.0 * 2.0;  // diameter
        earth.scale.x = earth_radius;
        earth.scale.y = earth_radius;
        earth.scale.z = earth_radius;

        // Blue color
        earth.color.r = 0.2;
        earth.color.g = 0.4;
        earth.color.b = 1.0;
        earth.color.a = 0.8;

        earth_marker_pub_->publish(earth);
    }

    void publishScaledVisualization()
    {
        // Scale factor: 1:1,000,000 (6778km orbit -> 6.778m)
        const double SCALE = 1.0e-6;
        const double mu = 3.986004418e14;
        const int num_points = 100;

        // Publish orbit_view TF frame
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "orbit_view";
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf);

        // Publish scaled Earth marker
        visualization_msgs::msg::Marker earth;
        earth.header.stamp = this->now();
        earth.header.frame_id = "orbit_view";
        earth.ns = "earth";
        earth.id = 0;
        earth.type = visualization_msgs::msg::Marker::SPHERE;
        earth.action = visualization_msgs::msg::Marker::ADD;
        earth.pose.orientation.w = 1.0;
        double earth_diameter = 6371000.0 * 2.0 * SCALE;  // ~12.7m
        earth.scale.x = earth_diameter;
        earth.scale.y = earth_diameter;
        earth.scale.z = earth_diameter;
        earth.color.r = 0.2;
        earth.color.g = 0.4;
        earth.color.b = 1.0;
        earth.color.a = 0.8;
        viz_earth_pub_->publish(earth);

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t sat_idx = 0; sat_idx < satellites_.size(); ++sat_idx)
        {
            auto& sat = satellites_[sat_idx];
            nav_msgs::msg::Path path;
            path.header.stamp = this->now();
            path.header.frame_id = "orbit_view";

            // Generate scaled orbit path
            for (int i = 0; i <= num_points; ++i)
            {
                double M = 2.0 * M_PI * i / num_points;
                double E = M;
                for (int j = 0; j < 10; ++j)
                    E = M + sat.eccentricity * std::sin(E);

                double nu = 2.0 * std::atan2(
                    std::sqrt(1.0 + sat.eccentricity) * std::sin(E / 2.0),
                    std::sqrt(1.0 - sat.eccentricity) * std::cos(E / 2.0));

                double r = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E));
                double x_orb = r * std::cos(nu);
                double y_orb = r * std::sin(nu);

                double cos_O = std::cos(sat.raan);
                double sin_O = std::sin(sat.raan);
                double cos_i = std::cos(sat.inclination);
                double sin_i = std::sin(sat.inclination);
                double cos_w = std::cos(sat.arg_perigee);
                double sin_w = std::sin(sat.arg_perigee);

                double x = (cos_O * cos_w - sin_O * sin_w * cos_i) * x_orb +
                           (-cos_O * sin_w - sin_O * cos_w * cos_i) * y_orb;
                double y = (sin_O * cos_w + cos_O * sin_w * cos_i) * x_orb +
                           (-sin_O * sin_w + cos_O * cos_w * cos_i) * y_orb;
                double z = (sin_w * sin_i) * x_orb + (cos_w * sin_i) * y_orb;

                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = x * SCALE;
                pose.pose.position.y = y * SCALE;
                pose.pose.position.z = z * SCALE;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }

            // Publish scaled path
            if (sat.id == "CHASER")
                viz_chaser_path_pub_->publish(path);
            else
                viz_target_path_pub_->publish(path);

            // Get current position
            double n = std::sqrt(mu / (sat.semi_major_axis * sat.semi_major_axis * sat.semi_major_axis));
            double M_curr = sat.mean_anomaly + n * simulation_time_;
            double E_curr = M_curr;
            for (int j = 0; j < 10; ++j)
                E_curr = M_curr + sat.eccentricity * std::sin(E_curr);

            double nu_curr = 2.0 * std::atan2(
                std::sqrt(1.0 + sat.eccentricity) * std::sin(E_curr / 2.0),
                std::sqrt(1.0 - sat.eccentricity) * std::cos(E_curr / 2.0));

            double r_curr = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E_curr));
            double x_orb = r_curr * std::cos(nu_curr);
            double y_orb = r_curr * std::sin(nu_curr);

            double cos_O = std::cos(sat.raan);
            double sin_O = std::sin(sat.raan);
            double cos_i = std::cos(sat.inclination);
            double sin_i = std::sin(sat.inclination);
            double cos_w = std::cos(sat.arg_perigee);
            double sin_w = std::sin(sat.arg_perigee);

            double pos_x = ((cos_O * cos_w - sin_O * sin_w * cos_i) * x_orb +
                           (-cos_O * sin_w - sin_O * cos_w * cos_i) * y_orb) * SCALE;
            double pos_y = ((sin_O * cos_w + cos_O * sin_w * cos_i) * x_orb +
                           (-sin_O * sin_w + cos_O * cos_w * cos_i) * y_orb) * SCALE;
            double pos_z = ((sin_w * sin_i) * x_orb + (cos_w * sin_i) * y_orb) * SCALE;

            // Create scaled satellite body marker (CUBE representing CubeSat)
            visualization_msgs::msg::Marker body_marker;
            body_marker.header.stamp = this->now();
            body_marker.header.frame_id = "orbit_view";
            body_marker.ns = "satellite_bodies";
            body_marker.id = sat_idx;
            body_marker.type = visualization_msgs::msg::Marker::CUBE;
            body_marker.action = visualization_msgs::msg::Marker::ADD;
            body_marker.pose.position.x = pos_x;
            body_marker.pose.position.y = pos_y;
            body_marker.pose.position.z = pos_z;
            body_marker.pose.orientation.w = 1.0;

            // CubeSat scaled size (for visibility at orbit scale)
            // Model scale: 100x larger than 1:1,000,000 would give for visibility
            // 6U = 0.1m x 0.2m x 0.3m -> 0.01m x 0.02m x 0.03m at orbit view
            // But that's too small, so use 0.05m base for visibility
            if (sat.id == "CHASER") {
                // 6U CubeSat: 0.1 x 0.2 x 0.3 -> scaled for visibility
                body_marker.scale.x = 0.05;
                body_marker.scale.y = 0.10;
                body_marker.scale.z = 0.15;
                body_marker.color.r = 0.3; body_marker.color.g = 0.8; body_marker.color.b = 0.3;
            } else {
                // 3U CubeSat: 0.1 x 0.1 x 0.3 -> scaled for visibility
                body_marker.scale.x = 0.05;
                body_marker.scale.y = 0.05;
                body_marker.scale.z = 0.15;
                body_marker.color.r = 0.8; body_marker.color.g = 0.5; body_marker.color.b = 0.2;
            }
            body_marker.color.a = 1.0;
            marker_array.markers.push_back(body_marker);

            // Create solar panel markers
            visualization_msgs::msg::Marker panel_marker;
            panel_marker.header.stamp = this->now();
            panel_marker.header.frame_id = "orbit_view";
            panel_marker.ns = "solar_panels";
            panel_marker.id = sat_idx * 2;  // Left panel
            panel_marker.type = visualization_msgs::msg::Marker::CUBE;
            panel_marker.action = visualization_msgs::msg::Marker::ADD;
            panel_marker.pose.position.x = pos_x;
            panel_marker.pose.position.y = pos_y + 0.15;  // Offset for panel
            panel_marker.pose.position.z = pos_z;
            panel_marker.pose.orientation.w = 1.0;
            panel_marker.scale.x = 0.12;
            panel_marker.scale.y = 0.08;
            panel_marker.scale.z = 0.01;
            panel_marker.color.r = 0.1; panel_marker.color.g = 0.1; panel_marker.color.b = 0.4;
            panel_marker.color.a = 1.0;
            marker_array.markers.push_back(panel_marker);

            // Right panel
            visualization_msgs::msg::Marker panel_marker2 = panel_marker;
            panel_marker2.id = sat_idx * 2 + 1;
            panel_marker2.pose.position.y = pos_y - 0.15;
            marker_array.markers.push_back(panel_marker2);

            // Add robot arm for CHASER (simplified representation)
            if (sat.id == "CHASER") {
                visualization_msgs::msg::Marker arm_marker;
                arm_marker.header.stamp = this->now();
                arm_marker.header.frame_id = "orbit_view";
                arm_marker.ns = "robot_arm";
                arm_marker.id = sat_idx;
                arm_marker.type = visualization_msgs::msg::Marker::CYLINDER;
                arm_marker.action = visualization_msgs::msg::Marker::ADD;
                arm_marker.pose.position.x = pos_x + 0.12;
                arm_marker.pose.position.y = pos_y;
                arm_marker.pose.position.z = pos_z;
                // Rotate to point along X axis
                arm_marker.pose.orientation.x = 0.0;
                arm_marker.pose.orientation.y = 0.707;
                arm_marker.pose.orientation.z = 0.0;
                arm_marker.pose.orientation.w = 0.707;
                arm_marker.scale.x = 0.02;  // diameter
                arm_marker.scale.y = 0.02;
                arm_marker.scale.z = 0.2;   // length
                arm_marker.color.r = 0.7; arm_marker.color.g = 0.7; arm_marker.color.b = 0.7;
                arm_marker.color.a = 1.0;
                marker_array.markers.push_back(arm_marker);
            }
        }

        viz_marker_pub_->publish(marker_array);
    }

    void publishLVLHVisualization()
    {
        // LVLH (Local Vertical Local Horizontal) frame visualization
        // Shows Target position relative to Chaser - perfect for verifying rendezvous
        // R = radial (Earth center to Chaser, outward positive)
        // S = along-track (velocity direction, positive forward)
        // W = cross-track (completes right-hand system)

        const double mu = 3.986004418e14;

        // Get current positions of both satellites
        struct SatPosition {
            double x, y, z;  // ECI position
            double vx, vy, vz;  // ECI velocity
        };
        SatPosition chaser_pos{}, target_pos{};

        for (auto& sat : satellites_) {
            double n = std::sqrt(mu / (sat.semi_major_axis * sat.semi_major_axis * sat.semi_major_axis));
            double M_curr = sat.mean_anomaly + n * simulation_time_;
            double E_curr = M_curr;
            for (int j = 0; j < 10; ++j)
                E_curr = M_curr + sat.eccentricity * std::sin(E_curr);

            double nu_curr = 2.0 * std::atan2(
                std::sqrt(1.0 + sat.eccentricity) * std::sin(E_curr / 2.0),
                std::sqrt(1.0 - sat.eccentricity) * std::cos(E_curr / 2.0));

            double r = sat.semi_major_axis * (1.0 - sat.eccentricity * std::cos(E_curr));
            double x_orb = r * std::cos(nu_curr);
            double y_orb = r * std::sin(nu_curr);

            // Velocity in orbital plane
            double h = std::sqrt(mu * sat.semi_major_axis * (1.0 - sat.eccentricity * sat.eccentricity));
            double vx_orb = -mu / h * std::sin(nu_curr);
            double vy_orb = mu / h * (sat.eccentricity + std::cos(nu_curr));

            double cos_O = std::cos(sat.raan);
            double sin_O = std::sin(sat.raan);
            double cos_i = std::cos(sat.inclination);
            double sin_i = std::sin(sat.inclination);
            double cos_w = std::cos(sat.arg_perigee);
            double sin_w = std::sin(sat.arg_perigee);

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

            if (sat.id == "CHASER") {
                chaser_pos = {x, y, z, vx, vy, vz};
            } else {
                target_pos = {x, y, z, vx, vy, vz};
            }
        }

        // Calculate relative position (Target relative to Chaser)
        double rel_x = target_pos.x - chaser_pos.x;
        double rel_y = target_pos.y - chaser_pos.y;
        double rel_z = target_pos.z - chaser_pos.z;

        // Calculate LVLH frame from Chaser position/velocity
        // R = radial unit vector (position direction)
        double r_mag = std::sqrt(chaser_pos.x * chaser_pos.x +
                                 chaser_pos.y * chaser_pos.y +
                                 chaser_pos.z * chaser_pos.z);
        double R_x = chaser_pos.x / r_mag;
        double R_y = chaser_pos.y / r_mag;
        double R_z = chaser_pos.z / r_mag;

        // W = cross-track (r x v direction)
        double h_x = chaser_pos.y * chaser_pos.vz - chaser_pos.z * chaser_pos.vy;
        double h_y = chaser_pos.z * chaser_pos.vx - chaser_pos.x * chaser_pos.vz;
        double h_z = chaser_pos.x * chaser_pos.vy - chaser_pos.y * chaser_pos.vx;
        double h_mag = std::sqrt(h_x * h_x + h_y * h_y + h_z * h_z);
        double W_x = h_x / h_mag;
        double W_y = h_y / h_mag;
        double W_z = h_z / h_mag;

        // S = along-track (W x R direction)
        double S_x = W_y * R_z - W_z * R_y;
        double S_y = W_z * R_x - W_x * R_z;
        double S_z = W_x * R_y - W_y * R_x;

        // Transform relative position to LVLH frame
        double lvlh_r = rel_x * R_x + rel_y * R_y + rel_z * R_z;  // radial
        double lvlh_s = rel_x * S_x + rel_y * S_y + rel_z * S_z;  // along-track
        double lvlh_w = rel_x * W_x + rel_y * W_y + rel_z * W_z;  // cross-track

        // Calculate distance
        double distance = std::sqrt(rel_x * rel_x + rel_y * rel_y + rel_z * rel_z);

        // Publish LVLH TF frame
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "world";
        tf.child_frame_id = "lvlh";
        tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf);

        // Scale for visualization: 1m display = 100m real distance
        // This makes 1500 km = 15 km in display (still large, let's use 1:10000)
        const double LVLH_SCALE = 1.0e-4;  // 1:10000 scale -> 1500km = 150m

        // Create marker array
        visualization_msgs::msg::MarkerArray marker_array;

        // Chaser at origin (green cube)
        visualization_msgs::msg::Marker chaser_marker;
        chaser_marker.header.stamp = this->now();
        chaser_marker.header.frame_id = "lvlh";
        chaser_marker.ns = "lvlh_satellites";
        chaser_marker.id = 0;
        chaser_marker.type = visualization_msgs::msg::Marker::CUBE;
        chaser_marker.action = visualization_msgs::msg::Marker::ADD;
        chaser_marker.pose.position.x = 0;
        chaser_marker.pose.position.y = 0;
        chaser_marker.pose.position.z = 0;
        chaser_marker.pose.orientation.w = 1.0;
        chaser_marker.scale.x = 0.3;
        chaser_marker.scale.y = 0.6;
        chaser_marker.scale.z = 0.9;
        chaser_marker.color.r = 0.2; chaser_marker.color.g = 0.8; chaser_marker.color.b = 0.2;
        chaser_marker.color.a = 1.0;
        marker_array.markers.push_back(chaser_marker);

        // Target relative to Chaser (orange cube)
        visualization_msgs::msg::Marker target_marker;
        target_marker.header.stamp = this->now();
        target_marker.header.frame_id = "lvlh";
        target_marker.ns = "lvlh_satellites";
        target_marker.id = 1;
        target_marker.type = visualization_msgs::msg::Marker::CUBE;
        target_marker.action = visualization_msgs::msg::Marker::ADD;
        // In LVLH: X = along-track (S), Y = cross-track (W), Z = radial (R)
        target_marker.pose.position.x = lvlh_s * LVLH_SCALE;  // along-track
        target_marker.pose.position.y = lvlh_w * LVLH_SCALE;  // cross-track
        target_marker.pose.position.z = lvlh_r * LVLH_SCALE;  // radial
        target_marker.pose.orientation.w = 1.0;
        target_marker.scale.x = 0.3;
        target_marker.scale.y = 0.3;
        target_marker.scale.z = 0.9;
        target_marker.color.r = 1.0; target_marker.color.g = 0.5; target_marker.color.b = 0.0;
        target_marker.color.a = 1.0;
        marker_array.markers.push_back(target_marker);

        // Add axis labels
        // X axis (along-track/S) - red
        visualization_msgs::msg::Marker x_axis;
        x_axis.header.stamp = this->now();
        x_axis.header.frame_id = "lvlh";
        x_axis.ns = "lvlh_axes";
        x_axis.id = 0;
        x_axis.type = visualization_msgs::msg::Marker::ARROW;
        x_axis.action = visualization_msgs::msg::Marker::ADD;
        x_axis.pose.orientation.w = 1.0;
        x_axis.scale.x = 50.0;  // length
        x_axis.scale.y = 0.5;   // width
        x_axis.scale.z = 0.5;   // height
        x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0;
        x_axis.color.a = 0.8;
        marker_array.markers.push_back(x_axis);

        // Add line from Chaser to Target
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = this->now();
        line_marker.header.frame_id = "lvlh";
        line_marker.ns = "lvlh_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        line_marker.scale.x = 0.1;  // line width
        geometry_msgs::msg::Point p1, p2;
        p1.x = 0; p1.y = 0; p1.z = 0;
        p2.x = lvlh_s * LVLH_SCALE;
        p2.y = lvlh_w * LVLH_SCALE;
        p2.z = lvlh_r * LVLH_SCALE;
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.color.r = 1.0; line_marker.color.g = 1.0; line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        marker_array.markers.push_back(line_marker);

        lvlh_marker_pub_->publish(marker_array);

        // Distance text marker
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.stamp = this->now();
        text_marker.header.frame_id = "lvlh";
        text_marker.ns = "lvlh_text";
        text_marker.id = 0;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = lvlh_s * LVLH_SCALE / 2.0;
        text_marker.pose.position.y = lvlh_w * LVLH_SCALE / 2.0 + 5.0;
        text_marker.pose.position.z = lvlh_r * LVLH_SCALE / 2.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 3.0;  // text height

        char dist_text[128];
        snprintf(dist_text, sizeof(dist_text),
            "Distance: %.2f km\nAlong-track: %.2f km\nRadial: %.2f km\nCross-track: %.2f km",
            distance / 1000.0, lvlh_s / 1000.0, lvlh_r / 1000.0, lvlh_w / 1000.0);
        text_marker.text = dist_text;
        text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        lvlh_distance_pub_->publish(text_marker);

        // Add current position to path history (every 10 seconds of sim time)
        static double last_path_time = 0;
        if (simulation_time_ - last_path_time > 10.0 || lvlh_path_history_.empty()) {
            last_path_time = simulation_time_;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "lvlh";
            pose.pose.position.x = lvlh_s * LVLH_SCALE;
            pose.pose.position.y = lvlh_w * LVLH_SCALE;
            pose.pose.position.z = lvlh_r * LVLH_SCALE;
            pose.pose.orientation.w = 1.0;
            lvlh_path_history_.push_back(pose);
            // Keep last 1000 points
            if (lvlh_path_history_.size() > 1000) {
                lvlh_path_history_.erase(lvlh_path_history_.begin());
            }
        }

        // Publish relative motion path
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "lvlh";
        path.poses = lvlh_path_history_;
        lvlh_path_pub_->publish(path);

        // Log distance every 60 seconds of simulation time
        static double last_dist_log = 0;
        if (propagating_ && (simulation_time_ - last_dist_log > 60.0)) {
            last_dist_log = simulation_time_;
            RCLCPP_INFO(this->get_logger(),
                "LVLH: Distance=%.2f km, Along-track=%.2f km, Radial=%.2f km",
                distance / 1000.0, lvlh_s / 1000.0, lvlh_r / 1000.0);
        }
    }

    std::vector<SatelliteConfig> satellites_;
    rclcpp::Publisher<hpop_msgs::msg::SatelliteState>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr chaser_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr earth_marker_pub_;
    // Scaled visualization publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_chaser_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr viz_target_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr viz_earth_pub_;
    // LVLH relative motion publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lvlh_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lvlh_distance_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lvlh_path_pub_;
    // LVLH path history
    std::vector<geometry_msgs::msg::PoseStamped> lvlh_path_history_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_set_state_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr deltav_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double simulation_time_;
    int last_log_time_ = 0;
    bool propagating_;
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
