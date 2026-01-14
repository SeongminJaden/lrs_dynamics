/**
 * @file gazebo_hpop_bridge_node.cpp
 * @brief Bridge node between Gazebo and HPOP/RViz visualization
 *
 * This node:
 * 1. Subscribes to Gazebo model state (position/velocity)
 * 2. Publishes state to /hpop/satellite_state for RViz visualization
 * 3. Broadcasts TF for satellite positions
 *
 * Note: Gravity is applied by the LeoGravityWorldPlugin in Gazebo directly.
 * This node only reads state and publishes for visualization.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hpop_msgs/msg/satellite_state.hpp>
#include <hpop_msgs/msg/orbital_elements.hpp>

#include <cmath>
#include <memory>
#include <map>
#include <string>
#include <vector>

namespace hpop_gazebo
{

class GazeboHpopBridgeNode : public rclcpp::Node
{
public:
    GazeboHpopBridgeNode()
        : Node("gazebo_hpop_bridge_node")
    {
        // Parameters
        this->declare_parameter("model_prefix", "satellite");
        this->declare_parameter("scale_factor", 0.001);  // For RViz display
        this->declare_parameter("publish_rate", 10.0);

        model_prefix_ = this->get_parameter("model_prefix").as_string();
        scale_factor_ = this->get_parameter("scale_factor").as_double();

        // Subscribe to Gazebo model states
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&GazeboHpopBridgeNode::modelStatesCallback, this, std::placeholders::_1));

        // Publisher for HPOP satellite state (RViz visualization)
        state_pub_ = this->create_publisher<hpop_msgs::msg::SatelliteState>(
            "/hpop/satellite_state", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for periodic updates when no Gazebo data
        double rate = this->get_parameter("publish_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&GazeboHpopBridgeNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Gazebo-HPOP Bridge Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /gazebo/model_states");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /hpop/satellite_state");
        RCLCPP_INFO(this->get_logger(), "Model prefix filter: '%s'", model_prefix_.c_str());
    }

private:
    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        last_gazebo_update_ = this->now();

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const std::string& name = msg->name[i];

            // Filter by prefix
            if (!model_prefix_.empty() && name.find(model_prefix_) == std::string::npos)
            {
                continue;
            }

            // Skip ground_plane and other static models
            if (name == "ground_plane" || name == "sun" || name == "earth")
            {
                continue;
            }

            // Get position and velocity
            double x = msg->pose[i].position.x;
            double y = msg->pose[i].position.y;
            double z = msg->pose[i].position.z;

            double vx = msg->twist[i].linear.x;
            double vy = msg->twist[i].linear.y;
            double vz = msg->twist[i].linear.z;

            // Store state
            SatelliteState state;
            state.name = name;
            state.x = x; state.y = y; state.z = z;
            state.vx = vx; state.vy = vy; state.vz = vz;
            state.last_update = this->now();
            satellites_[name] = state;

            // Publish state
            publishSatelliteState(name, x, y, z, vx, vy, vz);

            // Broadcast TF
            broadcastTransform(name, x, y, z);
        }
    }

    void timerCallback()
    {
        // Check if we have Gazebo data
        auto now = this->now();
        bool has_gazebo_data = (last_gazebo_update_.nanoseconds() > 0 &&
                                (now - last_gazebo_update_).seconds() < 1.0);

        if (!has_gazebo_data && satellites_.empty())
        {
            // No Gazebo data - run demo mode
            runDemoMode();
        }
        else if (has_gazebo_data)
        {
            // Re-publish existing satellite states for consistent visualization
            for (const auto& [name, state] : satellites_)
            {
                if ((now - state.last_update).seconds() < 0.5)
                {
                    broadcastTransform(name, state.x, state.y, state.z);
                }
            }
        }
    }

    void publishSatelliteState(const std::string& name,
                               double x, double y, double z,
                               double vx, double vy, double vz)
    {
        auto msg = hpop_msgs::msg::SatelliteState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "earth";
        msg.satellite_id = name;
        msg.name = name;
        msg.norad_id = 99999;  // Demo ID

        // Position in meters
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;

        // Velocity in m/s
        msg.velocity.x = vx;
        msg.velocity.y = vy;
        msg.velocity.z = vz;

        // Calculate orbital elements
        calculateOrbitalElements(msg, x, y, z, vx, vy, vz);

        // Physical properties (default)
        msg.mass = 500.0;
        msg.cross_section_drag = 10.0;
        msg.drag_coefficient = 2.2;
        msg.reflectivity_coeff = 1.3;

        state_pub_->publish(msg);
    }

    void calculateOrbitalElements(hpop_msgs::msg::SatelliteState& msg,
                                  double x, double y, double z,
                                  double vx, double vy, double vz)
    {
        const double mu = 3.986004418e14;  // Earth GM [m^3/s^2]

        // Position and velocity magnitudes
        double r = std::sqrt(x*x + y*y + z*z);
        double v = std::sqrt(vx*vx + vy*vy + vz*vz);

        // Specific angular momentum
        double hx = y * vz - z * vy;
        double hy = z * vx - x * vz;
        double hz = x * vy - y * vx;
        double h = std::sqrt(hx*hx + hy*hy + hz*hz);

        // Semi-major axis
        double energy = 0.5 * v * v - mu / r;
        double a = -mu / (2.0 * energy);

        // Eccentricity vector
        double ev_x = (v*v - mu/r) * x / mu - (x*vx + y*vy + z*vz) * vx / mu;
        double ev_y = (v*v - mu/r) * y / mu - (x*vx + y*vy + z*vz) * vy / mu;
        double ev_z = (v*v - mu/r) * z / mu - (x*vx + y*vy + z*vz) * vz / mu;
        double e = std::sqrt(ev_x*ev_x + ev_y*ev_y + ev_z*ev_z);

        // Inclination
        double i = std::acos(hz / h);

        // RAAN
        double n_x = -hy;
        double n_y = hx;
        double n_mag = std::sqrt(n_x*n_x + n_y*n_y);
        double raan = 0.0;
        if (n_mag > 1e-10)
        {
            raan = std::acos(n_x / n_mag);
            if (n_y < 0) raan = 2.0 * M_PI - raan;
        }

        // Argument of perigee
        double omega = 0.0;
        if (e > 1e-10 && n_mag > 1e-10)
        {
            double dot_n_e = n_x * ev_x + n_y * ev_y;
            omega = std::acos(dot_n_e / (n_mag * e));
            if (ev_z < 0) omega = 2.0 * M_PI - omega;
        }

        // True anomaly
        double nu = 0.0;
        if (e > 1e-10)
        {
            double dot_e_r = ev_x * x + ev_y * y + ev_z * z;
            nu = std::acos(dot_e_r / (e * r));
            double rdotv = x * vx + y * vy + z * vz;
            if (rdotv < 0) nu = 2.0 * M_PI - nu;
        }

        // Fill message (radians)
        msg.elements.semi_major_axis = a;
        msg.elements.eccentricity = e;
        msg.elements.inclination = i;
        msg.elements.raan = raan;
        msg.elements.arg_periapsis = omega;
        msg.elements.true_anomaly = nu;
    }

    void broadcastTransform(const std::string& name, double x, double y, double z)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "earth";
        t.child_frame_id = "satellite_" + name;

        // Scale for visualization
        t.transform.translation.x = x * scale_factor_;
        t.transform.translation.y = y * scale_factor_;
        t.transform.translation.z = z * scale_factor_;

        t.transform.rotation.w = 1.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;

        tf_broadcaster_->sendTransform(t);
    }

    void runDemoMode()
    {
        // Demo simulation when no Gazebo data
        static double sim_time = 0.0;
        sim_time += 0.1;

        // Simulate ISS-like orbit
        const double mu = 3.986004418e14;
        double sma = 6.778e6;  // 400 km altitude
        double inc = 51.64 * M_PI / 180.0;

        double n = std::sqrt(mu / (sma * sma * sma));
        double M = n * sim_time * 100.0;  // 100x time scale

        // Kepler's equation (circular approximation)
        double r = sma;
        double x = r * std::cos(M);
        double y = r * std::sin(M) * std::cos(inc);
        double z = r * std::sin(M) * std::sin(inc);

        double v = std::sqrt(mu / r);
        double vx = -v * std::sin(M);
        double vy = v * std::cos(M) * std::cos(inc);
        double vz = v * std::cos(M) * std::sin(inc);

        publishSatelliteState("DEMO_ISS", x, y, z, vx, vy, vz);
        broadcastTransform("DEMO_ISS", x, y, z);
    }

    struct SatelliteState
    {
        std::string name;
        double x, y, z;
        double vx, vy, vz;
        rclcpp::Time last_update;
    };

    // ROS2 interfaces
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Publisher<hpop_msgs::msg::SatelliteState>::SharedPtr state_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string model_prefix_;
    double scale_factor_;

    // State
    std::map<std::string, SatelliteState> satellites_;
    rclcpp::Time last_gazebo_update_;
};

}  // namespace hpop_gazebo

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hpop_gazebo::GazeboHpopBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
