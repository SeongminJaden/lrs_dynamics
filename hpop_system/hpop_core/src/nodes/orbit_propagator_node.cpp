#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "hpop_msgs/msg/satellite_state.hpp"
#include "hpop_msgs/msg/propagator_status.hpp"
#include "hpop_msgs/msg/orbital_elements.hpp"
#include "hpop_msgs/srv/add_satellite.hpp"
#include "hpop_msgs/srv/remove_satellite.hpp"
#include "hpop_msgs/srv/set_perturbations.hpp"
#include "hpop_msgs/action/propagate_orbit.hpp"

#include "hpop_core/propagator.hpp"
#include "hpop_core/state_vector.hpp"
#include "hpop_core/coordinate_frames.hpp"
#include "hpop_core/time_system.hpp"
#include "hpop_core/constants.hpp"

#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>

namespace hpop_core
{

/// Satellite data structure
struct SatelliteData
{
    std::string id;
    std::string name;
    uint32_t norad_id{0};
    StateVector state;
    SatelliteConfig config;
    bool active{true};
};

class OrbitPropagatorNode : public rclcpp::Node
{
public:
    using AddSatellite = hpop_msgs::srv::AddSatellite;
    using RemoveSatellite = hpop_msgs::srv::RemoveSatellite;
    using SetPerturbations = hpop_msgs::srv::SetPerturbations;
    using PropagateOrbit = hpop_msgs::action::PropagateOrbit;
    using GoalHandlePropagate = rclcpp_action::ServerGoalHandle<PropagateOrbit>;

    OrbitPropagatorNode()
        : Node("orbit_propagator_node")
    {
        // Declare parameters
        this->declare_parameter("update_rate", 10.0);
        this->declare_parameter("integrator", "RKF78");
        this->declare_parameter("step_size", 60.0);
        this->declare_parameter("tolerance", 1e-10);
        this->declare_parameter("gravity_degree", 20);
        this->declare_parameter("enable_j2", true);
        this->declare_parameter("enable_drag", false);
        this->declare_parameter("enable_srp", false);
        this->declare_parameter("enable_third_body", false);

        // Get parameters
        double update_rate = this->get_parameter("update_rate").as_double();
        std::string integrator = this->get_parameter("integrator").as_string();
        double step_size = this->get_parameter("step_size").as_double();
        double tolerance = this->get_parameter("tolerance").as_double();
        enable_j2_ = this->get_parameter("enable_j2").as_bool();

        // Configure propagator
        if (integrator == "RK4")
            propagator_.setIntegrator(IntegratorType::RK4);
        else
            propagator_.setIntegrator(IntegratorType::RKF78);

        propagator_.setStepSize(step_size);
        propagator_.setTolerance(tolerance);

        // Add J2 perturbation if enabled
        if (enable_j2_)
        {
            propagator_.addForceModel("J2", [](const Vec3& pos, const Vec3&, double) -> Vec3 {
                double r = pos.norm();
                double r2 = r * r;
                double z2 = pos.z * pos.z;

                double factor = -1.5 * constants::EARTH_MU * constants::EARTH_J2
                               * constants::EARTH_RADIUS_EQ * constants::EARTH_RADIUS_EQ / (r2 * r2 * r);

                double coef = 5.0 * z2 / r2 - 1.0;
                return Vec3{
                    factor * pos.x * coef,
                    factor * pos.y * coef,
                    factor * pos.z * (5.0 * z2 / r2 - 3.0)
                };
            });
        }

        // Publishers
        state_pub_ = this->create_publisher<hpop_msgs::msg::SatelliteState>(
            "/hpop/states", 10);
        status_pub_ = this->create_publisher<hpop_msgs::msg::PropagatorStatus>(
            "/hpop/status", 10);

        // Services
        add_satellite_srv_ = this->create_service<AddSatellite>(
            "/hpop/add_satellite",
            std::bind(&OrbitPropagatorNode::addSatelliteCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        remove_satellite_srv_ = this->create_service<RemoveSatellite>(
            "/hpop/remove_satellite",
            std::bind(&OrbitPropagatorNode::removeSatelliteCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        set_perturbations_srv_ = this->create_service<SetPerturbations>(
            "/hpop/set_perturbations",
            std::bind(&OrbitPropagatorNode::setPerturbationsCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Action server
        propagate_action_server_ = rclcpp_action::create_server<PropagateOrbit>(
            this,
            "/hpop/propagate",
            std::bind(&OrbitPropagatorNode::handleGoal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&OrbitPropagatorNode::handleCancel, this,
                     std::placeholders::_1),
            std::bind(&OrbitPropagatorNode::handleAccepted, this,
                     std::placeholders::_1));

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for real-time propagation
        auto period = std::chrono::duration<double>(1.0 / update_rate);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&OrbitPropagatorNode::propagationLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "Orbit Propagator Node initialized (integrator=%s, J2=%s)",
            integrator.c_str(), enable_j2_ ? "enabled" : "disabled");
    }

private:
    //=========================================================================
    // Service Callbacks
    //=========================================================================

    void addSatelliteCallback(
        const std::shared_ptr<AddSatellite::Request> request,
        std::shared_ptr<AddSatellite::Response> response)
    {
        std::lock_guard<std::mutex> lock(satellites_mutex_);

        SatelliteData sat;
        sat.id = request->satellite_id;
        sat.name = request->name.empty() ? request->satellite_id : request->name;
        sat.norad_id = request->norad_id;

        // Set physical properties
        sat.config.id = sat.id;
        sat.config.name = sat.name;
        sat.config.norad_id = sat.norad_id;
        sat.config.mass = request->mass > 0 ? request->mass : 1000.0;
        sat.config.area_drag = request->cross_section_drag > 0 ? request->cross_section_drag : 10.0;
        sat.config.area_srp = request->cross_section_srp > 0 ? request->cross_section_srp : 10.0;
        sat.config.cd = request->drag_coefficient > 0 ? request->drag_coefficient : 2.2;
        sat.config.cr = request->reflectivity_coeff > 0 ? request->reflectivity_coeff : 1.5;

        // Initialize state from TLE or elements or Cartesian
        if (!request->tle.line1.empty())
        {
            // TODO: Parse TLE and convert to state
            response->success = false;
            response->message = "TLE parsing not yet implemented";
            return;
        }
        else if (request->elements.semi_major_axis > 0)
        {
            // Initialize from orbital elements
            OrbitalElements oe;
            oe.sma = request->elements.semi_major_axis;
            oe.ecc = request->elements.eccentricity;
            oe.inc = request->elements.inclination;
            oe.raan = request->elements.raan;
            oe.aop = request->elements.arg_periapsis;
            oe.ta = request->elements.true_anomaly;

            double epoch_sec = request->epoch.sec + request->epoch.nanosec * 1e-9;
            oe.epoch = TimeSystem::unixToJD(epoch_sec);
            if (oe.epoch < 2400000)  // Invalid epoch, use current time
                oe.epoch = TimeSystem::currentJD();

            sat.state = oe.toStateVector();
            sat.state.epoch = oe.epoch;
        }
        else if (request->position.x != 0 || request->position.y != 0 || request->position.z != 0)
        {
            // Initialize from Cartesian state
            sat.state.position = Vec3{request->position.x, request->position.y, request->position.z};
            sat.state.velocity = Vec3{request->velocity.x, request->velocity.y, request->velocity.z};

            double epoch_sec = request->epoch.sec + request->epoch.nanosec * 1e-9;
            sat.state.epoch = TimeSystem::unixToJD(epoch_sec);
            if (sat.state.epoch < 2400000)
                sat.state.epoch = TimeSystem::currentJD();
        }
        else
        {
            response->success = false;
            response->message = "No valid initial state provided";
            return;
        }

        satellites_[sat.id] = sat;

        response->success = true;
        response->message = "Satellite added successfully";
        response->satellite_id = sat.id;
        response->initial_state = stateToMsg(sat);

        RCLCPP_INFO(this->get_logger(), "Added satellite: %s", sat.id.c_str());
    }

    void removeSatelliteCallback(
        const std::shared_ptr<RemoveSatellite::Request> request,
        std::shared_ptr<RemoveSatellite::Response> response)
    {
        std::lock_guard<std::mutex> lock(satellites_mutex_);

        if (request->remove_all)
        {
            for (const auto& [id, _] : satellites_)
                response->removed_ids.push_back(id);
            satellites_.clear();
            response->success = true;
            response->message = "All satellites removed";
            RCLCPP_INFO(this->get_logger(), "Removed all satellites");
        }
        else
        {
            auto it = satellites_.find(request->satellite_id);
            if (it != satellites_.end())
            {
                response->removed_ids.push_back(it->first);
                satellites_.erase(it);
                response->success = true;
                response->message = "Satellite removed";
                RCLCPP_INFO(this->get_logger(), "Removed satellite: %s",
                           request->satellite_id.c_str());
            }
            else
            {
                response->success = false;
                response->message = "Satellite not found";
            }
        }
    }

    void setPerturbationsCallback(
        const std::shared_ptr<SetPerturbations::Request> request,
        std::shared_ptr<SetPerturbations::Response> response)
    {
        // Clear existing perturbation models
        propagator_.clearForceModels();

        // Add J2 if enabled
        if (request->gravity_enabled && request->gravity_degree >= 2)
        {
            propagator_.addForceModel("J2", [](const Vec3& pos, const Vec3&, double) -> Vec3 {
                double r = pos.norm();
                double r2 = r * r;
                double z2 = pos.z * pos.z;

                double factor = -1.5 * constants::EARTH_MU * constants::EARTH_J2
                               * constants::EARTH_RADIUS_EQ * constants::EARTH_RADIUS_EQ / (r2 * r2 * r);

                double coef = 5.0 * z2 / r2 - 1.0;
                return Vec3{
                    factor * pos.x * coef,
                    factor * pos.y * coef,
                    factor * pos.z * (5.0 * z2 / r2 - 3.0)
                };
            });
            enable_j2_ = true;
        }
        else
        {
            enable_j2_ = false;
        }

        // TODO: Add drag, SRP, third body models

        response->success = true;
        response->message = "Perturbations updated";
        response->gravity_enabled = request->gravity_enabled;
        response->gravity_degree = request->gravity_degree;
        response->drag_enabled = request->drag_enabled;
        response->srp_enabled = request->srp_enabled;
        response->third_body_moon_enabled = request->third_body_moon_enabled;
        response->third_body_sun_enabled = request->third_body_sun_enabled;
        response->solid_tides_enabled = request->solid_tides_enabled;

        RCLCPP_INFO(this->get_logger(), "Perturbations updated: J2=%s, drag=%s, SRP=%s",
                   enable_j2_ ? "on" : "off",
                   request->drag_enabled ? "on" : "off",
                   request->srp_enabled ? "on" : "off");
    }

    //=========================================================================
    // Action Server Callbacks
    //=========================================================================

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const PropagateOrbit::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received propagation goal for satellite: %s",
                   goal->satellite_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandlePropagate>)
    {
        RCLCPP_INFO(this->get_logger(), "Propagation cancelled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePropagate> goal_handle)
    {
        std::thread{std::bind(&OrbitPropagatorNode::executePropagation, this, goal_handle)}.detach();
    }

    void executePropagation(const std::shared_ptr<GoalHandlePropagate> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PropagateOrbit::Result>();
        auto feedback = std::make_shared<PropagateOrbit::Feedback>();

        // Find satellite
        SatelliteData sat;
        {
            std::lock_guard<std::mutex> lock(satellites_mutex_);
            auto it = satellites_.find(goal->satellite_id);
            if (it == satellites_.end())
            {
                result->success = false;
                result->message = "Satellite not found";
                goal_handle->abort(result);
                return;
            }
            sat = it->second;
        }

        // Calculate propagation duration
        double duration = goal->duration_seconds;
        if (duration <= 0 && goal->target_time.sec > 0)
        {
            double target_jd = TimeSystem::unixToJD(
                goal->target_time.sec + goal->target_time.nanosec * 1e-9);
            duration = (target_jd - sat.state.epoch) * constants::SECONDS_PER_DAY;
        }

        if (duration <= 0)
        {
            result->success = false;
            result->message = "Invalid propagation duration";
            goal_handle->abort(result);
            return;
        }

        // Propagate
        std::vector<PropagationStep> trajectory;
        auto start_time = std::chrono::high_resolution_clock::now();

        StateVector current = sat.state;
        double output_step = goal->output_step > 0 ? goal->output_step : 60.0;
        double t = 0;

        while (t < duration)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "Propagation cancelled";
                goal_handle->canceled(result);
                return;
            }

            double dt = std::min(output_step, duration - t);
            auto step = propagator_.singleStep(current, dt);
            current = step.state;
            t += dt;

            trajectory.push_back(step);

            // Send feedback
            feedback->current_state = stateVectorToMsg(current, sat.id);
            feedback->progress_percent = (t / duration) * 100.0;
            feedback->elapsed_sim_time = t;
            feedback->remaining_sim_time = duration - t;
            goal_handle->publish_feedback(feedback);

            if (goal->real_time)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(dt * 1000)));
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Build result
        result->success = true;
        result->message = "Propagation completed";
        result->final_state = stateVectorToMsg(current, sat.id);
        result->num_steps = trajectory.size();
        result->propagation_time_ms = elapsed_ms;

        for (const auto& step : trajectory)
        {
            result->trajectory.push_back(stateVectorToMsg(step.state, sat.id));
        }

        // Update satellite state
        {
            std::lock_guard<std::mutex> lock(satellites_mutex_);
            if (satellites_.count(sat.id))
                satellites_[sat.id].state = current;
        }

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Propagation completed: %zu steps in %.2f ms",
                   trajectory.size(), elapsed_ms);
    }

    //=========================================================================
    // Real-time Propagation Loop
    //=========================================================================

    void propagationLoop()
    {
        std::lock_guard<std::mutex> lock(satellites_mutex_);

        if (satellites_.empty())
            return;

        auto now = this->now();
        double current_jd = TimeSystem::currentJD();

        for (auto& [id, sat] : satellites_)
        {
            if (!sat.active)
                continue;

            // Propagate to current time
            double dt = (current_jd - sat.state.epoch) * constants::SECONDS_PER_DAY;
            if (dt > 0 && dt < 3600)  // Max 1 hour step
            {
                auto step = propagator_.singleStep(sat.state, dt);
                sat.state = step.state;
            }

            // Publish state
            auto msg = stateToMsg(sat);
            msg.header.stamp = now;
            state_pub_->publish(msg);

            // Broadcast TF
            broadcastTF(sat, now);
        }

        // Publish status
        publishStatus();
    }

    void broadcastTF(const SatelliteData& sat, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "earth_eci";
        tf.child_frame_id = sat.id + "_eci";

        // Position (scaled for visualization - km instead of m)
        tf.transform.translation.x = sat.state.position.x / 1000.0;
        tf.transform.translation.y = sat.state.position.y / 1000.0;
        tf.transform.translation.z = sat.state.position.z / 1000.0;

        // Identity rotation for now
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);
    }

    void publishStatus()
    {
        hpop_msgs::msg::PropagatorStatus status;
        status.header.stamp = this->now();

        status.state = running_ ?
            hpop_msgs::msg::PropagatorStatus::STATE_RUNNING :
            hpop_msgs::msg::PropagatorStatus::STATE_STOPPED;

        status.num_satellites = satellites_.size();
        for (const auto& [id, _] : satellites_)
            status.satellite_ids.push_back(id);

        status.integrator_type = propagator_.integratorName();
        status.gravity_enabled = enable_j2_;
        status.gravity_degree = enable_j2_ ? 2 : 0;

        status_pub_->publish(status);
    }

    //=========================================================================
    // Helper Functions
    //=========================================================================

    hpop_msgs::msg::SatelliteState stateToMsg(const SatelliteData& sat)
    {
        return stateVectorToMsg(sat.state, sat.id, sat.name, sat.norad_id, sat.config);
    }

    hpop_msgs::msg::SatelliteState stateVectorToMsg(
        const StateVector& state,
        const std::string& id,
        const std::string& name = "",
        uint32_t norad_id = 0,
        const SatelliteConfig& config = SatelliteConfig())
    {
        hpop_msgs::msg::SatelliteState msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "ECI_J2000";

        msg.satellite_id = id;
        msg.name = name;
        msg.norad_id = norad_id;
        msg.frame_id = "ECI_J2000";

        msg.position.x = state.position.x;
        msg.position.y = state.position.y;
        msg.position.z = state.position.z;

        msg.velocity.x = state.velocity.x;
        msg.velocity.y = state.velocity.y;
        msg.velocity.z = state.velocity.z;

        msg.orientation.w = 1.0;

        // Orbital elements
        OrbitalElements oe = OrbitalElements::fromStateVector(state);
        msg.elements.semi_major_axis = oe.sma;
        msg.elements.eccentricity = oe.ecc;
        msg.elements.inclination = oe.inc;
        msg.elements.raan = oe.raan;
        msg.elements.arg_periapsis = oe.aop;
        msg.elements.true_anomaly = oe.ta;
        msg.elements.mean_anomaly = oe.meanAnomaly();
        msg.elements.period = oe.period();
        msg.elements.mean_motion = oe.meanMotion();
        msg.elements.apoapsis = oe.apoapsis();
        msg.elements.periapsis = oe.periapsis();

        // Physical properties
        msg.mass = config.mass;
        msg.cross_section_drag = config.area_drag;
        msg.cross_section_srp = config.area_srp;
        msg.drag_coefficient = config.cd;
        msg.reflectivity_coeff = config.cr;

        msg.is_valid = true;

        return msg;
    }

    // Members
    OrbitPropagator propagator_;
    std::unordered_map<std::string, SatelliteData> satellites_;
    std::mutex satellites_mutex_;
    std::atomic<bool> running_{true};
    bool enable_j2_{true};

    // ROS2 interfaces
    rclcpp::Publisher<hpop_msgs::msg::SatelliteState>::SharedPtr state_pub_;
    rclcpp::Publisher<hpop_msgs::msg::PropagatorStatus>::SharedPtr status_pub_;
    rclcpp::Service<AddSatellite>::SharedPtr add_satellite_srv_;
    rclcpp::Service<RemoveSatellite>::SharedPtr remove_satellite_srv_;
    rclcpp::Service<SetPerturbations>::SharedPtr set_perturbations_srv_;
    rclcpp_action::Server<PropagateOrbit>::SharedPtr propagate_action_server_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace hpop_core

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hpop_core::OrbitPropagatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
