/**
 * @file rendezvous_controller.cpp
 * @brief Rendezvous and Docking Controller for Satellite Capture Mission
 *
 * This node manages the complete rendezvous scenario:
 * 1. Orbital motion simulation for Chaser and Target satellites
 * 2. Delta-V calculation and execution for approach
 * 3. Proximity operations
 * 4. Robot arm capture sequence
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <hpop_msgs/msg/satellite_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#ifdef MOVEIT2_AVAILABLE
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif

#include <cmath>
#include <vector>
#include <string>
#include <map>

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// Structure to store HPOP satellite state
struct HpopSatelliteState {
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 velocity;
    rclcpp::Time last_update;
};

enum class MissionPhase {
    IDLE,
    ORBITING,           // Satellites orbiting before rendezvous
    RENDEZVOUS_STARTED,
    APPROACH_PHASE,
    PROXIMITY_OPS,
    ARM_CAPTURE,
    DOCKED,
    MISSION_COMPLETE
};

class RendezvousController : public rclcpp::Node
{
public:
    RendezvousController() : Node("rendezvous_controller")
    {
        // Parameters
        this->declare_parameter("approach_distance", 0.5);  // meters
        this->declare_parameter("capture_distance", 0.3);   // meters
        this->declare_parameter("approach_velocity", 0.05); // m/s

        approach_distance_ = this->get_parameter("approach_distance").as_double();
        capture_distance_ = this->get_parameter("capture_distance").as_double();
        approach_velocity_ = this->get_parameter("approach_velocity").as_double();

        // Services
        start_rendezvous_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/rendezvous/start",
            std::bind(&RendezvousController::startRendezvousCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        abort_rendezvous_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/rendezvous/abort",
            std::bind(&RendezvousController::abortRendezvousCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        capture_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/rendezvous/capture",
            std::bind(&RendezvousController::captureCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Publishers
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/rendezvous/status", 10);
        delta_v_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rendezvous/delta_v", 10);
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/rendezvous/distance", 10);

        // Publisher for HPOP delta-v commands
        hpop_deltav_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/hpop/apply_delta_v", 10);

        // Subscribers
        model_states_sub_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&RendezvousController::modelStatesCallback, this, std::placeholders::_1));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RendezvousController::jointStatesCallback, this, std::placeholders::_1));

        // Subscribe to HPOP satellite states
        hpop_state_sub_ = this->create_subscription<hpop_msgs::msg::SatelliteState>(
            "/hpop/satellite_state", 10,
            std::bind(&RendezvousController::hpopStateCallback, this, std::placeholders::_1));

        // Action client for arm control
        arm_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");

        // Gazebo service client for moving satellites
        set_entity_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");

        // TF broadcaster for satellite positions
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for control loop (50Hz for smooth orbital motion)
        control_timer_ = this->create_wall_timer(
            20ms, std::bind(&RendezvousController::controlLoop, this));

#ifdef MOVEIT2_AVAILABLE
        // Timer for MoveIt2 initialization (delayed to allow node to spin)
        moveit_init_timer_ = this->create_wall_timer(
            3s, std::bind(&RendezvousController::initializeMoveIt2, this));
#endif

        // Initialize orbit time
        orbit_start_time_ = this->now();

        // Start in ORBITING mode automatically
        mission_phase_ = MissionPhase::ORBITING;

        RCLCPP_INFO(this->get_logger(), "Rendezvous Controller initialized (HPOP mode)");
        RCLCPP_INFO(this->get_logger(), "  - Using HPOP satellite states from /hpop/satellite_state");
#ifdef MOVEIT2_AVAILABLE
        RCLCPP_INFO(this->get_logger(), "  - MoveIt2 will be initialized after node starts spinning");
#else
        RCLCPP_INFO(this->get_logger(), "  - MoveIt2 not available, using trajectory controller only");
#endif
        RCLCPP_INFO(this->get_logger(), "  - Approach distance: %.2f m (scaled)", approach_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Capture distance: %.2f m (scaled)", capture_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Approach velocity: %.3f m/s (scaled)", approach_velocity_);

        publishStatus("ORBITING - Waiting for HPOP satellite positions");
    }

#ifdef MOVEIT2_AVAILABLE
    void initializeMoveIt2()
    {
        // Only initialize once
        moveit_init_timer_->cancel();

        // Skip MoveIt2 for now - requires move_group node which needs gazebo_ros2_control
        // TODO: Enable when gazebo_ros2_control parsing issue is fixed
        RCLCPP_WARN(this->get_logger(), "MoveIt2 initialization SKIPPED (move_group not available)");
        RCLCPP_INFO(this->get_logger(), "Using trajectory controller fallback for arm control");
        moveit_initialized_ = false;

        // Original MoveIt2 initialization (commented out to avoid blocking):
        /*
        try {
            RCLCPP_INFO(this->get_logger(), "Initializing MoveIt2 MoveGroupInterface...");

            // Create MoveGroupInterface for the arm
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "arm");

            // Configure planning
            move_group_->setPlanningTime(5.0);
            move_group_->setNumPlanningAttempts(10);
            move_group_->setMaxVelocityScalingFactor(0.5);
            move_group_->setMaxAccelerationScalingFactor(0.5);

            moveit_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "MoveIt2 MoveGroupInterface initialized successfully!");
            RCLCPP_INFO(this->get_logger(), "  - Planning frame: %s", move_group_->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "  - End effector link: %s", move_group_->getEndEffectorLink().c_str());

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "MoveIt2 initialization failed: %s. Using fallback trajectory control.", e.what());
            moveit_initialized_ = false;
        }
        */
    }
#endif

private:
    void startRendezvousCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (mission_phase_ == MissionPhase::IDLE ||
            mission_phase_ == MissionPhase::ORBITING) {
            mission_phase_ = MissionPhase::RENDEZVOUS_STARTED;
            response->success = true;
            response->message = "Rendezvous sequence started";
            publishStatus("RENDEZVOUS STARTED - Calculating approach trajectory");

            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), ">>> RENDEZVOUS SEQUENCE INITIATED <<<");
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "Current distance: %.2f km", current_distance_ / 1000.0);

            // Log current satellite states if available
            if (hpop_satellites_.count("CHASER") && hpop_satellites_.count("TARGET")) {
                auto& chaser = hpop_satellites_["CHASER"];
                auto& target = hpop_satellites_["TARGET"];
                RCLCPP_INFO(this->get_logger(), "Chaser velocity: [%.2f, %.2f, %.2f] m/s",
                    chaser.velocity.x, chaser.velocity.y, chaser.velocity.z);
                RCLCPP_INFO(this->get_logger(), "Target velocity: [%.2f, %.2f, %.2f] m/s",
                    target.velocity.x, target.velocity.y, target.velocity.z);
            }
            RCLCPP_INFO(this->get_logger(), "Calculating delta-v for approach...");
        } else {
            response->success = false;
            response->message = "Cannot start rendezvous - mission already in progress";
            RCLCPP_WARN(this->get_logger(), "Rendezvous rejected - mission already in progress (phase: %d)",
                static_cast<int>(mission_phase_));
        }
    }

    void abortRendezvousCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        mission_phase_ = MissionPhase::ORBITING;  // Return to orbiting
        response->success = true;
        response->message = "Rendezvous aborted - returning to orbit";
        publishStatus("ABORTED - Returning to orbital phase");
        RCLCPP_WARN(this->get_logger(), "Rendezvous aborted! Returning to orbit.");
    }

    void captureCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (current_distance_ < capture_distance_ * 2) {
            mission_phase_ = MissionPhase::ARM_CAPTURE;
            response->success = true;
            response->message = "Arm capture sequence initiated";
            publishStatus("ARM CAPTURE - Extending robot arm");
            executeArmCapture();
        } else {
            response->success = false;
            response->message = "Too far for capture. Current distance: " +
                               std::to_string(current_distance_) + " m";
        }
    }

    void modelStatesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
        // Find chaser and target positions
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "chaser_satellite") {
                chaser_pose_ = msg->pose[i];
                chaser_found_ = true;
            } else if (msg->name[i] == "target_satellite") {
                target_pose_ = msg->pose[i];
                target_found_ = true;
            }
        }

        // Calculate distance
        if (chaser_found_ && target_found_) {
            double dx = target_pose_.position.x - chaser_pose_.position.x;
            double dy = target_pose_.position.y - chaser_pose_.position.y;
            double dz = target_pose_.position.z - chaser_pose_.position.z;
            current_distance_ = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Publish distance
            auto dist_msg = std_msgs::msg::Float64();
            dist_msg.data = current_distance_;
            distance_pub_->publish(dist_msg);
        }
    }

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_states_ = *msg;
    }

    void hpopStateCallback(const hpop_msgs::msg::SatelliteState::SharedPtr msg)
    {
        // Store satellite positions from HPOP (real orbital scale in meters)
        HpopSatelliteState state;
        state.position.x = msg->position.x;  // Real position in meters
        state.position.y = msg->position.y;
        state.position.z = msg->position.z;
        state.velocity.x = msg->velocity.x;
        state.velocity.y = msg->velocity.y;
        state.velocity.z = msg->velocity.z;
        state.last_update = this->now();

        hpop_satellites_[msg->satellite_id] = state;

        // Only update docking view once per cycle (when TARGET message arrives)
        // This prevents duplicate updates since both CHASER and TARGET trigger this callback
        if (msg->satellite_id == "TARGET" &&
            hpop_satellites_.count("CHASER") && hpop_satellites_.count("TARGET")) {
            updateDockingView();
        }
    }

    void updateDockingView()
    {
        // Get real positions from HPOP
        auto& chaser = hpop_satellites_["CHASER"];
        auto& target = hpop_satellites_["TARGET"];

        // Calculate real relative position (Target relative to Chaser)
        double rel_x = target.position.x - chaser.position.x;
        double rel_y = target.position.y - chaser.position.y;
        double rel_z = target.position.z - chaser.position.z;
        double real_distance = std::sqrt(rel_x*rel_x + rel_y*rel_y + rel_z*rel_z);

        // Adaptive scaling for visualization:
        // - At > 100 km: Compress to ~50m display distance
        // - At 1-100 km: Logarithmic compression
        // - At < 1 km: Linear scale (1:100 for visibility)
        // - At < 10 m: Real scale (1:1)
        double display_distance;
        if (real_distance < 10.0) {
            // Very close: real scale
            display_distance = real_distance;
        } else if (real_distance < 1000.0) {
            // Close (< 1km): 1:10 scale
            display_distance = real_distance / 10.0;
        } else if (real_distance < 100000.0) {
            // Medium (1-100 km): logarithmic compression
            // Map 1km-100km to 10m-30m display
            double log_dist = std::log10(real_distance / 1000.0);  // 0 to 2
            display_distance = 10.0 + log_dist * 10.0;  // 10m to 30m
        } else {
            // Far (> 100 km): heavily compressed
            // Map 100km+ to 30m-50m display
            double log_dist = std::log10(real_distance / 100000.0);  // 0 to ~1.2 (for 1500km)
            display_distance = 30.0 + std::min(log_dist * 20.0, 20.0);  // 30m to 50m max
        }

        // Direction unit vector
        double dir_x = rel_x / real_distance;
        double dir_y = rel_y / real_distance;
        double dir_z = rel_z / real_distance;

        // Store for distance display
        current_distance_ = real_distance;

        // Publish distance
        auto dist_msg = std_msgs::msg::Float64();
        dist_msg.data = real_distance;
        distance_pub_->publish(dist_msg);

        // Log distance changes (only every 100km change to reduce spam)
        static double last_logged_distance = 0;
        if (std::abs(real_distance - last_logged_distance) > 100000.0) {  // Log every 100km change
            last_logged_distance = real_distance;
            RCLCPP_INFO(this->get_logger(),
                "Distance: %.2f km (display: %.2f m)",
                real_distance / 1000.0, display_distance);
        }

        // Update positions using docking_frame (Chaser-centered)
        updateDockingTF(dir_x, dir_y, dir_z, display_distance, chaser, target);
    }

    void updateDockingTF(double dir_x, double dir_y, double dir_z, double display_dist,
                         const HpopSatelliteState& chaser, const HpopSatelliteState& target)
    {
        // Chaser orientation: point along velocity direction
        double vx = chaser.velocity.x;
        double vy = chaser.velocity.y;
        double v_mag = std::sqrt(vx*vx + vy*vy);
        double qw_c = 1.0, qx_c = 0.0, qy_c = 0.0, qz_c = 0.0;
        if (v_mag > 0.001) {
            double yaw = std::atan2(vy, vx);
            qz_c = std::sin(yaw / 2.0);
            qw_c = std::cos(yaw / 2.0);
        }

        // Publish TF: world -> docking_frame (at origin for clarity)
        geometry_msgs::msg::TransformStamped docking_tf;
        docking_tf.header.stamp = this->now();
        docking_tf.header.frame_id = "world";
        docking_tf.child_frame_id = "docking_frame";
        docking_tf.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(docking_tf);

        // Publish TF: docking_frame -> dummy_root (Chaser at origin)
        geometry_msgs::msg::TransformStamped chaser_tf;
        chaser_tf.header.stamp = this->now();
        chaser_tf.header.frame_id = "docking_frame";
        chaser_tf.child_frame_id = "dummy_root";
        chaser_tf.transform.translation.x = 0.0;
        chaser_tf.transform.translation.y = 0.0;
        chaser_tf.transform.translation.z = 0.0;
        chaser_tf.transform.rotation.w = qw_c;
        chaser_tf.transform.rotation.x = qx_c;
        chaser_tf.transform.rotation.y = qy_c;
        chaser_tf.transform.rotation.z = qz_c;
        tf_broadcaster_->sendTransform(chaser_tf);

        // Target orientation: tumbling
        double elapsed = (this->now() - orbit_start_time_).seconds();
        double tumble_rate = 0.3;
        double roll = tumble_rate * elapsed * 0.5;
        double pitch = tumble_rate * elapsed * 0.3;
        double yaw_tumble = tumble_rate * elapsed * 0.7;

        double cy = std::cos(yaw_tumble * 0.5);
        double sy = std::sin(yaw_tumble * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        double qw_t = cr * cp * cy + sr * sp * sy;
        double qx_t = sr * cp * cy - cr * sp * sy;
        double qy_t = cr * sp * cy + sr * cp * sy;
        double qz_t = cr * cp * sy - sr * sp * cy;

        // Publish TF: docking_frame -> target/target_dummy_root (Target at relative position)
        geometry_msgs::msg::TransformStamped target_tf;
        target_tf.header.stamp = this->now();
        target_tf.header.frame_id = "docking_frame";
        target_tf.child_frame_id = "target/target_dummy_root";
        target_tf.transform.translation.x = dir_x * display_dist;
        target_tf.transform.translation.y = dir_y * display_dist;
        target_tf.transform.translation.z = dir_z * display_dist;
        target_tf.transform.rotation.w = qw_t;
        target_tf.transform.rotation.x = qx_t;
        target_tf.transform.rotation.y = qy_t;
        target_tf.transform.rotation.z = qz_t;
        tf_broadcaster_->sendTransform(target_tf);

        // Update Gazebo model positions (same adaptive scaling)
        updateGazeboPositions(dir_x, dir_y, dir_z, display_dist,
                              qw_c, qx_c, qy_c, qz_c,
                              qw_t, qx_t, qy_t, qz_t);
    }

    void updateGazeboPositions(double dir_x, double dir_y, double dir_z, double display_dist,
                               double qw_c, double qx_c, double qy_c, double qz_c,
                               double qw_t, double qx_t, double qy_t, double qz_t)
    {
        if (!set_entity_client_->wait_for_service(10ms)) return;

        // Chaser at origin
        auto chaser_req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        chaser_req->state.name = "chaser_satellite";
        chaser_req->state.reference_frame = "world";
        chaser_req->state.pose.position.x = 0.0;
        chaser_req->state.pose.position.y = 0.0;
        chaser_req->state.pose.position.z = 0.0;
        chaser_req->state.pose.orientation.w = qw_c;
        chaser_req->state.pose.orientation.x = qx_c;
        chaser_req->state.pose.orientation.y = qy_c;
        chaser_req->state.pose.orientation.z = qz_c;
        set_entity_client_->async_send_request(chaser_req);
        chaser_found_ = true;

        // Target at relative position
        auto target_req = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        target_req->state.name = "target_satellite";
        target_req->state.reference_frame = "world";
        target_req->state.pose.position.x = dir_x * display_dist;
        target_req->state.pose.position.y = dir_y * display_dist;
        target_req->state.pose.position.z = dir_z * display_dist;
        target_req->state.pose.orientation.w = qw_t;
        target_req->state.pose.orientation.x = qx_t;
        target_req->state.pose.orientation.y = qy_t;
        target_req->state.pose.orientation.z = qz_t;
        set_entity_client_->async_send_request(target_req);
        target_found_ = true;
    }

    void controlLoop()
    {
        // HPOP positions are updated in hpopStateCallback
        // This loop handles mission state transitions and approach logic

        switch (mission_phase_) {
            case MissionPhase::IDLE:
                // Waiting for command
                break;

            case MissionPhase::ORBITING:
                // Satellites are orbiting via HPOP - positions updated in callback
                // Just log status periodically
                {
                    static int orbit_counter = 0;
                    if (++orbit_counter % 100 == 0) {  // Every 2 seconds (50Hz * 100 = 2s)
                        publishStatus("ORBITING - Waiting for rendezvous command");
                    }
                }
                break;

            case MissionPhase::RENDEZVOUS_STARTED:
                // Calculate initial Delta-V and start approach
                calculateApproachDeltaV();
                mission_phase_ = MissionPhase::APPROACH_PHASE;
                break;

            case MissionPhase::APPROACH_PHASE:
                // Move chaser toward target (Target continues orbiting)
                executeApproach();

                // Log distance periodically (every 5 seconds at 50Hz = every 250 iterations)
                {
                    static int approach_log_counter = 0;
                    static double last_approach_distance = -1.0;
                    static rclcpp::Time last_approach_log_time = this->now();

                    if (++approach_log_counter % 250 == 0) {
                        double now_sec = this->now().seconds();
                        double last_sec = last_approach_log_time.seconds();
                        double dt = now_sec - last_sec;

                        if (last_approach_distance > 0 && dt > 0.1) {
                            double distance_change = last_approach_distance - current_distance_;
                            double closing_rate = distance_change / dt;  // m/s
                            RCLCPP_INFO(this->get_logger(),
                                "[APPROACH] Distance: %.2f km | Closing rate: %.1f m/s (%.1f km/h)",
                                current_distance_ / 1000.0, closing_rate, closing_rate * 3.6);
                        } else {
                            RCLCPP_INFO(this->get_logger(),
                                "[APPROACH] Distance: %.2f km", current_distance_ / 1000.0);
                        }
                        last_approach_distance = current_distance_;
                        last_approach_log_time = this->now();
                    }
                }

                if (current_distance_ < approach_distance_) {
                    mission_phase_ = MissionPhase::PROXIMITY_OPS;
                    publishStatus("PROXIMITY OPS - Final approach phase");
                    RCLCPP_INFO(this->get_logger(), "Entering proximity operations at %.2f m", current_distance_);
                }
                break;

            case MissionPhase::PROXIMITY_OPS:
                // Fine approach (Target still moves, Chaser follows)
                executeFinalApproach();
                if (current_distance_ < capture_distance_) {
                    mission_phase_ = MissionPhase::ARM_CAPTURE;
                    publishStatus("ARM CAPTURE - Initiating capture sequence");
                    RCLCPP_INFO(this->get_logger(), "Capture distance reached: %.3f m", current_distance_);
                    executeArmCapture();
                }
                break;

            case MissionPhase::ARM_CAPTURE:
                // Arm is moving, wait for completion
                // Keep both satellites stationary during arm operation
                break;

            case MissionPhase::DOCKED:
                publishStatus("DOCKED - Capture successful!");
                mission_phase_ = MissionPhase::MISSION_COMPLETE;
                RCLCPP_INFO(this->get_logger(), "Mission complete! Target captured.");
                break;

            case MissionPhase::MISSION_COMPLETE:
                // Mission done - satellites stay locked together
                break;
        }
    }

    void calculateApproachDeltaV()
    {
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "Calculating approach Delta-V...");

        // Calculate delta-v based on HPOP relative position
        if (hpop_satellites_.count("CHASER") && hpop_satellites_.count("TARGET")) {
            auto& chaser = hpop_satellites_["CHASER"];
            auto& target = hpop_satellites_["TARGET"];

            // Relative position
            double rel_x = target.position.x - chaser.position.x;
            double rel_y = target.position.y - chaser.position.y;
            double rel_z = target.position.z - chaser.position.z;
            double distance_km = std::sqrt(rel_x*rel_x + rel_y*rel_y + rel_z*rel_z) / 1000.0;

            // Relative velocity
            double dv_x = target.velocity.x - chaser.velocity.x;
            double dv_y = target.velocity.y - chaser.velocity.y;
            double dv_z = target.velocity.z - chaser.velocity.z;
            double rel_vel = std::sqrt(dv_x*dv_x + dv_y*dv_y + dv_z*dv_z);

            RCLCPP_INFO(this->get_logger(), "Current state:");
            RCLCPP_INFO(this->get_logger(), "  Distance: %.2f km", distance_km);
            RCLCPP_INFO(this->get_logger(), "  Relative velocity: %.2f m/s", rel_vel);
            RCLCPP_INFO(this->get_logger(), "  Relative position (km): [%.2f, %.2f, %.2f]",
                rel_x/1000.0, rel_y/1000.0, rel_z/1000.0);

            // Calculate delta-v magnitude based on distance
            // Rule: At 1000km, need ~50 m/s retrograde burn for Hohmann-like transfer
            double delta_v_magnitude = std::min(100.0, distance_km * 0.05);  // 0.05 m/s per km

            // Publish Delta-V to HPOP for orbital mechanics
            // Negative in-track (y) = retrograde burn = lower orbit = faster = catch up
            auto hpop_dv = geometry_msgs::msg::Vector3();
            hpop_dv.x = 0.0;                    // radial (not used for phasing)
            hpop_dv.y = -delta_v_magnitude;    // in-track: retrograde burn
            hpop_dv.z = 0.0;                    // cross-track

            hpop_deltav_pub_->publish(hpop_dv);

            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), ">>> DELTA-V COMMAND SENT TO HPOP <<<");
            RCLCPP_INFO(this->get_logger(), "  Radial (x):     %.2f m/s", hpop_dv.x);
            RCLCPP_INFO(this->get_logger(), "  In-track (y):   %.2f m/s (retrograde)", hpop_dv.y);
            RCLCPP_INFO(this->get_logger(), "  Cross-track (z): %.2f m/s", hpop_dv.z);
            RCLCPP_INFO(this->get_logger(), "  Total delta-v:  %.2f m/s", delta_v_magnitude);
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Maneuver: Retrograde burn to lower orbit");
            RCLCPP_INFO(this->get_logger(), "Effect: Lower orbit = higher velocity = catch up to target");
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");

            // Publish to local delta-v topic
            auto dv_msg = geometry_msgs::msg::Twist();
            dv_msg.linear.x = hpop_dv.x;
            dv_msg.linear.y = hpop_dv.y;
            dv_msg.linear.z = hpop_dv.z;
            delta_v_pub_->publish(dv_msg);

        } else {
            RCLCPP_WARN(this->get_logger(), "HPOP satellite data not available yet!");
        }
    }

    void executeApproach()
    {
        if (!chaser_found_ || !target_found_) return;
        if (!set_entity_client_->wait_for_service(100ms)) return;

        // Move chaser toward target
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = "chaser_satellite";

        // New position (move toward target)
        double dx = target_pose_.position.x - chaser_pose_.position.x;
        double dy = target_pose_.position.y - chaser_pose_.position.y;
        double dz = target_pose_.position.z - chaser_pose_.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist > capture_distance_) {
            double step = approach_velocity_ * 0.1;  // 100ms loop
            request->state.pose.position.x = chaser_pose_.position.x + (dx / dist) * step;
            request->state.pose.position.y = chaser_pose_.position.y + (dy / dist) * step;
            request->state.pose.position.z = chaser_pose_.position.z + (dz / dist) * step;
            request->state.pose.orientation = chaser_pose_.orientation;

            set_entity_client_->async_send_request(request);

            // Update status periodically
            static int counter = 0;
            if (++counter % 20 == 0) {
                publishStatus("APPROACH - Distance: " + std::to_string(current_distance_) + " m");
            }
        }
    }

    void executeFinalApproach()
    {
        if (!chaser_found_ || !target_found_) return;
        if (!set_entity_client_->wait_for_service(100ms)) return;

        // Slower approach in proximity phase
        auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
        request->state.name = "chaser_satellite";

        double dx = target_pose_.position.x - chaser_pose_.position.x;
        double dy = target_pose_.position.y - chaser_pose_.position.y;
        double dz = target_pose_.position.z - chaser_pose_.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist > capture_distance_) {
            double step = approach_velocity_ * 0.05;  // Half speed
            request->state.pose.position.x = chaser_pose_.position.x + (dx / dist) * step;
            request->state.pose.position.y = chaser_pose_.position.y + (dy / dist) * step;
            request->state.pose.position.z = chaser_pose_.position.z + (dz / dist) * step;
            request->state.pose.orientation = chaser_pose_.orientation;

            set_entity_client_->async_send_request(request);
        }
    }

    void executeArmCapture()
    {
#ifdef MOVEIT2_AVAILABLE
        // Try MoveIt2 first if initialized
        if (moveit_initialized_ && move_group_) {
            executeArmCaptureWithMoveIt2();
            return;
        }
#endif
        executeArmCaptureWithTrajectory();
    }

#ifdef MOVEIT2_AVAILABLE
    void executeArmCaptureWithMoveIt2()
    {
        RCLCPP_INFO(this->get_logger(), "Executing arm capture with MoveIt2...");

        // Step 1: Go to ready position
        RCLCPP_INFO(this->get_logger(), "Step 1: Moving to ready position...");
        move_group_->setNamedTarget("ready");
        auto result = move_group_->move();

        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "MoveIt2 ready position failed, trying capture directly...");
        }

        // Step 2: Go to capture position
        RCLCPP_INFO(this->get_logger(), "Step 2: Moving to capture position...");
        move_group_->setNamedTarget("capture");
        result = move_group_->move();

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "MoveIt2 capture completed successfully!");
            mission_phase_ = MissionPhase::DOCKED;
        } else {
            RCLCPP_ERROR(this->get_logger(), "MoveIt2 capture failed! Error code: %d", result.val);
            // Fallback to trajectory control
            executeArmCaptureWithTrajectory();
        }
    }
#endif

    void executeArmCaptureWithTrajectory()
    {
        if (!arm_action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Arm action server not available!");
            return;
        }

        // Create trajectory for arm extension
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // Point 1: Ready position
        trajectory_msgs::msg::JointTrajectoryPoint point1;
        point1.positions = {0.0, 0.3, -0.5, 0.0, 0.2, 0.0};
        point1.time_from_start = rclcpp::Duration(2, 0);

        // Point 2: Extend toward target
        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.positions = {0.0, 0.5, -0.8, 0.0, 0.3, 0.0};
        point2.time_from_start = rclcpp::Duration(4, 0);

        // Point 3: Capture position
        trajectory_msgs::msg::JointTrajectoryPoint point3;
        point3.positions = {0.0, 0.6, -1.0, 0.0, 0.4, 0.0};
        point3.time_from_start = rclcpp::Duration(6, 0);

        goal_msg.trajectory.points = {point1, point2, point3};

        RCLCPP_INFO(this->get_logger(), "Sending arm capture trajectory (fallback mode)...");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFJT::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Arm goal rejected!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Arm goal accepted!");
                }
            };
        send_goal_options.result_callback =
            [this](const GoalHandleFJT::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Arm capture trajectory completed!");
                    mission_phase_ = MissionPhase::DOCKED;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Arm trajectory failed!");
                }
            };

        arm_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void publishStatus(const std::string& status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_rendezvous_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_rendezvous_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr delta_v_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr hpop_deltav_pub_;

    // Subscribers
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<hpop_msgs::msg::SatelliteState>::SharedPtr hpop_state_sub_;

    // HPOP satellite states
    std::map<std::string, HpopSatelliteState> hpop_satellites_;
    geometry_msgs::msg::Point target_hpop_pos_;
    geometry_msgs::msg::Point chaser_hpop_pos_;

    // Action client
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_action_client_;

    // Service client
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_client_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
#ifdef MOVEIT2_AVAILABLE
    rclcpp::TimerBase::SharedPtr moveit_init_timer_;

    // MoveIt2
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    bool moveit_initialized_ = false;
#endif

    // State
    MissionPhase mission_phase_ = MissionPhase::IDLE;
    geometry_msgs::msg::Pose chaser_pose_;
    geometry_msgs::msg::Pose target_pose_;
    sensor_msgs::msg::JointState current_joint_states_;
    bool chaser_found_ = false;
    bool target_found_ = false;
    double current_distance_ = 999.0;
    double approach_direction_[3] = {0.0, 0.0, 0.0};

    // Accumulated delta-v offset for approach maneuver
    // This offset is added to HPOP position to simulate the effect of thrust
    double deltav_offset_x_ = 0.0;
    double deltav_offset_y_ = 0.0;
    double deltav_offset_z_ = 0.0;
    rclcpp::Time last_deltav_update_;
    bool deltav_initialized_ = false;

    // Time tracking
    rclcpp::Time orbit_start_time_;

    // Parameters
    double approach_distance_;
    double capture_distance_;
    double approach_velocity_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RendezvousController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
