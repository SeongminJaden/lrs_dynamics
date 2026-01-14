/**
 * @file arm_capture_controller.cpp
 * @brief Simple arm controller for satellite capture demonstration
 *
 * This node demonstrates arm movement for approaching and capturing
 * the target satellite using joint position commands.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>
#include <vector>
#include <memory>

class ArmCaptureController : public rclcpp::Node
{
public:
    ArmCaptureController()
        : Node("arm_capture_controller")
    {
        // Parameters
        this->declare_parameter<double>("approach_speed", 0.1);
        this->declare_parameter<double>("capture_threshold", 0.15);

        approach_speed_ = this->get_parameter("approach_speed").as_double();
        capture_threshold_ = this->get_parameter("capture_threshold").as_double();

        // Publishers
        joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_controller/commands", 10);

        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmCaptureController::jointStateCallback, this, std::placeholders::_1));

        relative_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/hpop/docking_relative_pose", 10,
            std::bind(&ArmCaptureController::relativePoseCallback, this, std::placeholders::_1));

        docking_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hpop/docking_status", 10,
            std::bind(&ArmCaptureController::dockingStatusCallback, this, std::placeholders::_1));

        // Services
        start_capture_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/arm/start_capture",
            std::bind(&ArmCaptureController::startCaptureCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/arm/go_home",
            std::bind(&ArmCaptureController::goHomeCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        extend_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/arm/extend",
            std::bind(&ArmCaptureController::extendCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ArmCaptureController::controlLoop, this));

        // Initialize joint positions (6 joints)
        current_joints_.resize(6, 0.0);
        target_joints_.resize(6, 0.0);
        home_position_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        extended_position_ = {0.0, -0.3, 0.6, 0.0, -0.3, 0.0};  // Arm extended forward

        RCLCPP_INFO(this->get_logger(), "Arm Capture Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Services: /arm/start_capture, /arm/go_home, /arm/extend");
    }

private:
    // Subscriber callbacks
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current joint positions
        for (size_t i = 0; i < msg->name.size() && i < 6; i++)
        {
            // Find joint index by name
            for (int j = 1; j <= 6; j++)
            {
                std::string joint_name = "joint" + std::to_string(j);
                if (msg->name[i] == joint_name && !msg->position.empty())
                {
                    current_joints_[j-1] = msg->position[i];
                    break;
                }
            }
        }
    }

    void relativePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        relative_pose_ = *msg;
        double dist = std::sqrt(
            msg->pose.position.x * msg->pose.position.x +
            msg->pose.position.y * msg->pose.position.y +
            msg->pose.position.z * msg->pose.position.z
        );
        distance_to_target_ = dist;
    }

    void dockingStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_docked_ = msg->data;
        if (is_docked_ && capture_in_progress_)
        {
            RCLCPP_INFO(this->get_logger(), "CAPTURE SUCCESSFUL! Target docked.");
            capture_in_progress_ = false;
        }
    }

    // Service callbacks
    void startCaptureCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (is_docked_)
        {
            response->success = false;
            response->message = "Already docked";
            return;
        }

        capture_in_progress_ = true;
        target_joints_ = extended_position_;
        response->success = true;
        response->message = "Capture sequence started - extending arm";
        RCLCPP_INFO(this->get_logger(), "Starting capture sequence...");

        // Activate magnetic docking
        auto client = this->create_client<std_srvs::srv::Trigger>("/hpop/activate_dock");
        if (client->wait_for_service(std::chrono::seconds(1)))
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            client->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Magnetic docking activated");
        }
    }

    void goHomeCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        target_joints_ = home_position_;
        capture_in_progress_ = false;
        response->success = true;
        response->message = "Moving to home position";
        RCLCPP_INFO(this->get_logger(), "Moving arm to home position");
    }

    void extendCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        target_joints_ = extended_position_;
        response->success = true;
        response->message = "Extending arm";
        RCLCPP_INFO(this->get_logger(), "Extending arm");
    }

    void controlLoop()
    {
        // Simple proportional control to target positions
        std_msgs::msg::Float64MultiArray cmd;
        cmd.data.resize(6);

        bool at_target = true;
        for (size_t i = 0; i < 6; i++)
        {
            double error = target_joints_[i] - current_joints_[i];

            // Limit velocity
            double max_delta = approach_speed_ * 0.05;  // 50ms * speed
            if (std::abs(error) > max_delta)
            {
                error = (error > 0) ? max_delta : -max_delta;
                at_target = false;
            }

            cmd.data[i] = current_joints_[i] + error;
        }

        joint_command_pub_->publish(cmd);

        // Log status periodically
        static int log_counter = 0;
        if (++log_counter >= 40)  // Every 2 seconds
        {
            log_counter = 0;
            if (capture_in_progress_)
            {
                RCLCPP_INFO(this->get_logger(),
                    "Capture in progress - Distance to target: %.3f m, Docked: %s",
                    distance_to_target_, is_docked_ ? "YES" : "NO");
            }
        }
    }

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr relative_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr docking_status_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_capture_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extend_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    std::vector<double> current_joints_;
    std::vector<double> target_joints_;
    std::vector<double> home_position_;
    std::vector<double> extended_position_;
    geometry_msgs::msg::PoseStamped relative_pose_;
    double distance_to_target_ = 999.0;
    bool is_docked_ = false;
    bool capture_in_progress_ = false;

    // Parameters
    double approach_speed_;
    double capture_threshold_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmCaptureController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
