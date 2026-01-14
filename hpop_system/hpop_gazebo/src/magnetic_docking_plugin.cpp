/**
 * @file magnetic_docking_plugin.cpp
 * @brief Gazebo plugin for magnetic docking between satellites
 *
 * This plugin simulates magnetic docking by creating a fixed joint
 * when the magnetic end-effector approaches the target docking port.
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>

namespace gazebo
{

class MagneticDockingPlugin : public ModelPlugin
{
public:
    MagneticDockingPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
        model_ = model;
        world_ = model_->GetWorld();

        // Initialize ROS2 node
        ros_node_ = gazebo_ros::Node::Get(sdf);

        // Get parameters from SDF
        if (sdf->HasElement("magnet_link"))
        {
            magnet_link_name_ = sdf->Get<std::string>("magnet_link");
        }
        else
        {
            magnet_link_name_ = "magnet_contact_point";
        }

        if (sdf->HasElement("target_model"))
        {
            target_model_name_ = sdf->Get<std::string>("target_model");
        }
        else
        {
            target_model_name_ = "target_satellite";
        }

        if (sdf->HasElement("target_link"))
        {
            target_link_name_ = sdf->Get<std::string>("target_link");
        }
        else
        {
            target_link_name_ = "magnetic_contact_surface";
        }

        if (sdf->HasElement("docking_distance"))
        {
            docking_distance_ = sdf->Get<double>("docking_distance");
        }
        else
        {
            docking_distance_ = 0.1;  // 10 cm
        }

        if (sdf->HasElement("magnetic_force"))
        {
            magnetic_force_ = sdf->Get<double>("magnetic_force");
        }
        else
        {
            magnetic_force_ = 100.0;  // N
        }

        // Get links
        magnet_link_ = model_->GetLink(magnet_link_name_);
        if (!magnet_link_)
        {
            RCLCPP_ERROR(ros_node_->get_logger(),
                        "Magnet link '%s' not found in model '%s'",
                        magnet_link_name_.c_str(), model_->GetName().c_str());
            return;
        }

        // Create publishers
        docking_status_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
            "/hpop/docking_status", 10);

        relative_pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/hpop/docking_relative_pose", 10);

        // Create services
        dock_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
            "/hpop/activate_dock",
            std::bind(&MagneticDockingPlugin::activateDockCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        undock_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
            "/hpop/deactivate_dock",
            std::bind(&MagneticDockingPlugin::deactivateDockCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Register update callback
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&MagneticDockingPlugin::OnUpdate, this));

        RCLCPP_INFO(ros_node_->get_logger(),
                   "Magnetic Docking Plugin loaded for model '%s'",
                   model_->GetName().c_str());
        RCLCPP_INFO(ros_node_->get_logger(),
                   "  Magnet link: %s, Target: %s::%s",
                   magnet_link_name_.c_str(),
                   target_model_name_.c_str(), target_link_name_.c_str());
        RCLCPP_INFO(ros_node_->get_logger(),
                   "  Docking distance: %.3f m, Magnetic force: %.1f N",
                   docking_distance_, magnetic_force_);
    }

    void OnUpdate()
    {
        // Try to find target model if not already found
        if (!target_model_)
        {
            target_model_ = world_->ModelByName(target_model_name_);
            if (target_model_)
            {
                target_link_ = target_model_->GetLink(target_link_name_);
                if (target_link_)
                {
                    RCLCPP_INFO(ros_node_->get_logger(),
                               "Found target: %s::%s",
                               target_model_name_.c_str(), target_link_name_.c_str());
                }
            }
        }

        if (!target_link_ || !magnet_link_)
        {
            return;
        }

        // Calculate distance between magnet and target
        ignition::math::Pose3d magnet_pose = magnet_link_->WorldPose();
        ignition::math::Pose3d target_pose = target_link_->WorldPose();

        ignition::math::Vector3d diff = target_pose.Pos() - magnet_pose.Pos();
        double distance = diff.Length();

        // Publish relative pose
        geometry_msgs::msg::PoseStamped rel_pose_msg;
        rel_pose_msg.header.stamp = ros_node_->now();
        rel_pose_msg.header.frame_id = "magnet_contact_point";
        rel_pose_msg.pose.position.x = diff.X();
        rel_pose_msg.pose.position.y = diff.Y();
        rel_pose_msg.pose.position.z = diff.Z();
        relative_pose_pub_->publish(rel_pose_msg);

        // State machine for docking
        if (!is_docked_ && dock_enabled_)
        {
            // Apply magnetic attraction force when close
            if (distance < docking_distance_ * 3.0)
            {
                ignition::math::Vector3d force_dir = diff.Normalized();
                double force_magnitude = magnetic_force_ *
                    (1.0 - distance / (docking_distance_ * 3.0));
                force_magnitude = std::max(0.0, force_magnitude);

                ignition::math::Vector3d force = force_dir * force_magnitude;
                magnet_link_->AddForce(force);
            }

            // Check if close enough to dock
            if (distance < docking_distance_)
            {
                createDockingJoint();
            }
        }

        // Publish docking status
        std_msgs::msg::Bool status_msg;
        status_msg.data = is_docked_;
        docking_status_pub_->publish(status_msg);
    }

private:
    void createDockingJoint()
    {
        if (is_docked_)
        {
            return;
        }

        RCLCPP_INFO(ros_node_->get_logger(), "Creating docking joint...");

        // Create a fixed joint between chaser and target
        physics::JointPtr joint = world_->Physics()->CreateJoint(
            "fixed", model_);

        joint->SetName("docking_joint");
        joint->Attach(magnet_link_, target_link_);

        // Set joint properties
        joint->Load(magnet_link_, target_link_,
                   ignition::math::Pose3d::Zero);
        joint->Init();

        docking_joint_ = joint;
        is_docked_ = true;

        RCLCPP_INFO(ros_node_->get_logger(),
                   "DOCKED! Joint created between %s and %s",
                   magnet_link_->GetName().c_str(),
                   target_link_->GetName().c_str());
    }

    void removeDockingJoint()
    {
        if (!is_docked_ || !docking_joint_)
        {
            return;
        }

        RCLCPP_INFO(ros_node_->get_logger(), "Removing docking joint...");

        docking_joint_->Detach();
        docking_joint_.reset();
        is_docked_ = false;

        RCLCPP_INFO(ros_node_->get_logger(), "UNDOCKED!");
    }

    void activateDockCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        dock_enabled_ = true;
        response->success = true;
        response->message = "Magnetic docking activated";
        RCLCPP_INFO(ros_node_->get_logger(), "Magnetic docking ACTIVATED");
    }

    void deactivateDockCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        dock_enabled_ = false;
        if (is_docked_)
        {
            removeDockingJoint();
        }
        response->success = true;
        response->message = "Magnetic docking deactivated and undocked";
        RCLCPP_INFO(ros_node_->get_logger(), "Magnetic docking DEACTIVATED");
    }

    // Gazebo members
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr magnet_link_;
    physics::ModelPtr target_model_;
    physics::LinkPtr target_link_;
    physics::JointPtr docking_joint_;
    event::ConnectionPtr update_connection_;

    // ROS2 members
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docking_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr relative_pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dock_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undock_service_;

    // Parameters
    std::string magnet_link_name_;
    std::string target_model_name_;
    std::string target_link_name_;
    double docking_distance_;
    double magnetic_force_;

    // State
    bool is_docked_ = false;
    bool dock_enabled_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(MagneticDockingPlugin)

} // namespace gazebo
