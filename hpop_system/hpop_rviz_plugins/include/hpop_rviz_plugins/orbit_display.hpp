/**
 * @file orbit_display.hpp
 * @brief Orbit trajectory display for RViz2
 *
 * Displays satellite orbits as 3D lines with color coding.
 */

#ifndef HPOP_RVIZ_PLUGINS_ORBIT_DISPLAY_HPP
#define HPOP_RVIZ_PLUGINS_ORBIT_DISPLAY_HPP

#include <memory>
#include <vector>
#include <map>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include "hpop_msgs/msg/satellite_state.hpp"

namespace hpop_rviz_plugins
{

/**
 * @brief Orbit trajectory data for one satellite
 */
struct OrbitTrajectory
{
    std::string satellite_id;
    std::vector<Ogre::Vector3> positions;
    Ogre::ManualObject* line_object;
    Ogre::SceneNode* scene_node;
    Ogre::ColourValue color;
    size_t max_points;

    OrbitTrajectory() : line_object(nullptr), scene_node(nullptr), max_points(1000) {}
};

/**
 * @brief RViz2 Display for satellite orbits
 */
class OrbitDisplay : public rviz_common::Display
{
    Q_OBJECT

public:
    OrbitDisplay();
    ~OrbitDisplay() override;

    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

protected Q_SLOTS:
    void updateTopic();
    void updateVisuals();
    void updateTrailLength();
    void updateLineWidth();
    void updateScaleFactor();

private:
    void subscribe();
    void unsubscribe();
    void processMessage(const hpop_msgs::msg::SatelliteState::SharedPtr msg);
    void createOrbitLine(const std::string& satellite_id);
    void updateOrbitLine(OrbitTrajectory& traj);
    void clearAllOrbits();
    Ogre::ColourValue getColorForSatellite(const std::string& id);

    // ROS subscription
    rclcpp::Subscription<hpop_msgs::msg::SatelliteState>::SharedPtr sub_;

    // Properties
    rviz_common::properties::RosTopicProperty* topic_property_;
    rviz_common::properties::IntProperty* trail_length_property_;
    rviz_common::properties::FloatProperty* line_width_property_;
    rviz_common::properties::FloatProperty* scale_factor_property_;
    rviz_common::properties::BoolProperty* show_current_pos_property_;
    rviz_common::properties::ColorProperty* default_color_property_;

    // Orbit data
    std::map<std::string, OrbitTrajectory> orbits_;

    // Scene
    Ogre::SceneNode* root_node_;

    // Settings
    float scale_factor_;  // km to meters visualization scale
    int trail_length_;
    float line_width_;
};

} // namespace hpop_rviz_plugins

#endif // HPOP_RVIZ_PLUGINS_ORBIT_DISPLAY_HPP
