/**
 * @file earth_display.hpp
 * @brief 3D Earth display for RViz2
 *
 * Displays Earth as a textured sphere with proper orientation.
 */

#ifndef HPOP_RVIZ_PLUGINS_EARTH_DISPLAY_HPP
#define HPOP_RVIZ_PLUGINS_EARTH_DISPLAY_HPP

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMesh.h>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>

namespace hpop_rviz_plugins
{

/**
 * @brief RViz2 Display for 3D Earth
 */
class EarthDisplay : public rviz_common::Display
{
    Q_OBJECT

public:
    EarthDisplay();
    ~EarthDisplay() override;

    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

protected Q_SLOTS:
    void updateEarthRadius();
    void updateScaleFactor();
    void updateShowGrid();
    void updateShowEquator();
    void updateRotation();

private:
    void createEarth();
    void createGrid();
    void createEquator();
    void destroyEarth();
    void updateEarthRotation(float dt);

    // Scene objects
    Ogre::SceneNode* earth_node_;
    Ogre::SceneNode* grid_node_;
    Ogre::SceneNode* equator_node_;
    Ogre::Entity* earth_entity_;
    Ogre::ManualObject* grid_object_;
    Ogre::ManualObject* equator_object_;

    // Materials
    Ogre::MaterialPtr earth_material_;
    Ogre::MaterialPtr grid_material_;

    // Properties
    rviz_common::properties::FloatProperty* scale_factor_property_;
    rviz_common::properties::BoolProperty* show_grid_property_;
    rviz_common::properties::BoolProperty* show_equator_property_;
    rviz_common::properties::BoolProperty* rotate_earth_property_;
    rviz_common::properties::FloatProperty* rotation_speed_property_;
    rviz_common::properties::ColorProperty* earth_color_property_;
    rviz_common::properties::ColorProperty* grid_color_property_;

    // State
    float scale_factor_;        // km to visualization scale
    float current_rotation_;    // Current Earth rotation angle
    bool rotate_earth_;
};

} // namespace hpop_rviz_plugins

#endif // HPOP_RVIZ_PLUGINS_EARTH_DISPLAY_HPP
