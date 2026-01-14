/**
 * @file earth_display.cpp
 * @brief Earth display implementation for RViz2
 */

#include "hpop_rviz_plugins/earth_display.hpp"

#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreMeshManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/mesh_loader.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <cmath>

namespace hpop_rviz_plugins
{

EarthDisplay::EarthDisplay()
    : earth_node_(nullptr)
    , grid_node_(nullptr)
    , equator_node_(nullptr)
    , earth_entity_(nullptr)
    , grid_object_(nullptr)
    , equator_object_(nullptr)
    , scale_factor_(0.001f)  // 6378 km -> 6.378 units
    , current_rotation_(0.0f)
    , rotate_earth_(false)
{
}

EarthDisplay::~EarthDisplay()
{
    destroyEarth();
}

void EarthDisplay::onInitialize()
{
    Display::onInitialize();

    // Create properties
    scale_factor_property_ = new rviz_common::properties::FloatProperty(
        "Scale Factor", 0.001f,
        "Scale factor for Earth radius (km to display units)",
        this, SLOT(updateScaleFactor()));
    scale_factor_property_->setMin(0.0001f);
    scale_factor_property_->setMax(1.0f);

    show_grid_property_ = new rviz_common::properties::BoolProperty(
        "Show Grid", true,
        "Show latitude/longitude grid",
        this, SLOT(updateShowGrid()));

    show_equator_property_ = new rviz_common::properties::BoolProperty(
        "Show Equator", true,
        "Highlight the equator",
        this, SLOT(updateShowEquator()));

    rotate_earth_property_ = new rviz_common::properties::BoolProperty(
        "Rotate Earth", false,
        "Animate Earth rotation",
        this, SLOT(updateRotation()));

    rotation_speed_property_ = new rviz_common::properties::FloatProperty(
        "Rotation Speed", 1.0f,
        "Earth rotation speed multiplier",
        this);
    rotation_speed_property_->setMin(0.1f);
    rotation_speed_property_->setMax(100.0f);

    earth_color_property_ = new rviz_common::properties::ColorProperty(
        "Earth Color", QColor(30, 100, 200),
        "Color of the Earth sphere",
        this, SLOT(updateEarthRadius()));

    grid_color_property_ = new rviz_common::properties::ColorProperty(
        "Grid Color", QColor(100, 100, 100),
        "Color of the latitude/longitude grid",
        this, SLOT(updateShowGrid()));
}

void EarthDisplay::onEnable()
{
    createEarth();
    createGrid();
    createEquator();
}

void EarthDisplay::onDisable()
{
    destroyEarth();
}

void EarthDisplay::update(float wall_dt, float /*ros_dt*/)
{
    if (rotate_earth_ && earth_node_)
    {
        updateEarthRotation(wall_dt);
    }
}

void EarthDisplay::reset()
{
    Display::reset();
    current_rotation_ = 0.0f;
    if (earth_node_)
    {
        earth_node_->setOrientation(Ogre::Quaternion::IDENTITY);
    }
}

void EarthDisplay::updateEarthRadius()
{
    if (earth_node_)
    {
        float radius = 6378.0f * scale_factor_;  // Earth equatorial radius in km
        earth_node_->setScale(radius, radius, radius);
    }
}

void EarthDisplay::updateScaleFactor()
{
    scale_factor_ = scale_factor_property_->getFloat();
    updateEarthRadius();
    updateShowGrid();
    updateShowEquator();
}

void EarthDisplay::updateShowGrid()
{
    if (grid_node_)
    {
        grid_node_->setVisible(show_grid_property_->getBool());
    }
}

void EarthDisplay::updateShowEquator()
{
    if (equator_node_)
    {
        equator_node_->setVisible(show_equator_property_->getBool());
    }
}

void EarthDisplay::updateRotation()
{
    rotate_earth_ = rotate_earth_property_->getBool();
}

void EarthDisplay::createEarth()
{
    if (earth_node_)
    {
        return;
    }

    earth_node_ = scene_node_->createChildSceneNode();

    // Create a sphere mesh if it doesn't exist
    try
    {
        // Try to use existing sphere mesh
        earth_entity_ = scene_manager_->createEntity(
            Ogre::SceneManager::PT_SPHERE);
    }
    catch (...)
    {
        // Sphere mesh not available, create manual sphere
        earth_entity_ = nullptr;
    }

    if (earth_entity_)
    {
        // Create material
        earth_material_ = Ogre::MaterialManager::getSingleton().create(
            "HpopEarthMaterial",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        auto* technique = earth_material_->getTechnique(0);
        auto* pass = technique->getPass(0);

        QColor color = earth_color_property_->getColor();
        pass->setDiffuse(color.redF(), color.greenF(), color.blueF(), 0.8f);
        pass->setAmbient(color.redF() * 0.3f, color.greenF() * 0.3f, color.blueF() * 0.3f);
        pass->setSpecular(0.3f, 0.3f, 0.3f, 1.0f);
        pass->setShininess(20.0f);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->setDepthWriteEnabled(true);

        earth_entity_->setMaterial(earth_material_);
        earth_node_->attachObject(earth_entity_);
    }

    updateEarthRadius();
}

void EarthDisplay::createGrid()
{
    if (grid_node_)
    {
        return;
    }

    grid_node_ = scene_node_->createChildSceneNode();
    grid_object_ = scene_manager_->createManualObject();
    grid_object_->setDynamic(false);

    float radius = 6378.0f * scale_factor_ * 1.001f;  // Slightly larger than Earth

    QColor color = grid_color_property_->getColor();
    Ogre::ColourValue grid_color(color.redF(), color.greenF(), color.blueF(), 0.5f);

    grid_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    // Latitude lines (every 30 degrees)
    for (int lat = -60; lat <= 60; lat += 30)
    {
        float phi = lat * M_PI / 180.0f;
        float r = radius * std::cos(phi);
        float z = radius * std::sin(phi);

        for (int lon = 0; lon < 360; lon += 5)
        {
            float theta1 = lon * M_PI / 180.0f;
            float theta2 = (lon + 5) * M_PI / 180.0f;

            grid_object_->position(r * std::cos(theta1), r * std::sin(theta1), z);
            grid_object_->colour(grid_color);
            grid_object_->position(r * std::cos(theta2), r * std::sin(theta2), z);
            grid_object_->colour(grid_color);
        }
    }

    // Longitude lines (every 30 degrees)
    for (int lon = 0; lon < 360; lon += 30)
    {
        float theta = lon * M_PI / 180.0f;

        for (int lat = -90; lat < 90; lat += 5)
        {
            float phi1 = lat * M_PI / 180.0f;
            float phi2 = (lat + 5) * M_PI / 180.0f;

            float x1 = radius * std::cos(phi1) * std::cos(theta);
            float y1 = radius * std::cos(phi1) * std::sin(theta);
            float z1 = radius * std::sin(phi1);

            float x2 = radius * std::cos(phi2) * std::cos(theta);
            float y2 = radius * std::cos(phi2) * std::sin(theta);
            float z2 = radius * std::sin(phi2);

            grid_object_->position(x1, y1, z1);
            grid_object_->colour(grid_color);
            grid_object_->position(x2, y2, z2);
            grid_object_->colour(grid_color);
        }
    }

    grid_object_->end();
    grid_node_->attachObject(grid_object_);
}

void EarthDisplay::createEquator()
{
    if (equator_node_)
    {
        return;
    }

    equator_node_ = scene_node_->createChildSceneNode();
    equator_object_ = scene_manager_->createManualObject();
    equator_object_->setDynamic(false);

    float radius = 6378.0f * scale_factor_ * 1.002f;

    Ogre::ColourValue equator_color(1.0f, 0.0f, 0.0f, 1.0f);  // Red

    equator_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (int lon = 0; lon <= 360; lon += 2)
    {
        float theta = lon * M_PI / 180.0f;
        equator_object_->position(radius * std::cos(theta), radius * std::sin(theta), 0.0f);
        equator_object_->colour(equator_color);
    }

    equator_object_->end();
    equator_node_->attachObject(equator_object_);
}

void EarthDisplay::destroyEarth()
{
    if (earth_entity_)
    {
        earth_node_->detachObject(earth_entity_);
        scene_manager_->destroyEntity(earth_entity_);
        earth_entity_ = nullptr;
    }

    if (grid_object_)
    {
        grid_node_->detachObject(grid_object_);
        scene_manager_->destroyManualObject(grid_object_);
        grid_object_ = nullptr;
    }

    if (equator_object_)
    {
        equator_node_->detachObject(equator_object_);
        scene_manager_->destroyManualObject(equator_object_);
        equator_object_ = nullptr;
    }

    if (earth_node_)
    {
        scene_manager_->destroySceneNode(earth_node_);
        earth_node_ = nullptr;
    }

    if (grid_node_)
    {
        scene_manager_->destroySceneNode(grid_node_);
        grid_node_ = nullptr;
    }

    if (equator_node_)
    {
        scene_manager_->destroySceneNode(equator_node_);
        equator_node_ = nullptr;
    }

    if (earth_material_)
    {
        Ogre::MaterialManager::getSingleton().remove(earth_material_);
        earth_material_.reset();
    }
}

void EarthDisplay::updateEarthRotation(float dt)
{
    // Earth rotation: 360 degrees / 86400 seconds = 0.00417 deg/s
    float rotation_rate = 0.00417f * rotation_speed_property_->getFloat();
    current_rotation_ += rotation_rate * dt;

    if (current_rotation_ > 360.0f)
    {
        current_rotation_ -= 360.0f;
    }

    Ogre::Quaternion rotation;
    rotation.FromAngleAxis(Ogre::Degree(current_rotation_), Ogre::Vector3::UNIT_Z);

    if (earth_node_)
    {
        earth_node_->setOrientation(rotation);
    }
    if (grid_node_)
    {
        grid_node_->setOrientation(rotation);
    }
}

} // namespace hpop_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hpop_rviz_plugins::EarthDisplay, rviz_common::Display)
