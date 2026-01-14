/**
 * @file orbit_display.cpp
 * @brief Orbit display implementation for RViz2
 */

#include "hpop_rviz_plugins/orbit_display.hpp"

#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace hpop_rviz_plugins
{

OrbitDisplay::OrbitDisplay()
    : root_node_(nullptr)
    , scale_factor_(0.001f)  // km to m (1:1000 for visualization)
    , trail_length_(500)
    , line_width_(2.0f)
{
}

OrbitDisplay::~OrbitDisplay()
{
    clearAllOrbits();
}

void OrbitDisplay::onInitialize()
{
    Display::onInitialize();

    // Create root scene node
    root_node_ = scene_node_->createChildSceneNode();

    // Create properties
    topic_property_ = new rviz_common::properties::RosTopicProperty(
        "Topic", "/hpop/satellite_state",
        QString::fromStdString(rosidl_generator_traits::name<hpop_msgs::msg::SatelliteState>()),
        "Topic to subscribe to for satellite states",
        this, SLOT(updateTopic()));

    trail_length_property_ = new rviz_common::properties::IntProperty(
        "Trail Length", 500,
        "Number of points in orbit trail",
        this, SLOT(updateTrailLength()));
    trail_length_property_->setMin(10);
    trail_length_property_->setMax(5000);

    line_width_property_ = new rviz_common::properties::FloatProperty(
        "Line Width", 2.0f,
        "Width of orbit lines",
        this, SLOT(updateLineWidth()));
    line_width_property_->setMin(0.5f);
    line_width_property_->setMax(10.0f);

    scale_factor_property_ = new rviz_common::properties::FloatProperty(
        "Scale Factor", 0.001f,
        "Scale factor for visualization (km to display units)",
        this, SLOT(updateScaleFactor()));
    scale_factor_property_->setMin(0.0001f);
    scale_factor_property_->setMax(1.0f);

    show_current_pos_property_ = new rviz_common::properties::BoolProperty(
        "Show Current Position", true,
        "Show marker at current satellite position",
        this, SLOT(updateVisuals()));

    default_color_property_ = new rviz_common::properties::ColorProperty(
        "Default Color", QColor(0, 255, 0),
        "Default color for orbit trails",
        this, SLOT(updateVisuals()));
}

void OrbitDisplay::onEnable()
{
    subscribe();
}

void OrbitDisplay::onDisable()
{
    unsubscribe();
    clearAllOrbits();
}

void OrbitDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
    // Update orbit visuals
    for (auto& [id, traj] : orbits_)
    {
        updateOrbitLine(traj);
    }
}

void OrbitDisplay::reset()
{
    Display::reset();
    clearAllOrbits();
}

void OrbitDisplay::updateTopic()
{
    unsubscribe();
    clearAllOrbits();
    subscribe();
}

void OrbitDisplay::updateVisuals()
{
    for (auto& [id, traj] : orbits_)
    {
        traj.color = getColorForSatellite(id);
        updateOrbitLine(traj);
    }
}

void OrbitDisplay::updateTrailLength()
{
    trail_length_ = trail_length_property_->getInt();
    for (auto& [id, traj] : orbits_)
    {
        traj.max_points = trail_length_;
        while (traj.positions.size() > traj.max_points)
        {
            traj.positions.erase(traj.positions.begin());
        }
    }
}

void OrbitDisplay::updateLineWidth()
{
    line_width_ = line_width_property_->getFloat();
}

void OrbitDisplay::updateScaleFactor()
{
    scale_factor_ = scale_factor_property_->getFloat();
}

void OrbitDisplay::subscribe()
{
    if (!isEnabled())
    {
        return;
    }

    try
    {
        sub_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->
            create_subscription<hpop_msgs::msg::SatelliteState>(
                topic_property_->getTopicStd(), 10,
                [this](const hpop_msgs::msg::SatelliteState::SharedPtr msg) {
                    processMessage(msg);
                });
    }
    catch (const std::exception& e)
    {
        setStatus(rviz_common::properties::StatusProperty::Error,
                  "Topic", QString("Error subscribing: ") + e.what());
    }
}

void OrbitDisplay::unsubscribe()
{
    sub_.reset();
}

void OrbitDisplay::processMessage(const hpop_msgs::msg::SatelliteState::SharedPtr msg)
{
    std::string id = msg->satellite_id;

    // Create orbit if new satellite
    if (orbits_.find(id) == orbits_.end())
    {
        createOrbitLine(id);
    }

    // Add position to trail (convert from meters, apply scale)
    Ogre::Vector3 pos(
        static_cast<float>(msg->position.x * scale_factor_),
        static_cast<float>(msg->position.y * scale_factor_),
        static_cast<float>(msg->position.z * scale_factor_)
    );

    auto& traj = orbits_[id];
    traj.positions.push_back(pos);

    // Trim to max length
    while (traj.positions.size() > traj.max_points)
    {
        traj.positions.erase(traj.positions.begin());
    }
}

void OrbitDisplay::createOrbitLine(const std::string& satellite_id)
{
    OrbitTrajectory traj;
    traj.satellite_id = satellite_id;
    traj.max_points = trail_length_;
    traj.color = getColorForSatellite(satellite_id);

    // Create scene node
    traj.scene_node = root_node_->createChildSceneNode();

    // Create manual object for line
    traj.line_object = scene_manager_->createManualObject();
    traj.line_object->setDynamic(true);
    traj.scene_node->attachObject(traj.line_object);

    orbits_[satellite_id] = traj;
}

void OrbitDisplay::updateOrbitLine(OrbitTrajectory& traj)
{
    if (traj.positions.size() < 2 || !traj.line_object)
    {
        return;
    }

    traj.line_object->clear();
    traj.line_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (const auto& pos : traj.positions)
    {
        traj.line_object->position(pos);
        traj.line_object->colour(traj.color);
    }

    traj.line_object->end();
}

void OrbitDisplay::clearAllOrbits()
{
    for (auto& [id, traj] : orbits_)
    {
        if (traj.line_object)
        {
            traj.scene_node->detachObject(traj.line_object);
            scene_manager_->destroyManualObject(traj.line_object);
        }
        if (traj.scene_node)
        {
            scene_manager_->destroySceneNode(traj.scene_node);
        }
    }
    orbits_.clear();
}

Ogre::ColourValue OrbitDisplay::getColorForSatellite(const std::string& id)
{
    // Generate consistent color based on satellite ID hash
    size_t hash = std::hash<std::string>{}(id);

    float hue = static_cast<float>(hash % 360) / 360.0f;
    float sat = 0.8f;
    float val = 0.9f;

    // HSV to RGB conversion
    int hi = static_cast<int>(hue * 6.0f) % 6;
    float f = hue * 6.0f - hi;
    float p = val * (1.0f - sat);
    float q = val * (1.0f - f * sat);
    float t = val * (1.0f - (1.0f - f) * sat);

    float r, g, b;
    switch (hi)
    {
        case 0: r = val; g = t; b = p; break;
        case 1: r = q; g = val; b = p; break;
        case 2: r = p; g = val; b = t; break;
        case 3: r = p; g = q; b = val; break;
        case 4: r = t; g = p; b = val; break;
        default: r = val; g = p; b = q; break;
    }

    return Ogre::ColourValue(r, g, b, 1.0f);
}

} // namespace hpop_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hpop_rviz_plugins::OrbitDisplay, rviz_common::Display)
