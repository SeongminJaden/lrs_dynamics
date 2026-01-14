/**
 * @file hpop_panel.cpp
 * @brief HPOP Control Panel implementation
 *
 * Integrated GUI for:
 * - Data source selection (TLE/Observation/Sensor)
 * - Satellite management
 * - Orbit propagation control
 * - Real-time Delta-V maneuvers
 * - Robot arm control for capture operations
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QGroupBox>
#include <QScrollArea>

#include <pluginlib/class_list_macros.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hpop_rviz_plugins
{

//=============================================================================
// HpopPanel Implementation
//=============================================================================

HpopPanel::HpopPanel(QWidget* parent)
    : rviz_common::Panel(parent)
    , simulation_time_(0.0)
    , satellite_count_(0)
    , is_connected_(false)
    , current_data_source_(DataSourceType::NONE)
{
    setupUi();
}

HpopPanel::~HpopPanel()
{
    if (update_timer_)
    {
        update_timer_->stop();
    }
}

void HpopPanel::setupUi()
{
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(5, 5, 5, 5);
    main_layout->setSpacing(5);

    // Title
    auto* title_label = new QLabel("<b>HPOP Satellite Control Panel</b>");
    title_label->setAlignment(Qt::AlignCenter);
    title_label->setStyleSheet("font-size: 14px; padding: 5px;");
    main_layout->addWidget(title_label);

    // Status bar
    auto* status_group = new QGroupBox();
    auto* status_layout = new QGridLayout(status_group);
    status_layout->setContentsMargins(5, 5, 5, 5);

    status_label_ = new QLabel("Status: Disconnected");
    status_label_->setStyleSheet("color: red; font-weight: bold;");
    status_layout->addWidget(status_label_, 0, 0);

    data_source_label_ = new QLabel("Source: None");
    data_source_label_->setStyleSheet("color: gray;");
    status_layout->addWidget(data_source_label_, 0, 1);

    sim_time_label_ = new QLabel("Sim Time: 0.0 s");
    status_layout->addWidget(sim_time_label_, 1, 0);

    satellite_count_label_ = new QLabel("Satellites: 0");
    status_layout->addWidget(satellite_count_label_, 1, 1);

    main_layout->addWidget(status_group);

    // Tab widget
    tab_widget_ = new QTabWidget();

    // 1. Data Source tab (NEW - Primary entry point)
    data_source_widget_ = new DataSourceWidget();
    tab_widget_->addTab(data_source_widget_, "Data Source");

    // 2. Satellites tab
    satellite_table_ = new SatelliteTableWidget();
    tab_widget_->addTab(satellite_table_, "Satellites");

    // 3. Propagation tab
    propagation_control_ = new PropagationControlWidget();
    tab_widget_->addTab(propagation_control_, "Propagation");

    // 4. Delta-V tab (NEW - Real-time maneuver control)
    delta_v_widget_ = new DeltaVWidget();
    tab_widget_->addTab(delta_v_widget_, "Delta-V");

    // 5. Robot Arm Control tab (NEW - For capture operations)
    setupRobotArmTab();

    // 6. Analysis tab
    analysis_tab_ = new QWidget();
    auto* analysis_layout = new QVBoxLayout(analysis_tab_);
    analysis_layout->addWidget(new QLabel("<b>Contact Prediction</b>"));
    analysis_layout->addWidget(new QLabel("Ground station visibility windows"));
    analysis_layout->addWidget(new QLabel("<b>Proximity Analysis</b>"));
    analysis_layout->addWidget(new QLabel("Collision avoidance monitoring"));
    analysis_layout->addStretch();
    tab_widget_->addTab(analysis_tab_, "Analysis");

    main_layout->addWidget(tab_widget_);

    // Connect signals
    connectSignals();

    // Update timer
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &HpopPanel::onTimerUpdate);
    update_timer_->start(100);  // 10 Hz update
}

void HpopPanel::setupRobotArmTab()
{
    arm_control_tab_ = new QWidget();
    auto* arm_layout = new QVBoxLayout(arm_control_tab_);

    // Arm status group
    auto* status_group = new QGroupBox("Robot Arm Status");
    auto* status_layout = new QVBoxLayout(status_group);

    arm_status_label_ = new QLabel("Status: Idle");
    arm_status_label_->setStyleSheet("font-weight: bold;");
    status_layout->addWidget(arm_status_label_);

    arm_layout->addWidget(status_group);

    // Joint control group
    auto* joint_group = new QGroupBox("Joint Control (6-DOF)");
    auto* joint_layout = new QGridLayout(joint_group);

    const char* joint_names[] = {"J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)",
                                  "J4 (Wrist1)", "J5 (Wrist2)", "J6 (Wrist3)"};

    for (int i = 0; i < 6; i++)
    {
        joint_layout->addWidget(new QLabel(joint_names[i]), i, 0);

        auto* slider = new QSlider(Qt::Horizontal);
        slider->setRange(-180, 180);
        slider->setValue(0);
        joint_sliders_[i] = slider;
        joint_layout->addWidget(slider, i, 1);

        auto* value_label = new QLabel("0.0 deg");
        joint_value_labels_[i] = value_label;
        joint_layout->addWidget(value_label, i, 2);

        connect(slider, &QSlider::valueChanged, [this, i](int value) {
            joint_value_labels_[i]->setText(QString("%1 deg").arg(value));
        });
    }

    arm_layout->addWidget(joint_group);

    // Preset positions
    auto* preset_group = new QGroupBox("Preset Positions");
    auto* preset_layout = new QHBoxLayout(preset_group);

    arm_home_button_ = new QPushButton("Home");
    arm_home_button_->setToolTip("Move arm to home position");
    preset_layout->addWidget(arm_home_button_);

    arm_ready_button_ = new QPushButton("Ready");
    arm_ready_button_->setToolTip("Move arm to ready/capture position");
    preset_layout->addWidget(arm_ready_button_);

    arm_capture_button_ = new QPushButton("Capture");
    arm_capture_button_->setToolTip("Execute capture sequence");
    arm_capture_button_->setStyleSheet("font-weight: bold;");
    preset_layout->addWidget(arm_capture_button_);

    preset_layout->addStretch();
    arm_layout->addWidget(preset_group);

    // Magnetic docking control
    auto* dock_group = new QGroupBox("Magnetic Docking");
    auto* dock_layout = new QHBoxLayout(dock_group);

    dock_activate_button_ = new QPushButton("Activate Magnet");
    dock_activate_button_->setStyleSheet("background-color: #4CAF50; color: white;");
    dock_layout->addWidget(dock_activate_button_);

    dock_deactivate_button_ = new QPushButton("Release");
    dock_deactivate_button_->setStyleSheet("background-color: #f44336; color: white;");
    dock_layout->addWidget(dock_deactivate_button_);

    dock_status_label_ = new QLabel("Magnet: Off");
    dock_layout->addWidget(dock_status_label_);
    dock_layout->addStretch();

    arm_layout->addWidget(dock_group);

    arm_layout->addStretch();

    // Connect arm control signals
    connect(arm_home_button_, &QPushButton::clicked, this, &HpopPanel::onArmHomeClicked);
    connect(arm_ready_button_, &QPushButton::clicked, this, &HpopPanel::onArmReadyClicked);
    connect(arm_capture_button_, &QPushButton::clicked, this, &HpopPanel::onArmCaptureClicked);
    connect(dock_activate_button_, &QPushButton::clicked, this, &HpopPanel::onDockActivateClicked);
    connect(dock_deactivate_button_, &QPushButton::clicked, this, &HpopPanel::onDockDeactivateClicked);

    tab_widget_->addTab(arm_control_tab_, "Robot Arm");
}

void HpopPanel::connectSignals()
{
    // Data source widget signals
    connect(data_source_widget_, &DataSourceWidget::dataSourceChanged,
            this, &HpopPanel::onDataSourceChanged);
    connect(data_source_widget_, &DataSourceWidget::configurationComplete,
            this, &HpopPanel::onConfigurationComplete);
    connect(data_source_widget_, &DataSourceWidget::fetchTleRequested,
            this, &HpopPanel::onFetchTleRequested);
    connect(data_source_widget_, &DataSourceWidget::addSatelliteRequested,
            this, &HpopPanel::onAddSatelliteRequested);

    // Satellite table signals
    connect(satellite_table_, &SatelliteTableWidget::removeSatelliteRequested,
            this, &HpopPanel::onRemoveSatelliteRequested);
    connect(satellite_table_, &SatelliteTableWidget::satelliteSelected,
            this, &HpopPanel::onSatelliteSelected);

    // Propagation control signals
    connect(propagation_control_, &PropagationControlWidget::startPropagation,
            this, &HpopPanel::onStartPropagation);
    connect(propagation_control_, &PropagationControlWidget::stopPropagation,
            this, &HpopPanel::onStopPropagation);
    connect(propagation_control_, &PropagationControlWidget::resetPropagation,
            this, &HpopPanel::onResetPropagation);

    // Delta-V widget signals
    connect(delta_v_widget_, &DeltaVWidget::deltaVChanged,
            this, &HpopPanel::onDeltaVChanged);
    connect(delta_v_widget_, &DeltaVWidget::applyManeuverRequested,
            this, &HpopPanel::onApplyManeuver);
}

void HpopPanel::onInitialize()
{
    // Create ROS2 node
    node_ = std::make_shared<rclcpp::Node>("hpop_panel_node");

    setupRosConnections();
}

void HpopPanel::setupRosConnections()
{
    // Subscribe to satellite states
    state_sub_ = node_->create_subscription<hpop_msgs::msg::SatelliteState>(
        "/hpop/satellite_state", 10,
        [this](const hpop_msgs::msg::SatelliteState::SharedPtr msg) {
            // Calculate altitude from position
            double r = std::sqrt(msg->position.x * msg->position.x +
                                 msg->position.y * msg->position.y +
                                 msg->position.z * msg->position.z);
            double altitude_km = (r - 6378137.0) / 1000.0;

            // Calculate velocity magnitude
            double velocity_kms = std::sqrt(msg->velocity.x * msg->velocity.x +
                                            msg->velocity.y * msg->velocity.y +
                                            msg->velocity.z * msg->velocity.z) / 1000.0;

            // Get orbital period
            double period_min = msg->elements.period / 60.0;
            if (period_min <= 0.0) {
                double sma = msg->elements.semi_major_axis;
                double mu = 3.986004418e14;
                period_min = (sma > 0) ?
                             2.0 * M_PI * std::sqrt(sma * sma * sma / mu) / 60.0 : 0.0;
            }

            std::string sat_name = msg->name.empty() ? msg->satellite_id : msg->name;
            satellite_table_->updateSatellite(sat_name, altitude_km, velocity_kms, period_min);
            is_connected_ = true;
        });

    // Service clients
    fetch_tle_client_ = node_->create_client<hpop_msgs::srv::FetchTLE>("/hpop/fetch_tle");
    add_sat_client_ = node_->create_client<hpop_msgs::srv::AddSatellite>("/hpop/add_satellite");
    maneuver_client_ = node_->create_client<hpop_msgs::srv::PlanManeuver>("/hpop/plan_maneuver");

    // Delta-V publisher
    delta_v_pub_ = node_->create_publisher<hpop_msgs::msg::ManeuverPlan>("/hpop/delta_v_command", 10);

    // Robot arm trajectory publisher
    arm_trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory", 10);

    // Magnetic docking service clients
    dock_activate_client_ = node_->create_client<std_srvs::srv::Trigger>("/hpop/activate_dock");
    dock_deactivate_client_ = node_->create_client<std_srvs::srv::Trigger>("/hpop/deactivate_dock");
}

void HpopPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

void HpopPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void HpopPanel::onDataSourceChanged(DataSourceType type)
{
    current_data_source_ = type;

    QString source_name;
    switch (type)
    {
        case DataSourceType::TLE:
            source_name = "TLE";
            break;
        case DataSourceType::OBSERVATION:
            source_name = "Orbital Elements";
            break;
        case DataSourceType::SENSOR:
            source_name = "Sensor";
            break;
        default:
            source_name = "None";
            break;
    }
    data_source_label_->setText("Source: " + source_name);

    // Disable propagation until configuration is complete
    propagation_control_->setStartEnabled(false);
}

void HpopPanel::onConfigurationComplete()
{
    // Enable propagation control
    propagation_control_->setStartEnabled(true);

    // Update status
    status_label_->setText("Status: Ready");
    status_label_->setStyleSheet("color: blue; font-weight: bold;");

    // Switch to Propagation tab
    tab_widget_->setCurrentIndex(2);  // Propagation tab

    // Spawn satellite in Gazebo if needed
    publishSpawnRequest();
}

void HpopPanel::onFetchTleRequested(uint32_t norad_id)
{
    if (!fetch_tle_client_->wait_for_service(std::chrono::seconds(1)))
    {
        QMessageBox::warning(this, "Service Unavailable",
                             "TLE fetch service is not available.");
        return;
    }

    auto request = std::make_shared<hpop_msgs::srv::FetchTLE::Request>();
    request->norad_ids.push_back(norad_id);

    status_label_->setText("Fetching TLE for NORAD " + QString::number(norad_id) + "...");
    status_label_->setStyleSheet("color: blue; font-weight: bold;");

    auto future = fetch_tle_client_->async_send_request(request);
}

void HpopPanel::onAddSatelliteRequested()
{
    // Get data from DataSourceWidget based on selected source
    switch (current_data_source_)
    {
        case DataSourceType::TLE:
        {
            uint32_t norad_id = data_source_widget_->getNoradId();
            QString tle1 = data_source_widget_->getTleLine1();
            QString tle2 = data_source_widget_->getTleLine2();

            if (norad_id > 0 || (!tle1.isEmpty() && !tle2.isEmpty()))
            {
                satellite_table_->addSatellite(
                    QString("SAT-%1").arg(norad_id).toStdString(),
                    norad_id, 400.0, 51.6);
                satellite_count_++;
            }
            break;
        }
        case DataSourceType::OBSERVATION:
        {
            double sma = data_source_widget_->getSemiMajorAxis();
            double inc = data_source_widget_->getInclination();
            double altitude_km = (sma - 6378137.0) / 1000.0;
            double inc_deg = inc * 180.0 / M_PI;

            satellite_table_->addSatellite("Custom-Sat", 0, altitude_km, inc_deg);
            satellite_count_++;
            break;
        }
        case DataSourceType::SENSOR:
        {
            satellite_table_->addSatellite("Sensor-Sat", 0, 400.0, 51.6);
            satellite_count_++;
            break;
        }
        default:
            break;
    }

    updateStatusDisplay();
}

void HpopPanel::onSatelliteSelected(const QString& name)
{
    selected_satellite_ = name;
    status_label_->setText("Selected: " + name);
}

void HpopPanel::onRemoveSatelliteRequested(const QString& name)
{
    satellite_table_->removeSatellite(name.toStdString());
    satellite_count_--;
    if (satellite_count_ < 0) satellite_count_ = 0;

    // Disable propagation if no satellites
    if (satellite_count_ == 0)
    {
        propagation_control_->setStartEnabled(false);
    }
    updateStatusDisplay();
}

void HpopPanel::onStartPropagation()
{
    status_label_->setText("Status: Propagating");
    status_label_->setStyleSheet("color: green; font-weight: bold;");
}

void HpopPanel::onStopPropagation()
{
    status_label_->setText("Status: Paused");
    status_label_->setStyleSheet("color: orange; font-weight: bold;");
}

void HpopPanel::onResetPropagation()
{
    simulation_time_ = 0.0;
    updateStatusDisplay();
}

void HpopPanel::onDeltaVChanged(double dvr, double dvt, double dvn)
{
    // Real-time update of delta-V preview (could publish to a preview topic)
    (void)dvr; (void)dvt; (void)dvn;
}

void HpopPanel::onApplyManeuver()
{
    if (selected_satellite_.isEmpty())
    {
        QMessageBox::warning(this, "No Satellite Selected",
                             "Please select a satellite in the Satellites tab first.");
        return;
    }

    // Create and publish maneuver command
    auto msg = hpop_msgs::msg::ManeuverPlan();
    msg.header.stamp = node_->now();
    msg.satellite_id = selected_satellite_.toStdString();
    msg.maneuver_type = msg.TYPE_IMPULSIVE;

    // RTN components
    msg.dv_radial = delta_v_widget_->getDeltaVR();
    msg.dv_intrack = delta_v_widget_->getDeltaVT();
    msg.dv_crosstrack = delta_v_widget_->getDeltaVN();

    // Also set delta_v_rtn vector
    msg.delta_v_rtn.x = msg.dv_radial;
    msg.delta_v_rtn.y = msg.dv_intrack;
    msg.delta_v_rtn.z = msg.dv_crosstrack;

    // Calculate magnitude
    msg.delta_v_magnitude = std::sqrt(
        msg.dv_radial * msg.dv_radial +
        msg.dv_intrack * msg.dv_intrack +
        msg.dv_crosstrack * msg.dv_crosstrack);

    // Set execution time
    double exec_time = delta_v_widget_->getManeuverTime();
    msg.ignition_time.sec = static_cast<int32_t>(exec_time);
    msg.ignition_time.nanosec = static_cast<uint32_t>((exec_time - msg.ignition_time.sec) * 1e9);

    msg.status = msg.STATUS_PLANNED;

    delta_v_pub_->publish(msg);

    status_label_->setText("Delta-V applied to " + selected_satellite_);
    status_label_->setStyleSheet("color: green; font-weight: bold;");
}

void HpopPanel::onTimerUpdate()
{
    if (node_)
    {
        rclcpp::spin_some(node_);
    }

    if (propagation_control_->isPropagating())
    {
        simulation_time_ += 0.1 * propagation_control_->getTimeScale();
    }

    updateStatusDisplay();
}

void HpopPanel::updateStatusDisplay()
{
    if (is_connected_)
    {
        if (propagation_control_->isPropagating())
        {
            status_label_->setText("Status: Propagating");
            status_label_->setStyleSheet("color: green; font-weight: bold;");
        }
        else
        {
            status_label_->setText("Status: Connected");
            status_label_->setStyleSheet("color: blue; font-weight: bold;");
        }
    }

    sim_time_label_->setText(QString("Sim Time: %1 s").arg(simulation_time_, 0, 'f', 1));
    satellite_count_label_->setText(QString("Satellites: %1").arg(
        satellite_table_->getSatelliteCount()));
}

void HpopPanel::publishSpawnRequest()
{
    // TODO: Publish spawn request to Gazebo
    // This would trigger satellite spawning in Gazebo based on the configured data source
}

// Robot arm control slots
void HpopPanel::onArmHomeClicked()
{
    publishArmTrajectory({0, 0, 0, 0, 0, 0});
    arm_status_label_->setText("Status: Moving to Home");
}

void HpopPanel::onArmReadyClicked()
{
    publishArmTrajectory({0, -45, 90, -45, 90, 0});
    arm_status_label_->setText("Status: Moving to Ready");
}

void HpopPanel::onArmCaptureClicked()
{
    publishArmTrajectory({0, -30, 60, -30, 45, 0});
    arm_status_label_->setText("Status: Executing Capture");

    // Auto-activate magnet after reaching capture position
    QTimer::singleShot(3000, this, &HpopPanel::onDockActivateClicked);
}

void HpopPanel::onDockActivateClicked()
{
    if (dock_activate_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        dock_activate_client_->async_send_request(request);
        dock_status_label_->setText("Magnet: Active");
        dock_status_label_->setStyleSheet("color: green; font-weight: bold;");
    }
    else
    {
        dock_status_label_->setText("Magnet: Service N/A");
        dock_status_label_->setStyleSheet("color: red;");
    }
}

void HpopPanel::onDockDeactivateClicked()
{
    if (dock_deactivate_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        dock_deactivate_client_->async_send_request(request);
        dock_status_label_->setText("Magnet: Off");
        dock_status_label_->setStyleSheet("color: gray;");
    }
}

void HpopPanel::publishArmTrajectory(const std::vector<double>& joint_positions_deg)
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    traj.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (size_t i = 0; i < 6 && i < joint_positions_deg.size(); i++)
    {
        point.positions.push_back(joint_positions_deg[i] * M_PI / 180.0);  // deg to rad
        point.velocities.push_back(0.0);
    }
    point.time_from_start.sec = 3;  // 3 seconds to reach position
    point.time_from_start.nanosec = 0;

    traj.points.push_back(point);
    arm_trajectory_pub_->publish(traj);
}

} // namespace hpop_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hpop_rviz_plugins::HpopPanel, rviz_common::Panel)
