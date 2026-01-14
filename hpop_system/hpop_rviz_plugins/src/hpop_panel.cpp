/**
 * @file hpop_panel.cpp
 * @brief HPOP Control Panel implementation
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QMessageBox>

#include <pluginlib/class_list_macros.hpp>

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
    auto* title_label = new QLabel("<b>HPOP Control Panel</b>");
    title_label->setAlignment(Qt::AlignCenter);
    main_layout->addWidget(title_label);

    // Status bar
    auto* status_layout = new QHBoxLayout();
    status_label_ = new QLabel("Status: Disconnected");
    status_label_->setStyleSheet("color: red;");
    sim_time_label_ = new QLabel("Time: 0.0 s");
    satellite_count_label_ = new QLabel("Satellites: 0");
    status_layout->addWidget(status_label_);
    status_layout->addStretch();
    status_layout->addWidget(sim_time_label_);
    status_layout->addWidget(satellite_count_label_);
    main_layout->addLayout(status_layout);

    // Tab widget
    tab_widget_ = new QTabWidget();

    // Satellites tab
    satellite_table_ = new SatelliteTableWidget();
    tab_widget_->addTab(satellite_table_, "Satellites");

    // Propagation tab
    propagation_control_ = new PropagationControlWidget();
    tab_widget_->addTab(propagation_control_, "Propagation");

    // Analysis tab (placeholder)
    analysis_tab_ = new QWidget();
    auto* analysis_layout = new QVBoxLayout(analysis_tab_);
    analysis_layout->addWidget(new QLabel("Contact Prediction"));
    analysis_layout->addWidget(new QLabel("Proximity Analysis"));
    analysis_layout->addStretch();
    tab_widget_->addTab(analysis_tab_, "Analysis");

    // Maneuver tab (placeholder)
    maneuver_tab_ = new QWidget();
    auto* maneuver_layout = new QVBoxLayout(maneuver_tab_);
    maneuver_layout->addWidget(new QLabel("Maneuver Planning"));
    maneuver_layout->addStretch();
    tab_widget_->addTab(maneuver_tab_, "Maneuver");

    main_layout->addWidget(tab_widget_);

    // Connect signals
    connect(satellite_table_, &SatelliteTableWidget::addSatelliteRequested,
            this, &HpopPanel::onAddSatelliteRequested);
    connect(satellite_table_, &SatelliteTableWidget::removeSatelliteRequested,
            this, &HpopPanel::onRemoveSatelliteRequested);
    connect(satellite_table_, &SatelliteTableWidget::satelliteSelected,
            this, &HpopPanel::onSatelliteSelected);

    connect(propagation_control_, &PropagationControlWidget::startPropagation,
            this, &HpopPanel::onStartPropagation);
    connect(propagation_control_, &PropagationControlWidget::stopPropagation,
            this, &HpopPanel::onStopPropagation);
    connect(propagation_control_, &PropagationControlWidget::resetPropagation,
            this, &HpopPanel::onResetPropagation);

    // Update timer
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &HpopPanel::onTimerUpdate);
    update_timer_->start(100);  // 10 Hz update
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
            double altitude_km = (r - 6378137.0) / 1000.0;  // Earth radius = 6378137 m

            // Calculate velocity magnitude
            double velocity_kms = std::sqrt(msg->velocity.x * msg->velocity.x +
                                            msg->velocity.y * msg->velocity.y +
                                            msg->velocity.z * msg->velocity.z) / 1000.0;

            // Calculate orbital period from semi-major axis
            double sma = msg->elements.semi_major_axis;
            double mu = 3.986004418e14;  // Earth GM
            double period_min = (sma > 0) ?
                                2.0 * M_PI * std::sqrt(sma * sma * sma / mu) / 60.0 : 0.0;

            // Update satellite in table
            satellite_table_->updateSatellite(
                msg->satellite_id,
                altitude_km,
                velocity_kms,
                period_min
            );
            is_connected_ = true;
        });

    // Service clients
    fetch_tle_client_ = node_->create_client<hpop_msgs::srv::FetchTLE>("/hpop/fetch_tle");
    add_sat_client_ = node_->create_client<hpop_msgs::srv::AddSatellite>("/hpop/add_satellite");
}

void HpopPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
    // Save panel state
}

void HpopPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
    // Load panel state
}

void HpopPanel::onSatelliteSelected(const QString& name)
{
    // Handle satellite selection
    (void)name;
}

void HpopPanel::onAddSatelliteRequested(uint32_t norad_id)
{
    if (!fetch_tle_client_->wait_for_service(std::chrono::seconds(1)))
    {
        QMessageBox::warning(this, "Service Unavailable",
                             "TLE fetch service is not available.");
        return;
    }

    auto request = std::make_shared<hpop_msgs::srv::FetchTLE::Request>();
    request->norad_ids.push_back(norad_id);

    auto future = fetch_tle_client_->async_send_request(request);

    // Note: In production, handle async properly
    status_label_->setText("Fetching TLE for NORAD " + QString::number(norad_id) + "...");
}

void HpopPanel::onRemoveSatelliteRequested(const QString& name)
{
    satellite_table_->removeSatellite(name.toStdString());
    satellite_count_--;
    updateStatusDisplay();
}

void HpopPanel::onStartPropagation()
{
    status_label_->setText("Status: Propagating");
    status_label_->setStyleSheet("color: green;");
}

void HpopPanel::onStopPropagation()
{
    status_label_->setText("Status: Paused");
    status_label_->setStyleSheet("color: orange;");
}

void HpopPanel::onResetPropagation()
{
    simulation_time_ = 0.0;
    updateStatusDisplay();
}

void HpopPanel::onTimerUpdate()
{
    // Spin ROS node
    if (node_)
    {
        rclcpp::spin_some(node_);
    }

    // Update display
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
        status_label_->setText("Status: Connected");
        status_label_->setStyleSheet("color: green;");
    }

    sim_time_label_->setText(QString("Time: %1 s").arg(simulation_time_, 0, 'f', 1));
    satellite_count_label_->setText(QString("Satellites: %1").arg(satellite_count_));
}

} // namespace hpop_rviz_plugins

PLUGINLIB_EXPORT_CLASS(hpop_rviz_plugins::HpopPanel, rviz_common::Panel)
