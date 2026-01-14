#include "hpop_gazebo/mission_control_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QMessageBox>

namespace hpop_gazebo
{

MissionControlPanel::MissionControlPanel(QWidget* parent)
  : rviz_common::Panel(parent),
    current_status_("Unknown"),
    current_distance_(0.0),
    orbit_view_process_(nullptr)
{
  // Main layout
  auto* main_layout = new QVBoxLayout(this);

  // === HPOP Control Group ===
  auto* hpop_group = new QGroupBox("HPOP Orbit Propagation");
  auto* hpop_layout = new QHBoxLayout();

  start_hpop_btn_ = new QPushButton("Start");
  start_hpop_btn_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
  stop_hpop_btn_ = new QPushButton("Stop");
  stop_hpop_btn_->setStyleSheet("background-color: #f44336; color: white;");
  reset_hpop_btn_ = new QPushButton("Reset");
  reset_hpop_btn_->setStyleSheet("background-color: #ff9800; color: white;");

  hpop_layout->addWidget(start_hpop_btn_);
  hpop_layout->addWidget(stop_hpop_btn_);
  hpop_layout->addWidget(reset_hpop_btn_);
  hpop_group->setLayout(hpop_layout);
  main_layout->addWidget(hpop_group);

  // === Orbit View Button ===
  orbit_view_btn_ = new QPushButton("Open Orbit View (New Window)");
  orbit_view_btn_->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 10px;");
  main_layout->addWidget(orbit_view_btn_);

  // === Rendezvous Control Group ===
  auto* rendezvous_group = new QGroupBox("Rendezvous & Docking");
  auto* rendezvous_layout = new QHBoxLayout();

  start_rendezvous_btn_ = new QPushButton("Start Rendezvous");
  start_rendezvous_btn_->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
  abort_rendezvous_btn_ = new QPushButton("Abort");
  abort_rendezvous_btn_->setStyleSheet("background-color: #f44336; color: white;");
  capture_btn_ = new QPushButton("Capture");
  capture_btn_->setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold;");

  rendezvous_layout->addWidget(start_rendezvous_btn_);
  rendezvous_layout->addWidget(abort_rendezvous_btn_);
  rendezvous_layout->addWidget(capture_btn_);
  rendezvous_group->setLayout(rendezvous_layout);
  main_layout->addWidget(rendezvous_group);

  // === Status Display ===
  auto* status_group = new QGroupBox("Mission Status");
  auto* status_layout = new QVBoxLayout();

  status_label_ = new QLabel("Status: Waiting...");
  status_label_->setStyleSheet("font-size: 12px; padding: 5px; background-color: #333; color: #0f0;");
  distance_label_ = new QLabel("Distance: -- m");
  distance_label_->setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;");

  status_layout->addWidget(status_label_);
  status_layout->addWidget(distance_label_);
  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  main_layout->addStretch();

  // Connect buttons
  connect(start_hpop_btn_, &QPushButton::clicked, this, &MissionControlPanel::onStartHpop);
  connect(stop_hpop_btn_, &QPushButton::clicked, this, &MissionControlPanel::onStopHpop);
  connect(reset_hpop_btn_, &QPushButton::clicked, this, &MissionControlPanel::onResetHpop);
  connect(orbit_view_btn_, &QPushButton::clicked, this, &MissionControlPanel::onOpenOrbitView);
  connect(start_rendezvous_btn_, &QPushButton::clicked, this, &MissionControlPanel::onStartRendezvous);
  connect(abort_rendezvous_btn_, &QPushButton::clicked, this, &MissionControlPanel::onAbortRendezvous);
  connect(capture_btn_, &QPushButton::clicked, this, &MissionControlPanel::onCapture);

  // Timer for UI updates
  update_timer_ = new QTimer(this);
  connect(update_timer_, &QTimer::timeout, this, &MissionControlPanel::updateStatus);
  update_timer_->start(200);  // 5Hz update
}

MissionControlPanel::~MissionControlPanel()
{
  if (orbit_view_process_ && orbit_view_process_->state() == QProcess::Running) {
    orbit_view_process_->terminate();
    orbit_view_process_->waitForFinished(3000);
  }
}

void MissionControlPanel::onInitialize()
{
  // Get ROS node from display context
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create service clients
  start_hpop_client_ = node_->create_client<std_srvs::srv::Trigger>("/hpop/start_propagation");
  stop_hpop_client_ = node_->create_client<std_srvs::srv::Trigger>("/hpop/stop_propagation");
  reset_hpop_client_ = node_->create_client<std_srvs::srv::Trigger>("/hpop/reset_propagation");
  start_rendezvous_client_ = node_->create_client<std_srvs::srv::Trigger>("/rendezvous/start");
  abort_rendezvous_client_ = node_->create_client<std_srvs::srv::Trigger>("/rendezvous/abort");
  capture_client_ = node_->create_client<std_srvs::srv::Trigger>("/rendezvous/capture");

  // Create subscribers
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/rendezvous/status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      current_status_ = msg->data;
    });

  distance_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/rendezvous/distance", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      current_distance_ = msg->data;
    });
}

void MissionControlPanel::callService(const std::string& service_name)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  if (service_name == "start_hpop") client = start_hpop_client_;
  else if (service_name == "stop_hpop") client = stop_hpop_client_;
  else if (service_name == "reset_hpop") client = reset_hpop_client_;
  else if (service_name == "start_rendezvous") client = start_rendezvous_client_;
  else if (service_name == "abort_rendezvous") client = abort_rendezvous_client_;
  else if (service_name == "capture") client = capture_client_;

  if (!client) return;

  if (!client->wait_for_service(std::chrono::milliseconds(100))) {
    status_label_->setText("Status: Service not available!");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  // Non-blocking - just send and continue
  status_label_->setText("Status: Command sent...");
}

void MissionControlPanel::onStartHpop()
{
  callService("start_hpop");
}

void MissionControlPanel::onStopHpop()
{
  callService("stop_hpop");
}

void MissionControlPanel::onResetHpop()
{
  callService("reset_hpop");
}

void MissionControlPanel::onOpenOrbitView()
{
  // Open orbit view RViz in a new window
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("hpop_gazebo");
    std::string config_path = pkg_path + "/config/orbit_view.rviz";

    if (orbit_view_process_ && orbit_view_process_->state() == QProcess::Running) {
      QMessageBox::information(this, "Info", "Orbit View is already open!");
      return;
    }

    orbit_view_process_ = new QProcess(this);
    QStringList args;
    args << "-d" << QString::fromStdString(config_path);
    args << "--ros-args" << "-p" << "use_sim_time:=true";

    orbit_view_process_->start("rviz2", args);
    status_label_->setText("Status: Orbit View opened!");
  } catch (const std::exception& e) {
    QMessageBox::warning(this, "Error", QString("Failed to open Orbit View: %1").arg(e.what()));
  }
}

void MissionControlPanel::onStartRendezvous()
{
  callService("start_rendezvous");
}

void MissionControlPanel::onAbortRendezvous()
{
  callService("abort_rendezvous");
}

void MissionControlPanel::onCapture()
{
  callService("capture");
}

void MissionControlPanel::updateStatus()
{
  status_label_->setText(QString("Status: %1").arg(QString::fromStdString(current_status_)));
  distance_label_->setText(QString("Distance: %1 m").arg(current_distance_, 0, 'f', 3));

  // Color code distance
  if (current_distance_ < 0.35) {
    distance_label_->setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px; color: #4CAF50;");
  } else if (current_distance_ < 0.5) {
    distance_label_->setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px; color: #ff9800;");
  } else {
    distance_label_->setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px; color: white;");
  }
}

void MissionControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void MissionControlPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

}  // namespace hpop_gazebo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hpop_gazebo::MissionControlPanel, rviz_common::Panel)
