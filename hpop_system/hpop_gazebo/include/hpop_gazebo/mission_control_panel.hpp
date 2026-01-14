#ifndef HPOP_GAZEBO__MISSION_CONTROL_PANEL_HPP_
#define HPOP_GAZEBO__MISSION_CONTROL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QProcess>
#include <QTimer>

namespace hpop_gazebo
{

class MissionControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MissionControlPanel(QWidget* parent = nullptr);
  ~MissionControlPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config& config) override;

private Q_SLOTS:
  void onStartHpop();
  void onStopHpop();
  void onResetHpop();
  void onOpenOrbitView();
  void onStartRendezvous();
  void onAbortRendezvous();
  void onCapture();
  void updateStatus();

private:
  void callService(const std::string& service_name);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_hpop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_hpop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_hpop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_rendezvous_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr abort_rendezvous_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;

  // UI
  QPushButton* start_hpop_btn_;
  QPushButton* stop_hpop_btn_;
  QPushButton* reset_hpop_btn_;
  QPushButton* orbit_view_btn_;
  QPushButton* start_rendezvous_btn_;
  QPushButton* abort_rendezvous_btn_;
  QPushButton* capture_btn_;
  QLabel* status_label_;
  QLabel* distance_label_;

  QTimer* update_timer_;
  QProcess* orbit_view_process_;

  std::string current_status_;
  double current_distance_;
};

}  // namespace hpop_gazebo

#endif  // HPOP_GAZEBO__MISSION_CONTROL_PANEL_HPP_
