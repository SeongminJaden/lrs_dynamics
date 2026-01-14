/**
 * @file hpop_panel.hpp
 * @brief HPOP Control Panel for RViz2
 *
 * Main panel widget combining satellite management,
 * propagation control, and analysis displays.
 */

#ifndef HPOP_RVIZ_PLUGINS_HPOP_PANEL_HPP
#define HPOP_RVIZ_PLUGINS_HPOP_PANEL_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QGroupBox>
#include <QTimer>
#include <QTableWidget>

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "hpop_msgs/msg/satellite_state.hpp"
#include "hpop_msgs/msg/tle_data.hpp"
#include "hpop_msgs/srv/fetch_tle.hpp"
#include "hpop_msgs/srv/add_satellite.hpp"

namespace hpop_rviz_plugins
{

/**
 * @brief Satellite table widget for managing satellites
 */
class SatelliteTableWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SatelliteTableWidget(QWidget* parent = nullptr);
    ~SatelliteTableWidget() override = default;

    void addSatellite(const std::string& name, uint32_t norad_id,
                      double altitude_km, double inclination_deg);
    void updateSatellite(const std::string& name, double altitude_km,
                         double velocity_kms, double period_min);
    void removeSatellite(const std::string& name);
    void clearAll();

Q_SIGNALS:
    void satelliteSelected(const QString& name);
    void addSatelliteRequested(uint32_t norad_id);
    void removeSatelliteRequested(const QString& name);

private Q_SLOTS:
    void onAddButtonClicked();
    void onRemoveButtonClicked();
    void onTableSelectionChanged();

private:
    void setupUi();

    QTableWidget* table_;
    QLineEdit* norad_input_;
    QPushButton* add_button_;
    QPushButton* remove_button_;
    QPushButton* fetch_tle_button_;
};

/**
 * @brief Propagation control widget
 */
class PropagationControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PropagationControlWidget(QWidget* parent = nullptr);
    ~PropagationControlWidget() override = default;

    bool isPropagating() const { return is_propagating_; }
    double getTimeScale() const;
    double getStepSize() const;

Q_SIGNALS:
    void startPropagation();
    void stopPropagation();
    void resetPropagation();
    void timeScaleChanged(double scale);
    void stepSizeChanged(double step);
    void perturbationsChanged();

private Q_SLOTS:
    void onStartStopClicked();
    void onResetClicked();
    void onTimeScaleChanged(int value);

private:
    void setupUi();
    void updateButtonState();

    bool is_propagating_;

    QPushButton* start_stop_button_;
    QPushButton* reset_button_;
    QSlider* time_scale_slider_;
    QLabel* time_scale_label_;
    QDoubleSpinBox* step_size_spin_;

    // Perturbation checkboxes
    QCheckBox* j2_check_;
    QCheckBox* drag_check_;
    QCheckBox* srp_check_;
    QCheckBox* third_body_check_;

    // Integrator selection
    QComboBox* integrator_combo_;
    QSpinBox* gravity_degree_spin_;
};

/**
 * @brief Main HPOP Panel widget
 */
class HpopPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit HpopPanel(QWidget* parent = nullptr);
    ~HpopPanel() override;

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config& config) override;

protected Q_SLOTS:
    void onSatelliteSelected(const QString& name);
    void onAddSatelliteRequested(uint32_t norad_id);
    void onRemoveSatelliteRequested(const QString& name);
    void onStartPropagation();
    void onStopPropagation();
    void onResetPropagation();
    void onTimerUpdate();

private:
    void setupUi();
    void setupRosConnections();
    void updateStatusDisplay();

    // UI Components
    QTabWidget* tab_widget_;
    SatelliteTableWidget* satellite_table_;
    PropagationControlWidget* propagation_control_;
    QWidget* analysis_tab_;
    QWidget* maneuver_tab_;

    // Status display
    QLabel* status_label_;
    QLabel* sim_time_label_;
    QLabel* satellite_count_label_;

    // ROS2 node and connections
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<hpop_msgs::msg::SatelliteState>::SharedPtr state_sub_;
    rclcpp::Client<hpop_msgs::srv::FetchTLE>::SharedPtr fetch_tle_client_;
    rclcpp::Client<hpop_msgs::srv::AddSatellite>::SharedPtr add_sat_client_;

    // Update timer
    QTimer* update_timer_;

    // State
    double simulation_time_;
    int satellite_count_;
    bool is_connected_;
};

} // namespace hpop_rviz_plugins

#endif // HPOP_RVIZ_PLUGINS_HPOP_PANEL_HPP
