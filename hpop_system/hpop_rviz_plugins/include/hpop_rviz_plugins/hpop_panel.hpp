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
#include <QRadioButton>
#include <QButtonGroup>
#include <QStackedWidget>
#include <QSlider>

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "hpop_msgs/msg/satellite_state.hpp"
#include "hpop_msgs/msg/tle_data.hpp"
#include "hpop_msgs/msg/maneuver_plan.hpp"
#include "hpop_msgs/srv/fetch_tle.hpp"
#include "hpop_msgs/srv/add_satellite.hpp"
#include "hpop_msgs/srv/plan_maneuver.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hpop_rviz_plugins
{

/**
 * @brief Data source types
 */
enum class DataSourceType
{
    NONE,
    TLE,
    OBSERVATION,
    SENSOR
};

/**
 * @brief Data source selection widget
 */
class DataSourceWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DataSourceWidget(QWidget* parent = nullptr);
    ~DataSourceWidget() override = default;

    DataSourceType getSelectedSource() const { return selected_source_; }
    bool isConfigured() const { return is_configured_; }

    // TLE mode
    uint32_t getNoradId() const;
    QString getTleLine1() const;
    QString getTleLine2() const;

    // Observation mode
    double getSemiMajorAxis() const;
    double getEccentricity() const;
    double getInclination() const;
    double getRaan() const;
    double getArgPeriapsis() const;
    double getTrueAnomaly() const;
    double getMass() const;
    double getCrossSection() const;

Q_SIGNALS:
    void dataSourceChanged(DataSourceType type);
    void configurationComplete();
    void fetchTleRequested(uint32_t norad_id);
    void addSatelliteRequested();

private Q_SLOTS:
    void onSourceTypeChanged();
    void onFetchTleClicked();
    void onApplyClicked();

private:
    void setupUi();
    void updateConfigurationState();

    DataSourceType selected_source_;
    bool is_configured_;

    // Source selection
    QButtonGroup* source_group_;
    QRadioButton* tle_radio_;
    QRadioButton* observation_radio_;
    QRadioButton* sensor_radio_;
    QStackedWidget* source_stack_;

    // TLE input
    QWidget* tle_page_;
    QLineEdit* norad_input_;
    QLineEdit* tle_line1_input_;
    QLineEdit* tle_line2_input_;
    QPushButton* fetch_tle_button_;
    QLabel* tle_status_label_;

    // Observation input (manual orbital elements)
    QWidget* observation_page_;
    QDoubleSpinBox* sma_spin_;      // Semi-major axis [km]
    QDoubleSpinBox* ecc_spin_;      // Eccentricity
    QDoubleSpinBox* inc_spin_;      // Inclination [deg]
    QDoubleSpinBox* raan_spin_;     // RAAN [deg]
    QDoubleSpinBox* aop_spin_;      // Arg of periapsis [deg]
    QDoubleSpinBox* ta_spin_;       // True anomaly [deg]
    QDoubleSpinBox* mass_spin_;     // Mass [kg]
    QDoubleSpinBox* cs_spin_;       // Cross section [m^2]

    // Sensor input
    QWidget* sensor_page_;
    QLabel* sensor_status_label_;
    QComboBox* sensor_topic_combo_;

    // Apply button
    QPushButton* apply_button_;
};

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
    int getSatelliteCount() const;

Q_SIGNALS:
    void satelliteSelected(const QString& name);
    void removeSatelliteRequested(const QString& name);

private Q_SLOTS:
    void onRemoveButtonClicked();
    void onTableSelectionChanged();
    void onClearAllClicked();

private:
    void setupUi();

    QTableWidget* table_;
    QPushButton* remove_button_;
    QPushButton* clear_all_button_;
};

/**
 * @brief Delta-V maneuver control widget
 */
class DeltaVWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DeltaVWidget(QWidget* parent = nullptr);
    ~DeltaVWidget() override = default;

    // Get Delta-V components in RTN frame (Radial, Transverse, Normal)
    double getDeltaVR() const;  // Radial [m/s]
    double getDeltaVT() const;  // Transverse (along-track) [m/s]
    double getDeltaVN() const;  // Normal (cross-track) [m/s]

    // Get maneuver time
    double getManeuverTime() const;  // Seconds from now

    bool isManeuverEnabled() const { return maneuver_enabled_; }

Q_SIGNALS:
    void deltaVChanged(double dvr, double dvt, double dvn);
    void applyManeuverRequested();
    void clearManeuverRequested();

private Q_SLOTS:
    void onDeltaVChanged();
    void onApplyClicked();
    void onClearClicked();
    void onPresetSelected(int index);

private:
    void setupUi();

    bool maneuver_enabled_;

    // Delta-V inputs (RTN frame)
    QDoubleSpinBox* dv_r_spin_;
    QDoubleSpinBox* dv_t_spin_;
    QDoubleSpinBox* dv_n_spin_;

    // Magnitude display
    QLabel* dv_magnitude_label_;

    // Time input
    QDoubleSpinBox* maneuver_time_spin_;

    // Preset maneuvers
    QComboBox* preset_combo_;

    // Control buttons
    QPushButton* apply_button_;
    QPushButton* clear_button_;

    // Status
    QLabel* status_label_;
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

    void setStartEnabled(bool enabled);

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
    bool start_enabled_;

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
    void onDataSourceChanged(DataSourceType type);
    void onConfigurationComplete();
    void onFetchTleRequested(uint32_t norad_id);
    void onAddSatelliteRequested();
    void onSatelliteSelected(const QString& name);
    void onRemoveSatelliteRequested(const QString& name);
    void onStartPropagation();
    void onStopPropagation();
    void onResetPropagation();
    void onDeltaVChanged(double dvr, double dvt, double dvn);
    void onApplyManeuver();
    void onTimerUpdate();

    // Robot arm control slots
    void onArmHomeClicked();
    void onArmReadyClicked();
    void onArmCaptureClicked();
    void onDockActivateClicked();
    void onDockDeactivateClicked();

private:
    void setupUi();
    void setupRobotArmTab();
    void connectSignals();
    void setupRosConnections();
    void updateStatusDisplay();
    void publishSpawnRequest();
    void publishArmTrajectory(const std::vector<double>& joint_positions_deg);

    // UI Components
    QTabWidget* tab_widget_;
    DataSourceWidget* data_source_widget_;
    SatelliteTableWidget* satellite_table_;
    PropagationControlWidget* propagation_control_;
    DeltaVWidget* delta_v_widget_;
    QWidget* analysis_tab_;

    // Robot arm control tab
    QWidget* arm_control_tab_;
    QSlider* joint_sliders_[6];
    QLabel* joint_value_labels_[6];
    QPushButton* arm_home_button_;
    QPushButton* arm_ready_button_;
    QPushButton* arm_capture_button_;
    QPushButton* dock_activate_button_;
    QPushButton* dock_deactivate_button_;
    QLabel* arm_status_label_;
    QLabel* dock_status_label_;

    // Status display
    QLabel* status_label_;
    QLabel* sim_time_label_;
    QLabel* satellite_count_label_;
    QLabel* data_source_label_;

    // ROS2 node and connections
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<hpop_msgs::msg::SatelliteState>::SharedPtr state_sub_;
    rclcpp::Client<hpop_msgs::srv::FetchTLE>::SharedPtr fetch_tle_client_;
    rclcpp::Client<hpop_msgs::srv::AddSatellite>::SharedPtr add_sat_client_;
    rclcpp::Client<hpop_msgs::srv::PlanManeuver>::SharedPtr maneuver_client_;
    rclcpp::Publisher<hpop_msgs::msg::ManeuverPlan>::SharedPtr delta_v_pub_;

    // Robot arm ROS2 connections
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_trajectory_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr dock_activate_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr dock_deactivate_client_;

    // Update timer
    QTimer* update_timer_;

    // State
    double simulation_time_;
    int satellite_count_;
    bool is_connected_;
    DataSourceType current_data_source_;
    QString selected_satellite_;
};

} // namespace hpop_rviz_plugins

#endif // HPOP_RVIZ_PLUGINS_HPOP_PANEL_HPP
