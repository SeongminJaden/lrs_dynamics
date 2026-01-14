/**
 * @file data_source_widget.cpp
 * @brief Data source selection widget implementation
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QMessageBox>

namespace hpop_rviz_plugins
{

DataSourceWidget::DataSourceWidget(QWidget* parent)
    : QWidget(parent)
    , selected_source_(DataSourceType::NONE)
    , is_configured_(false)
{
    setupUi();
}

void DataSourceWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(5, 5, 5, 5);

    // Source type selection group
    auto* source_group_box = new QGroupBox("Select Data Source");
    auto* source_layout = new QHBoxLayout(source_group_box);

    source_group_ = new QButtonGroup(this);

    tle_radio_ = new QRadioButton("TLE");
    tle_radio_->setToolTip("Two-Line Element set from Celestrak");
    source_group_->addButton(tle_radio_, 0);
    source_layout->addWidget(tle_radio_);

    observation_radio_ = new QRadioButton("Orbital Elements");
    observation_radio_->setToolTip("Manual input of Keplerian elements");
    source_group_->addButton(observation_radio_, 1);
    source_layout->addWidget(observation_radio_);

    sensor_radio_ = new QRadioButton("Sensor");
    sensor_radio_->setToolTip("Real-time data from sensors/Gazebo");
    source_group_->addButton(sensor_radio_, 2);
    source_layout->addWidget(sensor_radio_);

    source_layout->addStretch();
    layout->addWidget(source_group_box);

    // Stacked widget for source-specific inputs
    source_stack_ = new QStackedWidget();

    // ========== TLE Page ==========
    tle_page_ = new QWidget();
    auto* tle_layout = new QVBoxLayout(tle_page_);

    auto* norad_layout = new QHBoxLayout();
    norad_layout->addWidget(new QLabel("NORAD ID:"));
    norad_input_ = new QLineEdit();
    norad_input_->setPlaceholderText("e.g., 25544 (ISS)");
    norad_input_->setMaximumWidth(150);
    norad_layout->addWidget(norad_input_);
    fetch_tle_button_ = new QPushButton("Fetch TLE");
    norad_layout->addWidget(fetch_tle_button_);
    norad_layout->addStretch();
    tle_layout->addLayout(norad_layout);

    tle_layout->addWidget(new QLabel("Or enter TLE directly:"));
    tle_line1_input_ = new QLineEdit();
    tle_line1_input_->setPlaceholderText("Line 1: 1 25544U 98067A ...");
    tle_layout->addWidget(tle_line1_input_);
    tle_line2_input_ = new QLineEdit();
    tle_line2_input_->setPlaceholderText("Line 2: 2 25544 51.6400 ...");
    tle_layout->addWidget(tle_line2_input_);

    tle_status_label_ = new QLabel("");
    tle_status_label_->setStyleSheet("color: gray;");
    tle_layout->addWidget(tle_status_label_);
    tle_layout->addStretch();

    source_stack_->addWidget(tle_page_);

    // ========== Observation (Manual Elements) Page ==========
    observation_page_ = new QWidget();
    auto* obs_layout = new QGridLayout(observation_page_);

    int row = 0;
    obs_layout->addWidget(new QLabel("Semi-major Axis [km]:"), row, 0);
    sma_spin_ = new QDoubleSpinBox();
    sma_spin_->setRange(6378, 50000);
    sma_spin_->setValue(6778);  // ~400km altitude
    sma_spin_->setDecimals(1);
    obs_layout->addWidget(sma_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("Eccentricity:"), row, 0);
    ecc_spin_ = new QDoubleSpinBox();
    ecc_spin_->setRange(0, 0.99);
    ecc_spin_->setValue(0.001);
    ecc_spin_->setDecimals(6);
    ecc_spin_->setSingleStep(0.001);
    obs_layout->addWidget(ecc_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("Inclination [deg]:"), row, 0);
    inc_spin_ = new QDoubleSpinBox();
    inc_spin_->setRange(0, 180);
    inc_spin_->setValue(51.6);
    inc_spin_->setDecimals(2);
    obs_layout->addWidget(inc_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("RAAN [deg]:"), row, 0);
    raan_spin_ = new QDoubleSpinBox();
    raan_spin_->setRange(0, 360);
    raan_spin_->setValue(0);
    raan_spin_->setDecimals(2);
    obs_layout->addWidget(raan_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("Arg of Periapsis [deg]:"), row, 0);
    aop_spin_ = new QDoubleSpinBox();
    aop_spin_->setRange(0, 360);
    aop_spin_->setValue(0);
    aop_spin_->setDecimals(2);
    obs_layout->addWidget(aop_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("True Anomaly [deg]:"), row, 0);
    ta_spin_ = new QDoubleSpinBox();
    ta_spin_->setRange(0, 360);
    ta_spin_->setValue(0);
    ta_spin_->setDecimals(2);
    obs_layout->addWidget(ta_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("Mass [kg]:"), row, 0);
    mass_spin_ = new QDoubleSpinBox();
    mass_spin_->setRange(1, 1000000);
    mass_spin_->setValue(1000);
    mass_spin_->setDecimals(1);
    obs_layout->addWidget(mass_spin_, row++, 1);

    obs_layout->addWidget(new QLabel("Cross Section [m^2]:"), row, 0);
    cs_spin_ = new QDoubleSpinBox();
    cs_spin_->setRange(0.01, 1000);
    cs_spin_->setValue(10);
    cs_spin_->setDecimals(2);
    obs_layout->addWidget(cs_spin_, row++, 1);

    obs_layout->setRowStretch(row, 1);
    source_stack_->addWidget(observation_page_);

    // ========== Sensor Page ==========
    sensor_page_ = new QWidget();
    auto* sensor_layout = new QVBoxLayout(sensor_page_);

    sensor_layout->addWidget(new QLabel("Sensor Topic:"));
    sensor_topic_combo_ = new QComboBox();
    sensor_topic_combo_->addItem("/gazebo/model_states");
    sensor_topic_combo_->addItem("/hpop/sensor_state");
    sensor_topic_combo_->addItem("/gps/fix");
    sensor_topic_combo_->setEditable(true);
    sensor_layout->addWidget(sensor_topic_combo_);

    sensor_status_label_ = new QLabel("Status: Not connected");
    sensor_status_label_->setStyleSheet("color: orange;");
    sensor_layout->addWidget(sensor_status_label_);
    sensor_layout->addStretch();

    source_stack_->addWidget(sensor_page_);

    // Add placeholder for no selection
    auto* no_selection_page = new QWidget();
    auto* no_sel_layout = new QVBoxLayout(no_selection_page);
    auto* no_sel_label = new QLabel("Select a data source above to configure satellite parameters");
    no_sel_label->setAlignment(Qt::AlignCenter);
    no_sel_label->setStyleSheet("color: gray; font-style: italic;");
    no_sel_layout->addWidget(no_sel_label);
    source_stack_->insertWidget(0, no_selection_page);
    source_stack_->setCurrentIndex(0);

    layout->addWidget(source_stack_);

    // Apply button
    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();
    apply_button_ = new QPushButton("Add Satellite & Start HPOP");
    apply_button_->setEnabled(false);
    apply_button_->setStyleSheet("font-weight: bold;");
    button_layout->addWidget(apply_button_);
    layout->addLayout(button_layout);

    // Connect signals
    connect(source_group_, QOverload<int>::of(&QButtonGroup::buttonClicked),
            this, &DataSourceWidget::onSourceTypeChanged);
    connect(fetch_tle_button_, &QPushButton::clicked, this, &DataSourceWidget::onFetchTleClicked);
    connect(apply_button_, &QPushButton::clicked, this, &DataSourceWidget::onApplyClicked);
}

void DataSourceWidget::onSourceTypeChanged()
{
    int id = source_group_->checkedId();

    switch (id)
    {
        case 0:
            selected_source_ = DataSourceType::TLE;
            source_stack_->setCurrentIndex(1);  // TLE page
            break;
        case 1:
            selected_source_ = DataSourceType::OBSERVATION;
            source_stack_->setCurrentIndex(2);  // Observation page
            break;
        case 2:
            selected_source_ = DataSourceType::SENSOR;
            source_stack_->setCurrentIndex(3);  // Sensor page
            break;
        default:
            selected_source_ = DataSourceType::NONE;
            source_stack_->setCurrentIndex(0);
            break;
    }

    emit dataSourceChanged(selected_source_);
    updateConfigurationState();
}

void DataSourceWidget::onFetchTleClicked()
{
    QString text = norad_input_->text().trimmed();
    if (text.isEmpty())
    {
        QMessageBox::warning(this, "Input Error", "Please enter a NORAD ID.");
        return;
    }

    bool ok;
    uint32_t norad_id = text.toUInt(&ok);
    if (!ok)
    {
        QMessageBox::warning(this, "Input Error", "Invalid NORAD ID format.");
        return;
    }

    tle_status_label_->setText("Fetching TLE...");
    tle_status_label_->setStyleSheet("color: blue;");
    emit fetchTleRequested(norad_id);
}

void DataSourceWidget::onApplyClicked()
{
    updateConfigurationState();

    if (is_configured_)
    {
        emit addSatelliteRequested();
        emit configurationComplete();
    }
    else
    {
        QMessageBox::warning(this, "Configuration Incomplete",
            "Please complete the configuration for the selected data source.");
    }
}

void DataSourceWidget::updateConfigurationState()
{
    is_configured_ = false;

    switch (selected_source_)
    {
        case DataSourceType::TLE:
            // Need either NORAD ID or both TLE lines
            if (!norad_input_->text().trimmed().isEmpty() ||
                (!tle_line1_input_->text().trimmed().isEmpty() &&
                 !tle_line2_input_->text().trimmed().isEmpty()))
            {
                is_configured_ = true;
            }
            break;

        case DataSourceType::OBSERVATION:
            // Always configured with default values
            is_configured_ = true;
            break;

        case DataSourceType::SENSOR:
            // Always configured if a topic is selected
            is_configured_ = !sensor_topic_combo_->currentText().isEmpty();
            break;

        default:
            is_configured_ = false;
            break;
    }

    apply_button_->setEnabled(is_configured_);
}

uint32_t DataSourceWidget::getNoradId() const
{
    return norad_input_->text().toUInt();
}

QString DataSourceWidget::getTleLine1() const
{
    return tle_line1_input_->text();
}

QString DataSourceWidget::getTleLine2() const
{
    return tle_line2_input_->text();
}

double DataSourceWidget::getSemiMajorAxis() const
{
    return sma_spin_->value() * 1000.0;  // km to m
}

double DataSourceWidget::getEccentricity() const
{
    return ecc_spin_->value();
}

double DataSourceWidget::getInclination() const
{
    return inc_spin_->value() * M_PI / 180.0;  // deg to rad
}

double DataSourceWidget::getRaan() const
{
    return raan_spin_->value() * M_PI / 180.0;
}

double DataSourceWidget::getArgPeriapsis() const
{
    return aop_spin_->value() * M_PI / 180.0;
}

double DataSourceWidget::getTrueAnomaly() const
{
    return ta_spin_->value() * M_PI / 180.0;
}

double DataSourceWidget::getMass() const
{
    return mass_spin_->value();
}

double DataSourceWidget::getCrossSection() const
{
    return cs_spin_->value();
}

} // namespace hpop_rviz_plugins
