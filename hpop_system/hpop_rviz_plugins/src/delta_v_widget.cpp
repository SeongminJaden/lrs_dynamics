/**
 * @file delta_v_widget.cpp
 * @brief Delta-V maneuver control widget implementation
 *
 * Real-time Delta-V input in RTN (Radial-Transverse-Normal) frame
 * Modifiable during HPOP propagation for real-time orbit changes
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <cmath>

namespace hpop_rviz_plugins
{

DeltaVWidget::DeltaVWidget(QWidget* parent)
    : QWidget(parent)
    , maneuver_enabled_(false)
{
    setupUi();
}

void DeltaVWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(5, 5, 5, 5);

    // Delta-V input group (RTN frame)
    auto* dv_group = new QGroupBox("Delta-V Input (RTN Frame)");
    auto* dv_layout = new QGridLayout(dv_group);

    // Radial (R) - perpendicular to orbit plane, outward
    dv_layout->addWidget(new QLabel("Radial (R):"), 0, 0);
    dv_r_spin_ = new QDoubleSpinBox();
    dv_r_spin_->setRange(-1000, 1000);
    dv_r_spin_->setValue(0);
    dv_r_spin_->setDecimals(3);
    dv_r_spin_->setSingleStep(0.1);
    dv_r_spin_->setSuffix(" m/s");
    dv_r_spin_->setToolTip("Radial: perpendicular to orbit, positive outward from Earth");
    dv_layout->addWidget(dv_r_spin_, 0, 1);

    // Transverse (T) - along-track, direction of velocity
    dv_layout->addWidget(new QLabel("Transverse (T):"), 1, 0);
    dv_t_spin_ = new QDoubleSpinBox();
    dv_t_spin_->setRange(-1000, 1000);
    dv_t_spin_->setValue(0);
    dv_t_spin_->setDecimals(3);
    dv_t_spin_->setSingleStep(0.1);
    dv_t_spin_->setSuffix(" m/s");
    dv_t_spin_->setToolTip("Transverse: along-track, positive in velocity direction (prograde)");
    dv_layout->addWidget(dv_t_spin_, 1, 1);

    // Normal (N) - cross-track, perpendicular to orbit plane
    dv_layout->addWidget(new QLabel("Normal (N):"), 2, 0);
    dv_n_spin_ = new QDoubleSpinBox();
    dv_n_spin_->setRange(-1000, 1000);
    dv_n_spin_->setValue(0);
    dv_n_spin_->setDecimals(3);
    dv_n_spin_->setSingleStep(0.1);
    dv_n_spin_->setSuffix(" m/s");
    dv_n_spin_->setToolTip("Normal: cross-track, perpendicular to orbit plane (changes inclination)");
    dv_layout->addWidget(dv_n_spin_, 2, 1);

    // Magnitude display
    dv_layout->addWidget(new QLabel("Magnitude:"), 3, 0);
    dv_magnitude_label_ = new QLabel("0.000 m/s");
    dv_magnitude_label_->setStyleSheet("font-weight: bold;");
    dv_layout->addWidget(dv_magnitude_label_, 3, 1);

    layout->addWidget(dv_group);

    // Maneuver time group
    auto* time_group = new QGroupBox("Maneuver Timing");
    auto* time_layout = new QHBoxLayout(time_group);

    time_layout->addWidget(new QLabel("Execute in:"));
    maneuver_time_spin_ = new QDoubleSpinBox();
    maneuver_time_spin_->setRange(0, 86400);  // 0 to 24 hours
    maneuver_time_spin_->setValue(0);
    maneuver_time_spin_->setDecimals(1);
    maneuver_time_spin_->setSuffix(" sec");
    maneuver_time_spin_->setToolTip("Time until maneuver execution (0 = immediate)");
    time_layout->addWidget(maneuver_time_spin_);
    time_layout->addStretch();

    layout->addWidget(time_group);

    // Preset maneuvers
    auto* preset_group = new QGroupBox("Preset Maneuvers");
    auto* preset_layout = new QHBoxLayout(preset_group);

    preset_layout->addWidget(new QLabel("Quick Select:"));
    preset_combo_ = new QComboBox();
    preset_combo_->addItem("-- Select Preset --");
    preset_combo_->addItem("Hohmann Raise (+10 m/s T)");
    preset_combo_->addItem("Hohmann Lower (-10 m/s T)");
    preset_combo_->addItem("Plane Change (+5 m/s N)");
    preset_combo_->addItem("Radial Impulse (+5 m/s R)");
    preset_combo_->addItem("Circularize (-5 m/s T)");
    preset_combo_->addItem("Clear All");
    preset_layout->addWidget(preset_combo_);
    preset_layout->addStretch();

    layout->addWidget(preset_group);

    // Control buttons
    auto* button_layout = new QHBoxLayout();

    apply_button_ = new QPushButton("Apply Delta-V");
    apply_button_->setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white;");
    apply_button_->setToolTip("Apply the Delta-V to the selected satellite");
    button_layout->addWidget(apply_button_);

    clear_button_ = new QPushButton("Clear");
    clear_button_->setToolTip("Reset all Delta-V values to zero");
    button_layout->addWidget(clear_button_);

    button_layout->addStretch();
    layout->addLayout(button_layout);

    // Status label
    status_label_ = new QLabel("Enter Delta-V values and click Apply");
    status_label_->setStyleSheet("color: gray; font-style: italic;");
    layout->addWidget(status_label_);

    layout->addStretch();

    // Connect signals
    connect(dv_r_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &DeltaVWidget::onDeltaVChanged);
    connect(dv_t_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &DeltaVWidget::onDeltaVChanged);
    connect(dv_n_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &DeltaVWidget::onDeltaVChanged);
    connect(apply_button_, &QPushButton::clicked, this, &DeltaVWidget::onApplyClicked);
    connect(clear_button_, &QPushButton::clicked, this, &DeltaVWidget::onClearClicked);
    connect(preset_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DeltaVWidget::onPresetSelected);
}

void DeltaVWidget::onDeltaVChanged()
{
    double r = dv_r_spin_->value();
    double t = dv_t_spin_->value();
    double n = dv_n_spin_->value();

    // Calculate magnitude
    double magnitude = std::sqrt(r*r + t*t + n*n);
    dv_magnitude_label_->setText(QString::number(magnitude, 'f', 3) + " m/s");

    // Update status
    if (magnitude > 0)
    {
        status_label_->setText("Delta-V configured - click Apply to execute");
        status_label_->setStyleSheet("color: blue;");
        maneuver_enabled_ = true;
    }
    else
    {
        status_label_->setText("Enter Delta-V values and click Apply");
        status_label_->setStyleSheet("color: gray; font-style: italic;");
        maneuver_enabled_ = false;
    }

    // Emit signal for real-time updates
    emit deltaVChanged(r, t, n);
}

void DeltaVWidget::onApplyClicked()
{
    double r = dv_r_spin_->value();
    double t = dv_t_spin_->value();
    double n = dv_n_spin_->value();
    double magnitude = std::sqrt(r*r + t*t + n*n);

    if (magnitude < 0.001)
    {
        QMessageBox::information(this, "No Delta-V",
            "Please enter non-zero Delta-V values before applying.");
        return;
    }

    status_label_->setText(QString("Applied: ΔV = %1 m/s").arg(magnitude, 0, 'f', 3));
    status_label_->setStyleSheet("color: green; font-weight: bold;");

    emit applyManeuverRequested();
}

void DeltaVWidget::onClearClicked()
{
    dv_r_spin_->setValue(0);
    dv_t_spin_->setValue(0);
    dv_n_spin_->setValue(0);
    maneuver_time_spin_->setValue(0);
    preset_combo_->setCurrentIndex(0);

    status_label_->setText("Delta-V cleared");
    status_label_->setStyleSheet("color: gray; font-style: italic;");
    maneuver_enabled_ = false;

    emit clearManeuverRequested();
}

void DeltaVWidget::onPresetSelected(int index)
{
    switch (index)
    {
        case 1:  // Hohmann Raise
            dv_r_spin_->setValue(0);
            dv_t_spin_->setValue(10);
            dv_n_spin_->setValue(0);
            break;
        case 2:  // Hohmann Lower
            dv_r_spin_->setValue(0);
            dv_t_spin_->setValue(-10);
            dv_n_spin_->setValue(0);
            break;
        case 3:  // Plane Change
            dv_r_spin_->setValue(0);
            dv_t_spin_->setValue(0);
            dv_n_spin_->setValue(5);
            break;
        case 4:  // Radial Impulse
            dv_r_spin_->setValue(5);
            dv_t_spin_->setValue(0);
            dv_n_spin_->setValue(0);
            break;
        case 5:  // Circularize
            dv_r_spin_->setValue(0);
            dv_t_spin_->setValue(-5);
            dv_n_spin_->setValue(0);
            break;
        case 6:  // Clear All
            onClearClicked();
            break;
        default:
            break;
    }

    // Reset combo to first item after applying preset
    if (index > 0 && index < 6)
    {
        preset_combo_->blockSignals(true);
        preset_combo_->setCurrentIndex(0);
        preset_combo_->blockSignals(false);
    }
}

double DeltaVWidget::getDeltaVR() const
{
    return dv_r_spin_->value();
}

double DeltaVWidget::getDeltaVT() const
{
    return dv_t_spin_->value();
}

double DeltaVWidget::getDeltaVN() const
{
    return dv_n_spin_->value();
}

double DeltaVWidget::getManeuverTime() const
{
    return maneuver_time_spin_->value();
}

} // namespace hpop_rviz_plugins
