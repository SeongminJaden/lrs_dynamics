/**
 * @file propagation_control_widget.cpp
 * @brief Propagation control widget implementation
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSlider>

namespace hpop_rviz_plugins
{

PropagationControlWidget::PropagationControlWidget(QWidget* parent)
    : QWidget(parent)
    , is_propagating_(false)
    , start_enabled_(false)
{
    setupUi();
}

void PropagationControlWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // Control buttons
    auto* button_layout = new QHBoxLayout();

    start_stop_button_ = new QPushButton("Start HPOP");
    start_stop_button_->setMinimumWidth(100);
    start_stop_button_->setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white;");
    start_stop_button_->setEnabled(false);  // Disabled until satellite is configured
    start_stop_button_->setToolTip("Configure a satellite in Data Source tab first");
    button_layout->addWidget(start_stop_button_);

    reset_button_ = new QPushButton("Reset");
    reset_button_->setMinimumWidth(80);
    button_layout->addWidget(reset_button_);

    button_layout->addStretch();
    layout->addLayout(button_layout);

    // Time scale slider
    auto* time_group = new QGroupBox("Time Scale");
    auto* time_layout = new QHBoxLayout(time_group);

    time_scale_slider_ = new QSlider(Qt::Horizontal);
    time_scale_slider_->setMinimum(1);
    time_scale_slider_->setMaximum(1000);
    time_scale_slider_->setValue(100);
    time_scale_slider_->setTickPosition(QSlider::TicksBelow);
    time_scale_slider_->setTickInterval(100);
    time_layout->addWidget(time_scale_slider_);

    time_scale_label_ = new QLabel("100x");
    time_scale_label_->setMinimumWidth(60);
    time_layout->addWidget(time_scale_label_);

    layout->addWidget(time_group);

    // Integrator settings
    auto* integrator_group = new QGroupBox("Integrator Settings");
    auto* integrator_layout = new QGridLayout(integrator_group);

    integrator_layout->addWidget(new QLabel("Method:"), 0, 0);
    integrator_combo_ = new QComboBox();
    integrator_combo_->addItems({"RK4", "RKF78", "Adams-Bashforth"});
    integrator_combo_->setCurrentIndex(1);  // RKF78 default
    integrator_layout->addWidget(integrator_combo_, 0, 1);

    integrator_layout->addWidget(new QLabel("Step Size (s):"), 1, 0);
    step_size_spin_ = new QDoubleSpinBox();
    step_size_spin_->setRange(1.0, 300.0);
    step_size_spin_->setValue(60.0);
    step_size_spin_->setDecimals(1);
    integrator_layout->addWidget(step_size_spin_, 1, 1);

    integrator_layout->addWidget(new QLabel("Gravity Degree:"), 2, 0);
    gravity_degree_spin_ = new QSpinBox();
    gravity_degree_spin_->setRange(2, 360);
    gravity_degree_spin_->setValue(70);
    integrator_layout->addWidget(gravity_degree_spin_, 2, 1);

    layout->addWidget(integrator_group);

    // Perturbation toggles
    auto* pert_group = new QGroupBox("Perturbations");
    auto* pert_layout = new QGridLayout(pert_group);

    j2_check_ = new QCheckBox("J2 Gravity");
    j2_check_->setChecked(true);
    pert_layout->addWidget(j2_check_, 0, 0);

    drag_check_ = new QCheckBox("Atmospheric Drag");
    drag_check_->setChecked(true);
    pert_layout->addWidget(drag_check_, 0, 1);

    srp_check_ = new QCheckBox("Solar Radiation Pressure");
    srp_check_->setChecked(true);
    pert_layout->addWidget(srp_check_, 1, 0);

    third_body_check_ = new QCheckBox("Third-Body (Sun/Moon)");
    third_body_check_->setChecked(true);
    pert_layout->addWidget(third_body_check_, 1, 1);

    layout->addWidget(pert_group);

    layout->addStretch();

    // Connect signals
    connect(start_stop_button_, &QPushButton::clicked, this, &PropagationControlWidget::onStartStopClicked);
    connect(reset_button_, &QPushButton::clicked, this, &PropagationControlWidget::onResetClicked);
    connect(time_scale_slider_, &QSlider::valueChanged, this, &PropagationControlWidget::onTimeScaleChanged);

    connect(j2_check_, &QCheckBox::stateChanged, this, &PropagationControlWidget::perturbationsChanged);
    connect(drag_check_, &QCheckBox::stateChanged, this, &PropagationControlWidget::perturbationsChanged);
    connect(srp_check_, &QCheckBox::stateChanged, this, &PropagationControlWidget::perturbationsChanged);
    connect(third_body_check_, &QCheckBox::stateChanged, this, &PropagationControlWidget::perturbationsChanged);
}

double PropagationControlWidget::getTimeScale() const
{
    return static_cast<double>(time_scale_slider_->value());
}

double PropagationControlWidget::getStepSize() const
{
    return step_size_spin_->value();
}

void PropagationControlWidget::onStartStopClicked()
{
    is_propagating_ = !is_propagating_;
    updateButtonState();

    if (is_propagating_)
    {
        emit startPropagation();
    }
    else
    {
        emit stopPropagation();
    }
}

void PropagationControlWidget::onResetClicked()
{
    is_propagating_ = false;
    updateButtonState();
    emit resetPropagation();
}

void PropagationControlWidget::onTimeScaleChanged(int value)
{
    time_scale_label_->setText(QString("%1x").arg(value));
    emit timeScaleChanged(static_cast<double>(value));
}

void PropagationControlWidget::updateButtonState()
{
    if (is_propagating_)
    {
        start_stop_button_->setText("Stop HPOP");
        start_stop_button_->setStyleSheet("font-weight: bold; background-color: #f44336; color: white;");
    }
    else
    {
        start_stop_button_->setText("Start HPOP");
        start_stop_button_->setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white;");
    }
}

void PropagationControlWidget::setStartEnabled(bool enabled)
{
    start_enabled_ = enabled;
    start_stop_button_->setEnabled(enabled);

    if (enabled)
    {
        start_stop_button_->setToolTip("Start orbit propagation");
    }
    else
    {
        start_stop_button_->setToolTip("Configure a satellite in Data Source tab first");
        if (is_propagating_)
        {
            is_propagating_ = false;
            updateButtonState();
            emit stopPropagation();
        }
    }
}

} // namespace hpop_rviz_plugins
