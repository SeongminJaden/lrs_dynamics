/**
 * @file satellite_table_widget.cpp
 * @brief Satellite table widget implementation
 */

#include "hpop_rviz_plugins/hpop_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>

namespace hpop_rviz_plugins
{

SatelliteTableWidget::SatelliteTableWidget(QWidget* parent)
    : QWidget(parent)
{
    setupUi();
}

void SatelliteTableWidget::setupUi()
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // Table
    table_ = new QTableWidget(0, 5);
    table_->setHorizontalHeaderLabels({"Name", "NORAD ID", "Alt (km)", "Vel (km/s)", "Period (min)"});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    layout->addWidget(table_);

    // Input row
    auto* input_layout = new QHBoxLayout();

    auto* norad_label = new QLabel("NORAD ID:");
    input_layout->addWidget(norad_label);

    norad_input_ = new QLineEdit();
    norad_input_->setPlaceholderText("e.g., 25544 (ISS)");
    norad_input_->setMaximumWidth(150);
    input_layout->addWidget(norad_input_);

    fetch_tle_button_ = new QPushButton("Fetch TLE");
    fetch_tle_button_->setMaximumWidth(80);
    input_layout->addWidget(fetch_tle_button_);

    add_button_ = new QPushButton("Add");
    add_button_->setMaximumWidth(60);
    input_layout->addWidget(add_button_);

    remove_button_ = new QPushButton("Remove");
    remove_button_->setMaximumWidth(70);
    remove_button_->setEnabled(false);
    input_layout->addWidget(remove_button_);

    input_layout->addStretch();
    layout->addLayout(input_layout);

    // Connect signals
    connect(add_button_, &QPushButton::clicked, this, &SatelliteTableWidget::onAddButtonClicked);
    connect(remove_button_, &QPushButton::clicked, this, &SatelliteTableWidget::onRemoveButtonClicked);
    connect(fetch_tle_button_, &QPushButton::clicked, this, &SatelliteTableWidget::onAddButtonClicked);
    connect(table_, &QTableWidget::itemSelectionChanged, this, &SatelliteTableWidget::onTableSelectionChanged);

    // Add some example satellites
    addSatellite("ISS (ZARYA)", 25544, 420.0, 51.64);
    addSatellite("STARLINK-1234", 48274, 550.0, 53.0);
    addSatellite("SENTINEL-6A", 46984, 1336.0, 66.0);
}

void SatelliteTableWidget::addSatellite(const std::string& name, uint32_t norad_id,
                                         double altitude_km, double inclination_deg)
{
    int row = table_->rowCount();
    table_->insertRow(row);

    table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(name)));
    table_->setItem(row, 1, new QTableWidgetItem(QString::number(norad_id)));
    table_->setItem(row, 2, new QTableWidgetItem(QString::number(altitude_km, 'f', 1)));
    table_->setItem(row, 3, new QTableWidgetItem("--"));
    table_->setItem(row, 4, new QTableWidgetItem(QString::number(inclination_deg, 'f', 1)));
}

void SatelliteTableWidget::updateSatellite(const std::string& name, double altitude_km,
                                            double velocity_kms, double period_min)
{
    for (int row = 0; row < table_->rowCount(); ++row)
    {
        if (table_->item(row, 0)->text().toStdString() == name)
        {
            table_->item(row, 2)->setText(QString::number(altitude_km, 'f', 1));
            table_->item(row, 3)->setText(QString::number(velocity_kms, 'f', 3));
            table_->item(row, 4)->setText(QString::number(period_min, 'f', 1));
            return;
        }
    }
}

void SatelliteTableWidget::removeSatellite(const std::string& name)
{
    for (int row = 0; row < table_->rowCount(); ++row)
    {
        if (table_->item(row, 0)->text().toStdString() == name)
        {
            table_->removeRow(row);
            return;
        }
    }
}

void SatelliteTableWidget::clearAll()
{
    table_->setRowCount(0);
}

void SatelliteTableWidget::onAddButtonClicked()
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

    emit addSatelliteRequested(norad_id);
    norad_input_->clear();
}

void SatelliteTableWidget::onRemoveButtonClicked()
{
    auto selected = table_->selectedItems();
    if (!selected.isEmpty())
    {
        QString name = table_->item(selected[0]->row(), 0)->text();
        emit removeSatelliteRequested(name);
    }
}

void SatelliteTableWidget::onTableSelectionChanged()
{
    auto selected = table_->selectedItems();
    remove_button_->setEnabled(!selected.isEmpty());

    if (!selected.isEmpty())
    {
        QString name = table_->item(selected[0]->row(), 0)->text();
        emit satelliteSelected(name);
    }
}

} // namespace hpop_rviz_plugins
