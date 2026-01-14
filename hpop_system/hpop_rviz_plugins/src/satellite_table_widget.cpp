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

    // Info label
    auto* info_label = new QLabel("Add satellites using the Data Source tab");
    info_label->setStyleSheet("color: gray; font-style: italic;");
    layout->addWidget(info_label);

    // Table
    table_ = new QTableWidget(0, 5);
    table_->setHorizontalHeaderLabels({"Name", "NORAD ID", "Alt (km)", "Vel (km/s)", "Period (min)"});
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    layout->addWidget(table_);

    // Button row
    auto* button_layout = new QHBoxLayout();

    remove_button_ = new QPushButton("Remove Selected");
    remove_button_->setEnabled(false);
    button_layout->addWidget(remove_button_);

    clear_all_button_ = new QPushButton("Clear All");
    clear_all_button_->setEnabled(false);
    button_layout->addWidget(clear_all_button_);

    button_layout->addStretch();
    layout->addLayout(button_layout);

    // Connect signals
    connect(remove_button_, &QPushButton::clicked, this, &SatelliteTableWidget::onRemoveButtonClicked);
    connect(clear_all_button_, &QPushButton::clicked, this, &SatelliteTableWidget::onClearAllClicked);
    connect(table_, &QTableWidget::itemSelectionChanged, this, &SatelliteTableWidget::onTableSelectionChanged);

    // No automatic satellite addition - user must add via Data Source tab
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

    clear_all_button_->setEnabled(true);
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

    // If satellite not found, add it
    addSatellite(name, 0, altitude_km, 0.0);
}

void SatelliteTableWidget::removeSatellite(const std::string& name)
{
    for (int row = 0; row < table_->rowCount(); ++row)
    {
        if (table_->item(row, 0)->text().toStdString() == name)
        {
            table_->removeRow(row);
            break;
        }
    }

    if (table_->rowCount() == 0)
    {
        clear_all_button_->setEnabled(false);
    }
}

void SatelliteTableWidget::clearAll()
{
    table_->setRowCount(0);
    clear_all_button_->setEnabled(false);
    remove_button_->setEnabled(false);
}

int SatelliteTableWidget::getSatelliteCount() const
{
    return table_->rowCount();
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

void SatelliteTableWidget::onClearAllClicked()
{
    int ret = QMessageBox::question(this, "Clear All",
        "Are you sure you want to remove all satellites?",
        QMessageBox::Yes | QMessageBox::No);

    if (ret == QMessageBox::Yes)
    {
        clearAll();
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
