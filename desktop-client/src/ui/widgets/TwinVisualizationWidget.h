#ifndef TWINVISUALIZATIONWIDGET_H
#define TWINVISUALIZATIONWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QPushButton>
#include <QComboBox>

class RobotMapView;

/**
 * @brief Widget for visualizing the digital twin as a 2D overhead map.
 */
class TwinVisualizationWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit TwinVisualizationWidget(QWidget *parent = nullptr);
    ~TwinVisualizationWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Digital Twin"; }

private slots:
    void onTwinStateChanged();
    void onToggleMode();
    void onResetTwin();

private:
    void setupUI();
    void syncModeCombo();

    QLabel*       m_modeLabel;
    QLabel*       m_coordLabel;
    QComboBox*    m_modeSelector;
    QPushButton*  m_resetButton;
    RobotMapView* m_mapView;
};

#endif // TWINVISUALIZATIONWIDGET_H
