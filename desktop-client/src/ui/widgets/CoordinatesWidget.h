#ifndef COORDINATESWIDGET_H
#define COORDINATESWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QGroupBox>

/**
 * @brief Widget for displaying robot position coordinates (x, y)
 */
class CoordinatesWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit CoordinatesWidget(QWidget *parent = nullptr);
    ~CoordinatesWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Coordinates"; }

private slots:
    void onCoordinatesReceived(double x, double y);

private:
    void setupUI();

    QLabel* m_xLabel;
    QLabel* m_yLabel;
    QLabel* m_xValueLabel;
    QLabel* m_yValueLabel;
    
    double m_currentX;
    double m_currentY;
};

#endif // COORDINATESWIDGET_H
