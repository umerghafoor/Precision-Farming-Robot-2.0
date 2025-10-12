#ifndef TWINVISUALIZATIONWIDGET_H
#define TWINVISUALIZATIONWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QTextEdit>
#include <QPushButton>
#include <QComboBox>

/**
 * @brief Widget for visualizing the digital twin
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
    void updateTwinDisplay();

    QLabel* m_modeLabel;
    QTextEdit* m_stateDisplay;
    QComboBox* m_modeSelector;
    QPushButton* m_resetButton;
};

#endif // TWINVISUALIZATIONWIDGET_H
