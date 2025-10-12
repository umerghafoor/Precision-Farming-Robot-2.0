#include "BaseWidget.h"
#include "Logger.h"
#include <QCloseEvent>

BaseWidget::BaseWidget(QWidget *parent)
    : QWidget(parent)
    , m_ros2Interface(nullptr)
    , m_digitalTwin(nullptr)
{
    setMinimumSize(300, 200);
}

BaseWidget::~BaseWidget()
{
    Logger::instance().debug(QString("Widget destroyed: %1").arg(m_widgetId));
}

void BaseWidget::closeEvent(QCloseEvent *event)
{
    emit widgetClosed(m_widgetId);
    event->accept();
}
