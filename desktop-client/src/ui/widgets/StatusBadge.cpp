#include "StatusBadge.h"
#include <QStyleOption>
#include <QPainter>

StatusBadge::StatusBadge(const QString &text, QWidget *parent)
    : QWidget(parent)
    , m_dot(new QFrame(this))
    , m_label(new QLabel(this))
{
    setObjectName("statusBadge");

    m_dot->setFixedSize(8, 8);
    m_dot->setStyleSheet("border-radius:4px; background-color: #7A9B79;"); // default grey

    m_label->setText(text);
    m_label->setStyleSheet("color: #E4F0E3; font-size:10px;");

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(4, 2, 4, 2);
    layout->setSpacing(4);
    layout->addWidget(m_dot);
    layout->addWidget(m_label);

    // keep the widget's background transparent so menu bar inherits
    setAttribute(Qt::WA_TranslucentBackground);
}

void StatusBadge::setText(const QString &text)
{
    m_label->setText(text);
}

void StatusBadge::setDotColor(const QColor &color)
{
    QString style = QString("border-radius:4px; background-color: %1;").arg(color.name());
    m_dot->setStyleSheet(style);
}
