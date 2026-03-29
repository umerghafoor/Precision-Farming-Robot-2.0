#include "MaterialDockWidget.h"
#include <QStyle>

MaterialDockWidget::MaterialDockWidget(const QString& title, QWidget* parent)
    : QDockWidget(title, parent)
{
    buildTitleBar(title);

    // Update floating dynamic property so QSS [floating="true"] selector works
    connect(this, &QDockWidget::topLevelChanged, this, [this](bool floating) {
        setProperty("floating", floating);
        style()->unpolish(this);
        style()->polish(this);
    });
}

void MaterialDockWidget::buildTitleBar(const QString& title)
{
    m_titleBar = new QWidget(this);
    m_titleBar->setObjectName("dockTitleBar");
    m_titleBar->setMinimumHeight(32);
    m_titleBar->setMaximumHeight(32);

    auto* layout = new QHBoxLayout(m_titleBar);
    layout->setContentsMargins(12, 0, 8, 0);
    layout->setSpacing(8);

    // Drag grip — decorative only, passes mouse events through to title bar
    auto* grip = new QLabel("\u28FF", m_titleBar);
    grip->setObjectName("dockGripLabel");
    grip->setAttribute(Qt::WA_TransparentForMouseEvents);
    layout->addWidget(grip);

    m_titleLabel = new QLabel(title, m_titleBar);
    m_titleLabel->setObjectName("dockTitleLabel");
    layout->addWidget(m_titleLabel, 1);

    m_floatButton = new QToolButton(m_titleBar);
    m_floatButton->setObjectName("dockFloatButton");
    m_floatButton->setText("\u2750");  // ❐ float/restore icon
    m_floatButton->setToolTip(tr("Float / Dock"));
    m_floatButton->setAutoRaise(true);
    connect(m_floatButton, &QToolButton::clicked, this, &MaterialDockWidget::onFloatClicked);
    layout->addWidget(m_floatButton);

    m_closeButton = new QToolButton(m_titleBar);
    m_closeButton->setObjectName("dockCloseButton");
    m_closeButton->setText("\u2715");  // ✕
    m_closeButton->setToolTip(tr("Close"));
    m_closeButton->setAutoRaise(true);
    connect(m_closeButton, &QToolButton::clicked, this, &MaterialDockWidget::onCloseClicked);
    layout->addWidget(m_closeButton);

    setTitleBarWidget(m_titleBar);
}

void MaterialDockWidget::setCardTitle(const QString& title)
{
    setWindowTitle(title);
    if (m_titleLabel)
        m_titleLabel->setText(title);
}

void MaterialDockWidget::onFloatClicked()
{
    setFloating(!isFloating());
}

void MaterialDockWidget::onCloseClicked()
{
    close();
}
