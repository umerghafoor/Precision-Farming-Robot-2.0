#include "MaterialDockWidget.h"
#include <QStyle>
#include <QTimer>
#include <QEvent>

// Title bar subclass that ignores all mouse events so they propagate up to the
// parent QDockWidget, which is required for Qt's built-in drag state machine to
// initialize correctly (QTBUG-43698). Without this, a plain QWidget accepts the
// mouse press, QDockWidget never sees it, the drag state is never created, and
// subsequent move events crash inside QDockWidget::event() via a null-state access.
class DockTitleBar final : public QWidget
{
public:
    explicit DockTitleBar(QWidget* parent = nullptr) : QWidget(parent) {}
protected:
    void mousePressEvent(QMouseEvent* e) override       { e->ignore(); }
    void mouseMoveEvent(QMouseEvent* e) override        { e->ignore(); }
    void mouseReleaseEvent(QMouseEvent* e) override     { e->ignore(); }
    void mouseDoubleClickEvent(QMouseEvent* e) override { e->ignore(); }
};

MaterialDockWidget::MaterialDockWidget(const QString& title, QWidget* parent)
    : QDockWidget(title, parent)
{
    buildTitleBar(title);

    // Update floating dynamic property so QSS [floating="true"] selector works.
    // Style polishing is deferred via QTimer to avoid interfering with Qt's
    // internal dock drag state machine, which can cause a crash if layout/style
    // changes are triggered synchronously during a drag operation.
    connect(this, &QDockWidget::topLevelChanged, this, [this](bool floating) {
        setProperty("floating", floating);
        QTimer::singleShot(0, this, [this]() {
            style()->unpolish(this);
            style()->polish(this);
        });
        updateFullscreenButton();
    });
}

void MaterialDockWidget::buildTitleBar(const QString& title)
{
    m_titleBar = new DockTitleBar(this);
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
    m_titleLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    layout->addWidget(m_titleLabel, 1);

    m_fullscreenButton = new QToolButton(m_titleBar);
    m_fullscreenButton->setObjectName("dockFullscreenButton");
    m_fullscreenButton->setText("\u26F6");  // ⛶ fullscreen icon
    m_fullscreenButton->setToolTip(tr("Full Screen"));
    m_fullscreenButton->setAutoRaise(true);
    connect(m_fullscreenButton, &QToolButton::clicked, this, &MaterialDockWidget::onFullscreenClicked);
    layout->addWidget(m_fullscreenButton);

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

void MaterialDockWidget::onFullscreenClicked()
{
    if (windowState() & Qt::WindowFullScreen) {
        showNormal();
        // Restore to docked if it wasn't floating before going fullscreen
        if (!m_wasFloating)
            setFloating(false);
    } else {
        m_wasFloating = isFloating();
        setFloating(true);
        showFullScreen();
    }
}

void MaterialDockWidget::onFloatClicked()
{
    setFloating(!isFloating());
}

void MaterialDockWidget::onCloseClicked()
{
    close();
}

void MaterialDockWidget::updateFullscreenButton()
{
    if (!m_fullscreenButton)
        return;

    if (windowState() & Qt::WindowFullScreen) {
        m_fullscreenButton->setText("\u2716");  // ✖ — exit fullscreen
        m_fullscreenButton->setToolTip(tr("Exit Full Screen  (Esc)"));
    } else {
        m_fullscreenButton->setText("\u26F6");  // ⛶ — enter fullscreen
        m_fullscreenButton->setToolTip(tr("Full Screen"));
    }
}

void MaterialDockWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Escape && (windowState() & Qt::WindowFullScreen)) {
        onFullscreenClicked();
        event->accept();
        return;
    }
    QDockWidget::keyPressEvent(event);
}

void MaterialDockWidget::changeEvent(QEvent* event)
{
    QDockWidget::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange)
        updateFullscreenButton();
}
