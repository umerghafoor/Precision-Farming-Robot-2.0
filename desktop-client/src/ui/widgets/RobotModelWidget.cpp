#include "RobotModelWidget.h"
#include "OBJLoader.h"
#include "DigitalTwin.h"
#include "TwinState.h"
#include "Logger.h"

#ifdef HAVE_QT_OPENGL
#include "ModelGLView.h"
#endif

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDir>
#include <QCoreApplication>
#include <QtMath>

// ─── Configuration ────────────────────────────────────────────────────────────

// robot.obj has no separate wheel sub-objects; leave empty so no mesh tinting
// is applied and wheel rotation skips gracefully.
static const QStringList WHEEL_OBJECT_NAMES = {};
static const QStringList WHEEL_LABELS = { "FL", "FR", "RL" };

// Wheel radius (m) for angular velocity → angle conversion
static constexpr float WHEEL_RADIUS = 0.10f;

// ─── Status colour helpers ────────────────────────────────────────────────────

static QVector3D batteryColor(double level)
{
    // red → yellow → green
    float t = float(qBound(0.0, level, 100.0) / 100.0);
    if (t < 0.5f)
        return QVector3D(1.0f, t * 2.0f, 0.0f);          // red → yellow
    return     QVector3D(1.0f - (t - 0.5f) * 2.0f, 1.0f, 0.0f);  // yellow → green
}

static QVector3D wheelStatusColor(const QString &robotStatus)
{
    if (robotStatus.contains("Error",   Qt::CaseInsensitive)) return {1.0f, 0.15f, 0.15f};
    if (robotStatus.contains("Warning", Qt::CaseInsensitive)) return {1.0f, 0.75f, 0.0f};
    return {0.15f, 0.85f, 0.25f};  // green = ok
}

static QString colorToCSS(const QVector3D &c)
{
    return QString("color: rgb(%1,%2,%3);")
        .arg(int(c.x() * 255)).arg(int(c.y() * 255)).arg(int(c.z() * 255));
}

// ─── OBJ search path ─────────────────────────────────────────────────────────

static QString findModelFile()
{
    const QString name = QStringLiteral("robot.obj");
    const QStringList candidates = {
        QCoreApplication::applicationDirPath() + "/" + name,
        QCoreApplication::applicationDirPath() + "/../" + name,
        QCoreApplication::applicationDirPath() + "/../../" + name,
        QDir::currentPath() + "/" + name
    };
    for (const QString &p : candidates) {
        if (QFile::exists(p)) return QDir::cleanPath(p);
    }
    return {};
}

// ─── Constructor / Destructor ─────────────────────────────────────────────────

RobotModelWidget::RobotModelWidget(QWidget *parent)
    : BaseWidget(parent)
{
    setupUI();
    m_wheelTimer.start();
}

RobotModelWidget::~RobotModelWidget()
{
    if (m_loader) {
        m_loader->requestInterruption();
        m_loader->wait(3000);
    }
}

// ─── BaseWidget interface ─────────────────────────────────────────────────────

bool RobotModelWidget::initialize()
{
    if (m_digitalTwin) {
        connect(m_digitalTwin, &DigitalTwin::stateChanged,
                this, &RobotModelWidget::onTwinStateChanged);
    }

    startLoading();
    return true;
}

// ─── UI setup ────────────────────────────────────────────────────────────────

void RobotModelWidget::setupUI()
{
    QVBoxLayout *root = new QVBoxLayout(this);
    root->setContentsMargins(4, 4, 4, 4);
    root->setSpacing(4);

    // ── Stacked pages ────────────────────────────────────────────────────
    m_stack = new QStackedWidget(this);

    // Page 0: loading
    {
        QWidget *loadPage = new QWidget;
        QVBoxLayout *l = new QVBoxLayout(loadPage);
        l->setAlignment(Qt::AlignCenter);

        m_loadLabel = new QLabel("Loading model…");
        m_loadLabel->setAlignment(Qt::AlignCenter);
        m_loadLabel->setStyleSheet("QLabel { font-size: 13px; color: #aaa; }");

        m_progressBar = new QProgressBar;
        m_progressBar->setRange(0, 100);
        m_progressBar->setValue(0);
        m_progressBar->setFixedHeight(10);
        m_progressBar->setTextVisible(false);
        m_progressBar->setStyleSheet(
            "QProgressBar { border: none; border-radius: 4px; background: #2a2a2a; }"
            "QProgressBar::chunk { background: #4CAF50; border-radius: 4px; }");

        l->addWidget(m_loadLabel);
        l->addWidget(m_progressBar);
        m_stack->addWidget(loadPage);  // index 0
    }

    // Page 1: 3D view
    {
        QWidget *viewPage = new QWidget;
        QVBoxLayout *l = new QVBoxLayout(viewPage);
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(2);

        // toolbar row
        QHBoxLayout *toolbar = new QHBoxLayout;
        QLabel *title = new QLabel("3D Model View");
        title->setStyleSheet("QLabel { font-weight: bold; font-size: 11px; color: #ccc; }");
        m_resetBtn = new QPushButton("Reset Camera");
        m_resetBtn->setFixedHeight(22);
        m_resetBtn->setStyleSheet(
            "QPushButton { background:#2a2a2a; color:#bbb; border:1px solid #444;"
            " border-radius:3px; padding:0 8px; font-size:11px; }"
            "QPushButton:hover { background:#3a3a3a; }");
        connect(m_resetBtn, &QPushButton::clicked, this, &RobotModelWidget::onResetCamera);
        toolbar->addWidget(title);
        toolbar->addStretch();
        toolbar->addWidget(m_resetBtn);
        l->addLayout(toolbar);

#ifdef HAVE_QT_OPENGL
        m_glView = new ModelGLView(viewPage);
        l->addWidget(m_glView, 1);
#else
        QLabel *noGL = new QLabel("OpenGL not available");
        noGL->setAlignment(Qt::AlignCenter);
        l->addWidget(noGL, 1);
#endif

        m_stack->addWidget(viewPage);  // index 1
    }

    // Page 2: unsupported / error
    {
        m_unsupported = new QLabel("3D rendering unavailable");
        m_unsupported->setAlignment(Qt::AlignCenter);
        m_unsupported->setStyleSheet("QLabel { color: #888; font-size: 13px; }");
        m_stack->addWidget(m_unsupported);  // index 2
    }

    m_stack->setCurrentIndex(0);
    root->addWidget(m_stack, 1);

    // ── Status bar ────────────────────────────────────────────────────────
    QWidget     *statusBar    = new QWidget;
    QHBoxLayout *statusLayout = new QHBoxLayout(statusBar);
    statusLayout->setContentsMargins(4, 2, 4, 2);
    statusLayout->setSpacing(8);
    statusBar->setStyleSheet("background: #1a1a1c; border-radius: 3px;");

    // Battery
    QLabel *batIcon = new QLabel("⚡");
    batIcon->setStyleSheet("color:#aaa; font-size:11px;");
    statusLayout->addWidget(batIcon);

    m_batteryBar = new QProgressBar;
    m_batteryBar->setRange(0, 100);
    m_batteryBar->setValue(100);
    m_batteryBar->setFixedSize(60, 10);
    m_batteryBar->setTextVisible(false);
    m_batteryBar->setStyleSheet(
        "QProgressBar { border:none; border-radius:3px; background:#333; }"
        "QProgressBar::chunk { background:#4CAF50; border-radius:3px; }");
    statusLayout->addWidget(m_batteryBar);

    m_batteryLabel = new QLabel("100%");
    m_batteryLabel->setStyleSheet("QLabel { font-size:11px; color:#aaa; min-width:34px; }");
    statusLayout->addWidget(m_batteryLabel);

    statusLayout->addStretch();

    // Wheel dots
    QLabel *wIcon = new QLabel("●●● Wheels:");
    wIcon->setStyleSheet("color:#888; font-size:10px;");
    statusLayout->addWidget(wIcon);

    for (int i = 0; i < 3; ++i) {
        m_wheelDots[i] = new QLabel(WHEEL_LABELS[i]);
        m_wheelDots[i]->setStyleSheet("color:#15d93f; font-size:11px; font-weight:bold;");
        statusLayout->addWidget(m_wheelDots[i]);
    }

    root->addWidget(statusBar);
}

// ─── Loading ──────────────────────────────────────────────────────────────────

void RobotModelWidget::startLoading()
{
    QString path = findModelFile();
    if (path.isEmpty()) {
        m_loadLabel->setText("Model file not found (robot.obj)");
        Logger::instance().warning("RobotModelWidget: robot.obj not found");
        m_stack->setCurrentIndex(2);
        return;
    }

#ifndef HAVE_QT_OPENGL
    m_unsupported->setText("OpenGL 3.3 not available");
    m_stack->setCurrentIndex(2);
    return;
#endif

    m_stack->setCurrentIndex(0);
    m_loadLabel->setText(QString("Loading %1…").arg(QFileInfo(path).fileName()));

    m_loader = new OBJLoader(path, this);
    connect(m_loader, &OBJLoader::progress,     this, &RobotModelWidget::onLoadProgress);
    connect(m_loader, &OBJLoader::loadFinished, this, &RobotModelWidget::onLoadFinished);
    connect(m_loader, &OBJLoader::loadFailed,   this, &RobotModelWidget::onLoadFailed);
    m_loader->start();
}

// ─── Loader slots ─────────────────────────────────────────────────────────────

void RobotModelWidget::onLoadProgress(int percent)
{
    m_progressBar->setValue(percent);
    m_loadLabel->setText(QString("Loading model… %1%").arg(percent));
}

void RobotModelWidget::onLoadFinished(QVector<OBJMeshData> meshes)
{
#ifdef HAVE_QT_OPENGL
    m_glView->loadMeshes(meshes);
    showGLView();

    // Apply initial wheel color (green = healthy)
    for (const QString &name : WHEEL_OBJECT_NAMES)
        m_glView->setColorOverride(name, {0.15f, 0.85f, 0.25f});
#else
    Q_UNUSED(meshes)
    m_stack->setCurrentIndex(2);
#endif
}

void RobotModelWidget::onLoadFailed(const QString &reason)
{
    m_unsupported->setText("Load failed: " + reason);
    m_stack->setCurrentIndex(2);
    Logger::instance().error("RobotModelWidget: " + reason);
}

void RobotModelWidget::showGLView()
{
    m_stack->setCurrentIndex(1);
}

// ─── Twin state slot ──────────────────────────────────────────────────────────

void RobotModelWidget::onTwinStateChanged()
{
#ifdef HAVE_QT_OPENGL
    if (!m_digitalTwin || !m_digitalTwin->state()) return;
    if (m_stack->currentIndex() != 1) return;  // model not loaded yet

    TwinState *state = m_digitalTwin->state();

    // ── Battery ───────────────────────────────────────────────────────────
    double batLevel = state->batteryLevel();
    updateBatteryDisplay(batLevel);

    // ── Robot pose (moves the model on the ground plane) ─────────────────
    m_glView->setRobotPose(state->pose().position, state->pose().orientation);

    // ── Wheel rotation (integrate angular velocity) ───────────────────────
    float dt = float(m_wheelTimer.restart()) / 1000.0f;
    dt = qBound(0.0f, dt, 0.1f);  // clamp; avoid large jump on first call

    float linearVel  = state->velocity().linear.x();   // m/s forward
    float angularVel = linearVel / WHEEL_RADIUS;        // rad/s
    m_wheelAngle += qRadiansToDegrees(angularVel * dt);

    for (const QString &name : WHEEL_OBJECT_NAMES)
        m_glView->setWheelAngle(name, m_wheelAngle);

    // ── Wheel status colour ───────────────────────────────────────────────
    updateWheelStatus(state->robotStatus());

    // setRobotPose already calls update(); no extra call needed
#endif
}

// ─── Status display helpers ───────────────────────────────────────────────────

void RobotModelWidget::updateBatteryDisplay(double level)
{
    int pct = int(qBound(0.0, level, 100.0));
    m_batteryBar->setValue(pct);
    m_batteryLabel->setText(QString("%1%").arg(pct));

    // Recolor bar: green > 50%, yellow 20–50%, red < 20%
    QString chunkColor = pct > 50 ? "#4CAF50" : (pct > 20 ? "#FFC107" : "#F44336");
    m_batteryBar->setStyleSheet(
        "QProgressBar { border:none; border-radius:3px; background:#333; }"
        "QProgressBar::chunk { background:" + chunkColor + "; border-radius:3px; }");

#ifdef HAVE_QT_OPENGL
    // No battery mesh in placeholder model – status shown in bar only
    Q_UNUSED(batteryColor(level));
#endif
}

void RobotModelWidget::updateWheelStatus(const QString &robotStatus)
{
    QVector3D color3d = wheelStatusColor(robotStatus);
    QString   css     = colorToCSS(color3d);

    for (int i = 0; i < 3; ++i) {
        m_wheelDots[i]->setStyleSheet("font-size:11px; font-weight:bold; " + css);
#ifdef HAVE_QT_OPENGL
        if (i < WHEEL_OBJECT_NAMES.size())
            m_glView->setColorOverride(WHEEL_OBJECT_NAMES[i], color3d);
#endif
    }
}

void RobotModelWidget::onResetCamera()
{
#ifdef HAVE_QT_OPENGL
    if (m_glView) m_glView->resetCamera();
#endif
}
