#include "LaserCalibrationWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>
#include <QSettings>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSpinBox>
#include <cmath>

// ── CalibrationCanvas ─────────────────────────────────────────────────────────

CalibrationCanvas::CalibrationCanvas(QWidget* parent) : QWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    m_corners.fill({-1, -1});
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(320, 240);
}

void CalibrationCanvas::setFrame(const QImage& img) {
    m_frame = img;
    update();
}

void CalibrationCanvas::setActiveCorner(int idx) {
    m_activeCorner = idx;
    update();
}

void CalibrationCanvas::setCornerPixel(int idx, QPointF px) {
    if (idx >= 0 && idx < 4) {
        m_corners[idx] = px;
        update();
    }
}

void CalibrationCanvas::setClickMode(bool enabled) {
    m_clickMode = enabled;
    setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
}

QRectF CalibrationCanvas::imageRect() const {
    if (m_frame.isNull()) return rect();
    const double scale = std::min(
        static_cast<double>(width())  / m_frame.width(),
        static_cast<double>(height()) / m_frame.height());
    const double w = m_frame.width()  * scale;
    const double h = m_frame.height() * scale;
    return {(width() - w) / 2, (height() - h) / 2, w, h};
}

QPointF CalibrationCanvas::imageToWidget(QPointF imgPt) const {
    const QRectF r = imageRect();
    if (m_frame.isNull()) return imgPt;
    const double sx = r.width()  / m_frame.width();
    const double sy = r.height() / m_frame.height();
    return {r.left() + imgPt.x() * sx, r.top() + imgPt.y() * sy};
}

void CalibrationCanvas::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(20, 20, 20));

    // Draw camera frame
    if (!m_frame.isNull()) {
        const QRectF r = imageRect();
        p.drawImage(r, m_frame);
    }

    // Define ideal corner positions in normalised image coords
    const QPointF idealNorm[4] = {{0.1, 0.1}, {0.9, 0.1}, {0.9, 0.9}, {0.1, 0.9}};
    const QColor  cornerColors[4] = {
        QColor(255, 80,  80),   // 1 = top-left     red
        QColor(80,  200, 80),   // 2 = top-right    green
        QColor(80,  120, 255),  // 3 = bottom-right blue
        QColor(255, 200, 40),   // 4 = bottom-left  yellow
    };
    const QRectF r = imageRect();

    for (int i = 0; i < 4; ++i) {
        // Compute widget position for this corner marker
        QPointF wPt;
        if (!m_frame.isNull() && m_corners[i].x() >= 0) {
            // Already placed — show where the user calibrated
            wPt = imageToWidget(m_corners[i]);
        } else {
            // Show suggested position
            wPt = {r.left() + idealNorm[i].x() * r.width(),
                   r.top()  + idealNorm[i].y() * r.height()};
        }

        const bool active = (i == m_activeCorner);
        const bool placed = m_corners[i].x() >= 0;

        // Outer ring
        p.setPen(QPen(cornerColors[i], active ? 3 : 2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(wPt, active ? 18.0 : 14.0, active ? 18.0 : 14.0);

        // Inner dot
        if (placed) {
            p.setBrush(cornerColors[i]);
            p.setPen(Qt::NoPen);
            p.drawEllipse(wPt, 5.0, 5.0);
        } else {
            // Cross-hair
            p.setPen(QPen(cornerColors[i], 1.5));
            p.drawLine(wPt + QPointF(-8, 0), wPt + QPointF(8, 0));
            p.drawLine(wPt + QPointF(0, -8), wPt + QPointF(0, 8));
        }

        // Label
        p.setPen(cornerColors[i]);
        QFont f = p.font();
        f.setBold(true);
        f.setPixelSize(14);
        p.setFont(f);
        p.drawText(wPt + QPointF(14, -14), QString::number(i + 1));
    }

    // Active-corner highlight overlay
    if (m_activeCorner >= 0) {
        p.setPen(QPen(cornerColors[m_activeCorner], 2, Qt::DashLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(r.adjusted(2, 2, -2, -2));
    }
}

void CalibrationCanvas::mousePressEvent(QMouseEvent* e) {
    if (!m_clickMode) return;
    const QRectF r = imageRect();
    if (!r.contains(e->pos())) return;
    const double nx = (e->pos().x() - r.left()) / r.width();
    const double ny = (e->pos().y() - r.top())  / r.height();
    emit clicked({nx, ny});
}

// ── LaserCalibrationWidget ────────────────────────────────────────────────────

LaserCalibrationWidget::LaserCalibrationWidget(QWidget* parent)
    : BaseWidget(parent)
{
    setupUI();
    setFocusPolicy(Qt::StrongFocus);
}

void LaserCalibrationWidget::setupUI() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(6);

    // ── Canvas ────────────────────────────────────────────────────────────
    m_canvas = new CalibrationCanvas(this);
    connect(m_canvas, &CalibrationCanvas::clicked,
            this, &LaserCalibrationWidget::onCanvasClicked);
    root->addWidget(m_canvas, 1);

    // ── Status ────────────────────────────────────────────────────────────
    m_statusLabel = new QLabel("Select a corner to begin calibration", this);
    m_statusLabel->setAlignment(Qt::AlignCenter);
    m_statusLabel->setStyleSheet("color:#90CAF9;font-weight:bold;padding:4px;");
    root->addWidget(m_statusLabel);

    // Servo readout + step size
    auto* servoRow = new QHBoxLayout;
    m_servo1Label = new QLabel("Servo1: 90°", this);
    m_servo2Label = new QLabel("Servo2: 90°", this);
    for (auto* l : {m_servo1Label, m_servo2Label}) {
        l->setAlignment(Qt::AlignCenter);
        l->setStyleSheet("color:#B0BEC5;font-family:monospace;");
        servoRow->addWidget(l);
    }

    auto* stepLabel = new QLabel("Step:", this);
    stepLabel->setStyleSheet("color:#90A4AE;");
    m_stepSpin = new QSpinBox(this);
    m_stepSpin->setRange(1, 20);
    m_stepSpin->setValue(m_step);
    m_stepSpin->setSuffix("°");
    m_stepSpin->setFixedWidth(70);
    m_stepSpin->setStyleSheet("QSpinBox{background:#263238;color:#CFD8DC;border:1px solid #455A64;border-radius:3px;padding:2px;}");
    connect(m_stepSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int v){ m_step = v; });
    servoRow->addWidget(stepLabel);
    servoRow->addWidget(m_stepSpin);
    root->addLayout(servoRow);

    // ── Corner buttons ────────────────────────────────────────────────────
    auto* cornersBox = new QGroupBox("Calibration corners", this);
    auto* cGrid = new QGridLayout(cornersBox);
    const char* labels[4] = {"1 Top-Left", "2 Top-Right", "3 Bottom-Right", "4 Bottom-Left"};
    const char* colors[4] = {"#EF5350","#66BB6A","#42A5F5","#FFA726"};
    for (int i = 0; i < 4; ++i) {
        m_cornerBtns[i] = new QPushButton(labels[i], this);
        m_cornerBtns[i]->setCheckable(true);
        m_cornerBtns[i]->setStyleSheet(
            QString("QPushButton{border:2px solid %1;border-radius:4px;padding:4px 8px;}"
                    "QPushButton:checked{background:%1;color:#000;}").arg(colors[i]));
        connect(m_cornerBtns[i], &QPushButton::clicked, this, [this, i]{ onStartCorner(i); });
        cGrid->addWidget(m_cornerBtns[i], i / 2, i % 2);
    }
    root->addWidget(cornersBox);

    // ── Action buttons ────────────────────────────────────────────────────
    auto* btnRow = new QHBoxLayout;
    m_confirmBtn = new QPushButton("✔ Confirm Corner", this);
    m_confirmBtn->setEnabled(false);
    m_confirmBtn->setStyleSheet("QPushButton{background:#388E3C;color:#fff;border-radius:4px;padding:5px 10px;}"
                                "QPushButton:disabled{background:#37474F;color:#607D8B;}");
    connect(m_confirmBtn, &QPushButton::clicked, this, &LaserCalibrationWidget::onConfirmCorner);

    m_saveBtn = new QPushButton("💾 Save Cal", this);
    m_saveBtn->setEnabled(false);
    m_saveBtn->setStyleSheet("QPushButton{background:#1565C0;color:#fff;border-radius:4px;padding:5px 10px;}"
                             "QPushButton:disabled{background:#37474F;color:#607D8B;}");
    connect(m_saveBtn, &QPushButton::clicked, this, &LaserCalibrationWidget::onSaveCalibration);

    m_clearBtn = new QPushButton("✖ Clear", this);
    m_clearBtn->setStyleSheet("QPushButton{background:#B71C1C;color:#fff;border-radius:4px;padding:5px 10px;}");
    connect(m_clearBtn, &QPushButton::clicked, this, &LaserCalibrationWidget::onClearCalibration);

    m_testBtn = new QPushButton("🎯 Test Mode", this);
    m_testBtn->setCheckable(true);
    m_testBtn->setEnabled(false);
    m_testBtn->setStyleSheet("QPushButton{border:2px solid #7B1FA2;border-radius:4px;padding:5px 10px;}"
                             "QPushButton:checked{background:#7B1FA2;color:#fff;}");
    connect(m_testBtn, &QPushButton::toggled, this, &LaserCalibrationWidget::onTestModeToggled);

    m_laserBtn = new QPushButton("🔴 Laser OFF", this);
    m_laserBtn->setCheckable(true);
    m_laserBtn->setStyleSheet("QPushButton{border:2px solid #F44336;border-radius:4px;padding:5px 10px;}"
                              "QPushButton:checked{background:#F44336;color:#fff;}");
    connect(m_laserBtn, &QPushButton::toggled, this, [this](bool on){
        m_laserOn = on;
        m_laserBtn->setText(on ? "🔴 Laser ON" : "🔴 Laser OFF");
        if (m_ros2Interface) m_ros2Interface->publishLaserCommand(on);
    });

    btnRow->addWidget(m_confirmBtn);
    btnRow->addWidget(m_saveBtn);
    btnRow->addWidget(m_clearBtn);
    btnRow->addWidget(m_testBtn);
    btnRow->addWidget(m_laserBtn);
    root->addLayout(btnRow);

    // ── Instructions ──────────────────────────────────────────────────────
    auto* hint = new QLabel(
        "📌 Click a corner button → use arrow keys to move laser → ✔ Confirm\n"
        "After all 4 corners: 🎯 Test Mode — click anywhere on the image to point laser there",
        this);
    hint->setWordWrap(true);
    hint->setAlignment(Qt::AlignCenter);
    hint->setStyleSheet("color:#78909C;font-size:11px;");
    root->addWidget(hint);

    // Load saved calibration
    loadCalibration();
}

bool LaserCalibrationWidget::initialize() {
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imageReceived,
                this, &LaserCalibrationWidget::onImageReceived);
        m_ros2Interface->subscribeCameraTopic("/camera/color_jpeg");
        Logger::instance().info("LaserCalibrationWidget initialized");
    }
    return true;
}

void LaserCalibrationWidget::onImageReceived(const QByteArray& data, int w, int h) {
    m_imgW = w; m_imgH = h;
    QImage img(reinterpret_cast<const uchar*>(data.constData()), w, h,
               w * 3, QImage::Format_RGB888);
    m_canvas->setFrame(img.copy());
}

void LaserCalibrationWidget::onStartCorner(int idx) {
    // Uncheck other buttons
    for (int i = 0; i < 4; ++i) m_cornerBtns[i]->setChecked(i == idx);

    m_activeCorner = idx;
    m_canvas->setActiveCorner(idx);
    m_confirmBtn->setEnabled(true);
    setFocus();

    // Restore servo angles to this corner's last saved values if set
    if (m_corners[idx].set) {
        m_servo1 = m_corners[idx].s1;
        m_servo2 = m_corners[idx].s2;
        if (m_ros2Interface) {
            m_ros2Interface->publishServoAngle(1, static_cast<int>(m_servo1));
            m_ros2Interface->publishServoAngle(2, static_cast<int>(m_servo2));
        }
    }
    updateStatus();
}

void LaserCalibrationWidget::onConfirmCorner() {
    if (m_activeCorner < 0) return;

    // Store current image centre pixel for this corner
    // We use the centre of the ideal target circle as the pixel —
    // the user has moved the laser to the target, so the target pixel IS the corner.
    const double idealNormX[4] = {0.1, 0.9, 0.9, 0.1};
    const double idealNormY[4] = {0.1, 0.1, 0.9, 0.9};
    const QPointF px{idealNormX[m_activeCorner] * m_imgW,
                     idealNormY[m_activeCorner] * m_imgH};

    m_corners[m_activeCorner].pixel = px;
    m_corners[m_activeCorner].s1    = m_servo1;
    m_corners[m_activeCorner].s2    = m_servo2;
    m_corners[m_activeCorner].set   = true;

    m_canvas->setCornerPixel(m_activeCorner, px);

    // Deactivate
    m_cornerBtns[m_activeCorner]->setChecked(false);
    m_activeCorner = -1;
    m_canvas->setActiveCorner(-1);
    m_confirmBtn->setEnabled(false);

    // Check if all 4 done
    m_calibrated = m_corners[0].set && m_corners[1].set &&
                   m_corners[2].set && m_corners[3].set;
    m_saveBtn->setEnabled(m_calibrated);
    m_testBtn->setEnabled(m_calibrated);

    updateStatus();
    Logger::instance().info(QString("Corner %1 confirmed: s1=%2 s2=%3")
                            .arg(m_activeCorner + 2)
                            .arg(m_servo1).arg(m_servo2));
}

void LaserCalibrationWidget::onSaveCalibration() {
    QSettings s;
    s.beginGroup("LaserCalibration");
    for (int i = 0; i < 4; ++i) {
        s.setValue(QString("c%1_px").arg(i), m_corners[i].pixel.x());
        s.setValue(QString("c%1_py").arg(i), m_corners[i].pixel.y());
        s.setValue(QString("c%1_s1").arg(i), m_corners[i].s1);
        s.setValue(QString("c%1_s2").arg(i), m_corners[i].s2);
        s.setValue(QString("c%1_set").arg(i), m_corners[i].set);
    }
    s.endGroup();
    m_statusLabel->setText("✅ Calibration saved!");
    Logger::instance().info("Laser calibration saved");
}

void LaserCalibrationWidget::loadCalibration() {
    QSettings s;
    s.beginGroup("LaserCalibration");
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        m_corners[i].set = s.value(QString("c%1_set").arg(i), false).toBool();
        if (m_corners[i].set) {
            m_corners[i].pixel = {s.value(QString("c%1_px").arg(i), 0.0).toDouble(),
                                  s.value(QString("c%1_py").arg(i), 0.0).toDouble()};
            m_corners[i].s1 = s.value(QString("c%1_s1").arg(i), 90.0).toDouble();
            m_corners[i].s2 = s.value(QString("c%1_s2").arg(i), 90.0).toDouble();
            m_canvas->setCornerPixel(i, m_corners[i].pixel);
            any = true;
        }
    }
    s.endGroup();
    if (any) {
        m_calibrated = m_corners[0].set && m_corners[1].set &&
                       m_corners[2].set && m_corners[3].set;
        m_saveBtn->setEnabled(m_calibrated);
        m_testBtn->setEnabled(m_calibrated);
        if (m_calibrated)
            m_statusLabel->setText("✅ Calibration loaded — all 4 corners set");
    }
}

void LaserCalibrationWidget::onClearCalibration() {
    for (auto& c : m_corners) c = {};
    m_calibrated = false;
    m_activeCorner = -1;
    m_canvas->setActiveCorner(-1);
    for (int i = 0; i < 4; ++i) {
        m_cornerBtns[i]->setChecked(false);
        m_canvas->setCornerPixel(i, {-1, -1});
    }
    m_saveBtn->setEnabled(false);
    m_testBtn->setEnabled(false);
    m_testBtn->setChecked(false);
    QSettings s;
    s.remove("LaserCalibration");
    updateStatus();
}

void LaserCalibrationWidget::onTestModeToggled(bool on) {
    m_testMode = on;
    m_canvas->setClickMode(on);
    m_canvas->setActiveCorner(-1);
    m_activeCorner = -1;
    m_confirmBtn->setEnabled(false);
    if (on) {
        m_statusLabel->setText("🎯 Test mode — click anywhere to point laser there");
        if (m_ros2Interface) m_ros2Interface->publishLaserCommand(true);
        m_laserBtn->setChecked(true);
    } else {
        updateStatus();
    }
}

void LaserCalibrationWidget::onCanvasClicked(QPointF norm) {
    if (!m_testMode || !m_calibrated) return;
    pointLaserAt(norm.x(), norm.y());
}

bool LaserCalibrationWidget::bilinearMap(double nx, double ny, double& s1, double& s2) const {
    if (!m_calibrated) return false;
    // corners: 0=TL 1=TR 2=BR 3=BL (normalised image space)
    // Bilinear blend: top = lerp(TL,TR,nx), bottom = lerp(BL,BR,nx), out = lerp(top,bottom,ny)
    const double topS1    = m_corners[0].s1 + nx * (m_corners[1].s1 - m_corners[0].s1);
    const double topS2    = m_corners[0].s2 + nx * (m_corners[1].s2 - m_corners[0].s2);
    const double botS1    = m_corners[3].s1 + nx * (m_corners[2].s1 - m_corners[3].s1);
    const double botS2    = m_corners[3].s2 + nx * (m_corners[2].s2 - m_corners[3].s2);
    s1 = topS1 + ny * (botS1 - topS1);
    s2 = topS2 + ny * (botS2 - topS2);
    s1 = std::max(0.0, std::min(180.0, s1));
    s2 = std::max(0.0, std::min(180.0, s2));
    return true;
}

void LaserCalibrationWidget::pointLaserAt(double normX, double normY) {
    double s1, s2;
    if (!bilinearMap(normX, normY, s1, s2)) return;
    m_servo1 = s1; m_servo2 = s2;
    if (m_ros2Interface) {
        m_ros2Interface->publishServoAngle(1, static_cast<int>(s1));
        m_ros2Interface->publishServoAngle(2, static_cast<int>(s2));
    }
    m_servo1Label->setText(QString("Servo1: %1°").arg(static_cast<int>(s1)));
    m_servo2Label->setText(QString("Servo2: %1°").arg(static_cast<int>(s2)));
    Logger::instance().debug(QString("Laser → norm(%.2f,%.2f) → s1=%1 s2=%2")
                             .arg(normX).arg(normY).arg(s1).arg(s2));
}

void LaserCalibrationWidget::keyPressEvent(QKeyEvent* e) {
    if (m_activeCorner < 0 && !m_testMode) {
        BaseWidget::keyPressEvent(e);
        return;
    }

    // Ignore auto-repeat so holding a key doesn't keep firing
    if (e->isAutoRepeat()) { e->accept(); return; }

    int d1 = 0, d2 = 0;
    switch (e->key()) {
        case Qt::Key_Left:  d1 = -1; break;
        case Qt::Key_Right: d1 = +1; break;
        case Qt::Key_Up:    d2 = -1; break;
        case Qt::Key_Down:  d2 = +1; break;
        default: BaseWidget::keyPressEvent(e); return;
    }

    m_servo1 = std::max(0.0, std::min(180.0, m_servo1 + d1 * m_step));
    m_servo2 = std::max(0.0, std::min(180.0, m_servo2 + d2 * m_step));
    if (m_ros2Interface) {
        if (d1) m_ros2Interface->publishServoAngle(1, static_cast<int>(m_servo1));
        if (d2) m_ros2Interface->publishServoAngle(2, static_cast<int>(m_servo2));
    }
    m_servo1Label->setText(QString("Servo1: %1°").arg(static_cast<int>(m_servo1)));
    m_servo2Label->setText(QString("Servo2: %1°").arg(static_cast<int>(m_servo2)));
    e->accept();
}

void LaserCalibrationWidget::updateStatus() {
    if (m_calibrated) {
        m_statusLabel->setText("✅ All 4 corners calibrated — enable Test Mode to verify");
        return;
    }
    int done = 0;
    for (const auto& c : m_corners) if (c.set) done++;
    if (m_activeCorner >= 0) {
        m_statusLabel->setText(QString("📍 Calibrating corner %1 — use ← → ↑ ↓ to move laser, ✔ to confirm")
                               .arg(m_activeCorner + 1));
    } else {
        m_statusLabel->setText(QString("Select a corner to calibrate (%1/4 done)").arg(done));
    }
}
