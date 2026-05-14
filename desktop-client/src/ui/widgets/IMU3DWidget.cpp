#include "IMU3DWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QPainter>
#include <QPainterPath>
#include <cmath>
#include <algorithm>

// ── IMUCubeView ───────────────────────────────────────────────────────────────

IMUCubeView::IMUCubeView(QWidget* parent) : QWidget(parent) {
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(160, 160);
}

void IMUCubeView::setAccel(double ax, double ay, double az) {
    m_ax = ax; m_ay = ay; m_az = az;
    update();
}

void IMUCubeView::rotateY(Vec3& v, double rad) const {
    const double c = std::cos(rad), s = std::sin(rad);
    const double x = v.x * c + v.z * s;
    const double z = -v.x * s + v.z * c;
    v.x = x; v.z = z;
}

void IMUCubeView::rotateX(Vec3& v, double rad) const {
    const double c = std::cos(rad), s = std::sin(rad);
    const double y = v.y * c - v.z * s;
    const double z = v.y * s + v.z * c;
    v.y = y; v.z = z;
}

QPointF IMUCubeView::project(Vec3 v, double cx, double cy, double scale) const {
    // Simple perspective
    const double fov = 4.0;
    const double z = v.z + fov;
    if (z < 0.01) return {cx, cy};
    return {cx + v.x / z * scale, cy - v.y / z * scale};
}

void IMUCubeView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(18, 18, 28));

    const double cx = width()  / 2.0;
    const double cy = height() / 2.0;
    const double scale = std::min(width(), height()) * 0.35;

    // Derive pitch (rotation around X) and roll (rotation around Z) from accel
    // Normalise gravity vector
    const double mag = std::sqrt(m_ax*m_ax + m_ay*m_ay + m_az*m_az);
    const double gx = (mag > 0.01) ? m_ax / mag : 0;
    const double gy = (mag > 0.01) ? m_ay / mag : 0;
    const double gz = (mag > 0.01) ? m_az / mag : 1;

    const double pitch = std::atan2(gx, gz);          // rotation around Y
    const double roll  = std::atan2(-gy, gz);          // rotation around X
    const double yaw   = 0.4;                          // fixed decorative yaw

    // Unit cube vertices
    const Vec3 verts[8] = {
        {-1,-1,-1},{+1,-1,-1},{+1,+1,-1},{-1,+1,-1},
        {-1,-1,+1},{+1,-1,+1},{+1,+1,+1},{-1,+1,+1}
    };

    // Rotate and project
    QPointF pts[8];
    double  depths[8];
    for (int i = 0; i < 8; ++i) {
        Vec3 v = verts[i];
        rotateX(v, roll);
        rotateY(v, pitch);
        rotateY(v, yaw);
        depths[i] = v.z;
        pts[i] = project(v, cx, cy, scale);
    }

    // 6 faces: vertex indices and colours
    struct Face { int v[4]; QColor col; };
    const Face faces[6] = {
        {{0,1,2,3}, QColor(60,  80, 180, 180)},  // front
        {{4,5,6,7}, QColor(40,  60, 160, 180)},  // back
        {{0,4,7,3}, QColor(80, 100, 200, 180)},  // left
        {{1,5,6,2}, QColor(80, 100, 200, 180)},  // right
        {{3,2,6,7}, QColor(100,130, 220, 200)},  // top
        {{0,1,5,4}, QColor(40,  60, 140, 180)},  // bottom
    };

    // Painter's algorithm: sort faces by average depth
    int order[6] = {0,1,2,3,4,5};
    std::sort(order, order+6, [&](int a, int b){
        double da = 0, db = 0;
        for (int k=0;k<4;k++) { da+=depths[faces[a].v[k]]; db+=depths[faces[b].v[k]]; }
        return da < db;
    });

    for (int fi : order) {
        const auto& f = faces[fi];
        QPainterPath path;
        path.moveTo(pts[f.v[0]]);
        for (int k=1;k<4;k++) path.lineTo(pts[f.v[k]]);
        path.closeSubpath();
        p.fillPath(path, f.col);
        p.setPen(QPen(QColor(140,160,255,200), 1.5));
        p.drawPath(path);
    }

    // Axis labels
    auto drawAxis = [&](Vec3 dir, double len, const QString& lbl, QColor col) {
        Vec3 tip = {dir.x*len, dir.y*len, dir.z*len};
        rotateX(tip, roll); rotateY(tip, pitch); rotateY(tip, yaw);
        QPointF p2 = project(tip, cx, cy, scale);
        p.setPen(QPen(col, 2));
        p.drawLine(QPointF(cx, cy), p2);
        p.setPen(col);
        QFont f = p.font(); f.setBold(true); f.setPixelSize(11); p.setFont(f);
        p.drawText(p2 + QPointF(4, -4), lbl);
    };
    drawAxis({1,0,0}, 1.6, "X", QColor(255,80,80));
    drawAxis({0,1,0}, 1.6, "Y", QColor(80,220,80));
    drawAxis({0,0,1}, 1.6, "Z", QColor(80,140,255));

    // Tilt angles overlay
    p.setPen(QColor(180,200,255));
    QFont f2 = p.font(); f2.setPixelSize(10); f2.setBold(false); p.setFont(f2);
    p.drawText(6, 16, QString("Pitch: %1°").arg(static_cast<int>(pitch * 180 / M_PI)));
    p.drawText(6, 30, QString("Roll:  %1°").arg(static_cast<int>(roll  * 180 / M_PI)));
}

// ── LiveGraph ─────────────────────────────────────────────────────────────────

LiveGraph::LiveGraph(const QString& label, const QColor& color, QWidget* parent)
    : QWidget(parent), m_label(label), m_color(color)
{
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setMinimumHeight(50);
}

void LiveGraph::push(double value) {
    m_data.push_back(value);
    if (static_cast<int>(m_data.size()) > HISTORY) m_data.pop_front();
    // rolling min/max with 10% headroom
    if (!m_data.empty()) {
        m_min = *std::min_element(m_data.begin(), m_data.end());
        m_max = *std::max_element(m_data.begin(), m_data.end());
        const double range = m_max - m_min;
        if (range < 0.001) { m_min -= 0.5; m_max += 0.5; }
        else { m_min -= range * 0.1; m_max += range * 0.1; }
    }
    update();
}

void LiveGraph::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(20, 22, 32));

    const int W = width(), H = height();
    const int n = static_cast<int>(m_data.size());

    // Zero line
    if (m_min < 0 && m_max > 0) {
        const double zy = H - (0 - m_min) / (m_max - m_min) * H;
        p.setPen(QPen(QColor(60, 60, 80), 1, Qt::DashLine));
        p.drawLine(0, static_cast<int>(zy), W, static_cast<int>(zy));
    }

    // Line graph
    if (n > 1) {
        QPainterPath path;
        for (int i = 0; i < n; ++i) {
            const double x = static_cast<double>(i) / (HISTORY - 1) * W;
            const double y = H - (m_data[i] - m_min) / (m_max - m_min) * H;
            i == 0 ? path.moveTo(x, y) : path.lineTo(x, y);
        }
        p.setPen(QPen(m_color, 1.5));
        p.drawPath(path);
    }

    // Label + latest value
    p.setPen(m_color);
    QFont f = p.font(); f.setBold(true); f.setPixelSize(10); p.setFont(f);
    const QString val = n ? QString("%1: %2").arg(m_label)
                                .arg(m_data.back(), 0, 'f', 2) : m_label;
    p.drawText(4, H - 4, val);

    // Border
    p.setPen(QPen(QColor(50, 55, 75), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(rect().adjusted(0,0,-1,-1));
}

// ── IMU3DWidget ───────────────────────────────────────────────────────────────

IMU3DWidget::IMU3DWidget(QWidget* parent)
    : BaseWidget(parent)
    , m_refreshTimer(new QTimer(this))
{
    setupUI();
    m_refreshTimer->setInterval(50); // 20 Hz UI update
    connect(m_refreshTimer, &QTimer::timeout, this, &IMU3DWidget::onRefreshTimer);
    m_refreshTimer->start();
}

void IMU3DWidget::setupUI() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(4);

    // ── 3D cube ───────────────────────────────────────────────────────────
    auto* cubeBox = new QGroupBox("Orientation (accel-derived)", this);
    auto* cubeLayout = new QVBoxLayout(cubeBox);
    m_cube = new IMUCubeView(this);
    cubeLayout->addWidget(m_cube);
    root->addWidget(cubeBox, 2);

    // ── Numeric readout ───────────────────────────────────────────────────
    auto* numBox = new QGroupBox("Raw values", this);
    auto* numGrid = new QGridLayout(numBox);
    numGrid->setSpacing(2);

    const char* names[6]  = {"Ax","Ay","Az","Gx","Gy","Gz"};
    const char* units[6]  = {"m/s²","m/s²","m/s²","rad/s","rad/s","rad/s"};
    QLabel** lbls[6] = {&m_axLbl,&m_ayLbl,&m_azLbl,&m_gxLbl,&m_gyLbl,&m_gzLbl};
    const char* colors[6] = {"#EF5350","#66BB6A","#42A5F5","#FFA726","#AB47BC","#26C6DA"};

    for (int i = 0; i < 6; ++i) {
        auto* name = new QLabel(QString("<b>%1</b>").arg(names[i]), this);
        name->setStyleSheet(QString("color:%1;").arg(colors[i]));
        name->setTextFormat(Qt::RichText);
        *lbls[i] = new QLabel("0.000", this);
        (*lbls[i])->setStyleSheet("color:#CFD8DC;font-family:monospace;");
        auto* unit = new QLabel(units[i], this);
        unit->setStyleSheet("color:#546E7A;font-size:10px;");
        numGrid->addWidget(name,     i, 0);
        numGrid->addWidget(*lbls[i], i, 1);
        numGrid->addWidget(unit,     i, 2);
    }
    root->addWidget(numBox);

    // ── Live graphs ───────────────────────────────────────────────────────
    const char* gLabels[6] = {"Ax","Ay","Az","Gx","Gy","Gz"};
    const QColor gColors[6] = {
        QColor(239,83,80), QColor(102,187,106), QColor(66,165,245),
        QColor(255,167,38), QColor(171,71,188), QColor(38,198,218)
    };

    auto* accelBox = new QGroupBox("Accelerometer (m/s²)", this);
    auto* accelLayout = new QVBoxLayout(accelBox);
    accelLayout->setSpacing(2);
    for (int i = 0; i < 3; ++i) {
        m_graphs[i] = new LiveGraph(gLabels[i], gColors[i], this);
        accelLayout->addWidget(m_graphs[i]);
    }
    root->addWidget(accelBox, 1);

    auto* gyroBox = new QGroupBox("Gyroscope (rad/s)", this);
    auto* gyroLayout = new QVBoxLayout(gyroBox);
    gyroLayout->setSpacing(2);
    for (int i = 3; i < 6; ++i) {
        m_graphs[i] = new LiveGraph(gLabels[i], gColors[i], this);
        gyroLayout->addWidget(m_graphs[i]);
    }
    root->addWidget(gyroBox, 1);
}

bool IMU3DWidget::initialize() {
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imuDataReceived,
                this, &IMU3DWidget::onIMUData);
        Logger::instance().info("IMU3DWidget initialized");
    }
    return true;
}

void IMU3DWidget::onIMUData(double ax, double ay, double az,
                             double gx, double gy, double gz)
{
    std::lock_guard<std::mutex> lk(m_mutex);
    m_ax = ax; m_ay = ay; m_az = az;
    m_gx = gx; m_gy = gy; m_gz = gz;
}

void IMU3DWidget::onRefreshTimer() {
    double ax, ay, az, gx, gy, gz;
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        ax=m_ax; ay=m_ay; az=m_az; gx=m_gx; gy=m_gy; gz=m_gz;
    }

    m_cube->setAccel(ax, ay, az);

    m_graphs[0]->push(ax); m_graphs[1]->push(ay); m_graphs[2]->push(az);
    m_graphs[3]->push(gx); m_graphs[4]->push(gy); m_graphs[5]->push(gz);

    m_axLbl->setText(QString::number(ax, 'f', 3));
    m_ayLbl->setText(QString::number(ay, 'f', 3));
    m_azLbl->setText(QString::number(az, 'f', 3));
    m_gxLbl->setText(QString::number(gx, 'f', 3));
    m_gyLbl->setText(QString::number(gy, 'f', 3));
    m_gzLbl->setText(QString::number(gz, 'f', 3));
}
