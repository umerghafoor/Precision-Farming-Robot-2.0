#include "RobotMapView.h"

#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QtMath>
#include <algorithm>

// Robot body dimensions in metres
static constexpr double ROBOT_LENGTH = 0.30;
static constexpr double ROBOT_WIDTH  = 0.25;

// Pixels of empty margin around the auto-fit bounding box
static constexpr double FIT_MARGIN_PX = 40.0;

// Minimum "virtual" field size so an idle robot still shows a visible grid (metres)
static constexpr double MIN_FIELD = 2.0;

RobotMapView::RobotMapView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(200, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setBackgroundRole(QPalette::Base);
    setAutoFillBackground(true);
}

void RobotMapView::setRobotPose(double x, double y, const QQuaternion& orientation)
{
    m_pose.x = x;
    m_pose.y = y;

    // Extract yaw from quaternion  (rotation around Z, in degrees for QPainter)
    // Using the standard formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    float qw = orientation.scalar();
    float qx = orientation.x();
    float qy = orientation.y();
    float qz = orientation.z();
    double yawRad = std::atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz));
    m_pose.yawDeg = qRadiansToDegrees(yawRad);

    // Append to trail (skip if too close to the last point to avoid clutter)
    QPointF newPt(x, y);
    if (m_trail.isEmpty() ||
        QLineF(m_trail.last(), newPt).length() > 0.02)
    {
        m_trail.append(newPt);
        m_transformDirty = true;
    }

    update();
}

void RobotMapView::clear()
{
    m_trail.clear();
    m_pose = {};
    m_transformDirty = true;
    update();
}

// ---------------------------------------------------------------------------
// Transform helpers
// ---------------------------------------------------------------------------

void RobotMapView::updateTransform()
{
    if (!m_transformDirty) return;
    m_transformDirty = false;

    // Compute bounding box in world space
    double minX = m_pose.x, maxX = m_pose.x;
    double minY = m_pose.y, maxY = m_pose.y;

    for (const QPointF& pt : m_trail) {
        minX = std::min(minX, pt.x());
        maxX = std::max(maxX, pt.x());
        minY = std::min(minY, pt.y());
        maxY = std::max(maxY, pt.y());
    }

    // Ensure a minimum field size around the robot
    double cx = (minX + maxX) / 2.0;
    double cy = (minY + maxY) / 2.0;
    double spanX = std::max(maxX - minX, MIN_FIELD);
    double spanY = std::max(maxY - minY, MIN_FIELD);
    minX = cx - spanX / 2.0;
    maxX = cx + spanX / 2.0;
    minY = cy - spanY / 2.0;
    maxY = cy + spanY / 2.0;

    double usableW = width()  - 2 * FIT_MARGIN_PX;
    double usableH = height() - 2 * FIT_MARGIN_PX;
    if (usableW < 1) usableW = 1;
    if (usableH < 1) usableH = 1;

    double scaleX = usableW / spanX;
    double scaleY = usableH / spanY;
    m_scale = std::min(scaleX, scaleY);

    // Centre the view
    double worldCX = (minX + maxX) / 2.0;
    double worldCY = (minY + maxY) / 2.0;
    m_offset = QPointF(width()  / 2.0 - m_scale * worldCX,
                       height() / 2.0 + m_scale * worldCY); // Y is flipped
}

QPointF RobotMapView::worldToWidget(double x, double y) const
{
    // Y is negated because widget Y grows downward, world Y grows upward
    return QPointF(m_offset.x() + m_scale * x,
                   m_offset.y() - m_scale * y);
}

// ---------------------------------------------------------------------------
// Paint
// ---------------------------------------------------------------------------

void RobotMapView::paintEvent(QPaintEvent*)
{
    updateTransform();

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    drawGrid(p);
    drawTrail(p);
    drawRobot(p);
}

void RobotMapView::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    m_transformDirty = true;
}

void RobotMapView::drawGrid(QPainter& p) const
{
    // Draw a light grid every 0.5 m in world space
    const double gridStep = 0.5; // metres
    QPen gridPen(QColor(60, 80, 60, 80), 1, Qt::DotLine);
    p.setPen(gridPen);

    // Determine world extent visible
    // (invert the transform for the four widget corners)
    auto widgetToWorldX = [&](double px) { return (px - m_offset.x()) / m_scale; };
    auto widgetToWorldY = [&](double py) { return (m_offset.y() - py) / m_scale; };

    double wMinX = widgetToWorldX(0);
    double wMaxX = widgetToWorldX(width());
    double wMinY = widgetToWorldY(height());
    double wMaxY = widgetToWorldY(0);

    // Snap to grid
    double startX = std::floor(wMinX / gridStep) * gridStep;
    double startY = std::floor(wMinY / gridStep) * gridStep;

    for (double wx = startX; wx <= wMaxX; wx += gridStep) {
        QPointF top    = worldToWidget(wx, wMaxY);
        QPointF bottom = worldToWidget(wx, wMinY);
        p.drawLine(top, bottom);
    }
    for (double wy = startY; wy <= wMaxY; wy += gridStep) {
        QPointF left  = worldToWidget(wMinX, wy);
        QPointF right = worldToWidget(wMaxX, wy);
        p.drawLine(left, right);
    }

    // World-origin axes
    QPen axisPen(QColor(80, 120, 80, 140), 1);
    p.setPen(axisPen);
    QPointF originW = worldToWidget(0, 0);
    p.drawLine(worldToWidget(wMinX, 0), worldToWidget(wMaxX, 0)); // X axis
    p.drawLine(worldToWidget(0, wMinY), worldToWidget(0, wMaxY)); // Y axis

    // Origin label
    p.setPen(QColor(120, 160, 120));
    p.setFont(QFont("monospace", 8));
    p.drawText(originW + QPointF(4, -4), "0,0");
}

void RobotMapView::drawTrail(QPainter& p) const
{
    if (m_trail.size() < 2) return;

    QPen trailPen(QColor(80, 180, 100, 180), 2);
    p.setPen(trailPen);
    p.setBrush(Qt::NoBrush);

    QPainterPath path;
    QPointF first = worldToWidget(m_trail[0].x(), m_trail[0].y());
    path.moveTo(first);
    for (int i = 1; i < m_trail.size(); ++i) {
        path.lineTo(worldToWidget(m_trail[i].x(), m_trail[i].y()));
    }
    p.drawPath(path);

    // Start marker
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(80, 180, 100, 200));
    p.drawEllipse(first, 4, 4);
}

void RobotMapView::drawRobot(QPainter& p) const
{
    QPointF centre = worldToWidget(m_pose.x, m_pose.y);
    double halfL = m_scale * ROBOT_LENGTH / 2.0;
    double halfW = m_scale * ROBOT_WIDTH  / 2.0;

    p.save();
    p.translate(centre);
    // QPainter rotates clockwise, world yaw is CCW from +X, and we flip Y —
    // so the painter angle is -yawDeg.
    p.rotate(-m_pose.yawDeg);

    // Robot body
    p.setPen(QPen(QColor(200, 220, 200), 2));
    p.setBrush(QColor(40, 100, 60, 200));
    p.drawRect(QRectF(-halfL, -halfW, halfL * 2, halfW * 2));

    // Direction arrow (front = +X in robot frame)
    double arrowLen = halfL * 0.8;
    p.setPen(QPen(QColor(180, 255, 180), 2));
    p.drawLine(QPointF(0, 0), QPointF(arrowLen, 0));
    // Arrowhead
    p.setBrush(QColor(180, 255, 180));
    p.setPen(Qt::NoPen);
    QPolygonF head;
    double ht = halfW * 0.5;
    head << QPointF(arrowLen + ht, 0)
         << QPointF(arrowLen - ht * 0.7,  ht * 0.7)
         << QPointF(arrowLen - ht * 0.7, -ht * 0.7);
    p.drawPolygon(head);

    p.restore();

    // Coordinate label near robot
    p.setPen(QColor(200, 240, 200));
    p.setFont(QFont("monospace", 8));
    p.drawText(centre + QPointF(halfL + 4, 4),
               QString("(%1, %2)").arg(m_pose.x, 0, 'f', 2)
                                  .arg(m_pose.y, 0, 'f', 2));
}
