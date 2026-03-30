#ifndef ROBOTMAPVIEW_H
#define ROBOTMAPVIEW_H

#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QQuaternion>

/**
 * @brief 2D overhead map that renders the robot's current position,
 *        heading, and path trail using QPainter.
 *
 * Coordinate system: robot world units (metres). The view auto-fits
 * to the bounding box of the trail with a small margin.
 */
class RobotMapView : public QWidget
{
    Q_OBJECT

public:
    explicit RobotMapView(QWidget *parent = nullptr);

    void setRobotPose(double x, double y, const QQuaternion& orientation);
    void clear();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    struct Pose {
        double x = 0.0;
        double y = 0.0;
        double yawDeg = 0.0;
    };

    // Convert world coords (metres) to widget pixels using current transform
    QPointF worldToWidget(double x, double y) const;

    // Recompute the world-to-widget scale/offset from the trail bounding box
    void updateTransform();

    void drawGrid(QPainter& p) const;
    void drawTrail(QPainter& p) const;
    void drawRobot(QPainter& p) const;

    Pose m_pose;
    QVector<QPointF> m_trail; // world-space positions

    // View transform: widget_pos = m_scale * world_pos + m_offset
    double  m_scale  = 60.0; // pixels per metre (initial)
    QPointF m_offset;        // pixel offset to world origin
    bool    m_transformDirty = true;
};

#endif // ROBOTMAPVIEW_H
