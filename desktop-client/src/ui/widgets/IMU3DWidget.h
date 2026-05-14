#ifndef IMU3DWIDGET_H
#define IMU3DWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QTimer>
#include <QPainter>
#include <deque>
#include <array>
#include <mutex>

// ── IMUCubeView ───────────────────────────────────────────────────────────────
// Draws a 3D wireframe box rotated to match accelerometer-derived tilt.
class IMUCubeView : public QWidget {
    Q_OBJECT
public:
    explicit IMUCubeView(QWidget* parent = nullptr);
    void setAccel(double ax, double ay, double az);
    QSize sizeHint() const override { return {240, 240}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    double m_ax{0}, m_ay{0}, m_az{1};

    // Project a 3D point (rotated by pitch/roll derived from accel) to 2D
    struct Vec3 { double x, y, z; };
    QPointF project(Vec3 v, double cx, double cy, double scale) const;
    void rotateY(Vec3& v, double rad) const;
    void rotateX(Vec3& v, double rad) const;
};

// ── LiveGraph ─────────────────────────────────────────────────────────────────
// Scrolling line-graph for one axis value.
class LiveGraph : public QWidget {
    Q_OBJECT
public:
    explicit LiveGraph(const QString& label, const QColor& color, QWidget* parent = nullptr);
    void push(double value);
    QSize sizeHint() const override { return {300, 60}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    static constexpr int HISTORY = 200;
    QString      m_label;
    QColor       m_color;
    std::deque<double> m_data;
    double       m_min{-1}, m_max{1};
};

// ── IMU3DWidget ───────────────────────────────────────────────────────────────
class IMU3DWidget : public BaseWidget {
    Q_OBJECT
public:
    explicit IMU3DWidget(QWidget* parent = nullptr);
    bool    initialize() override;
    QString displayName() const override { return "IMU 3D View"; }

private slots:
    void onIMUData(double ax, double ay, double az, double gx, double gy, double gz);
    void onRefreshTimer();

private:
    void setupUI();

    IMUCubeView* m_cube;

    // 6 live graphs: Ax Ay Az Gx Gy Gz
    LiveGraph* m_graphs[6];

    // Numeric labels
    QLabel* m_axLbl; QLabel* m_ayLbl; QLabel* m_azLbl;
    QLabel* m_gxLbl; QLabel* m_gyLbl; QLabel* m_gzLbl;

    // Cached latest values
    std::mutex m_mutex;
    double m_ax{0}, m_ay{0}, m_az{0};
    double m_gx{0}, m_gy{0}, m_gz{0};

    QTimer* m_refreshTimer;
};

#endif // IMU3DWIDGET_H
