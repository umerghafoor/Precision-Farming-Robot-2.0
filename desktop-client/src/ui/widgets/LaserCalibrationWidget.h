#ifndef LASERCALIBRATIONWIDGET_H
#define LASERCALIBRATIONWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QKeyEvent>
#include <QPointF>
#include <QImage>
#include <array>

// ── CalibrationCanvas ─────────────────────────────────────────────────────────
// Draws the camera frame with 4 labelled corner targets overlaid.
// Emits clicked(QPointF) in image-normalised coords [0,1] for test-click mode.
class CalibrationCanvas : public QWidget {
    Q_OBJECT
public:
    explicit CalibrationCanvas(QWidget* parent = nullptr);

    void setFrame(const QImage& img);
    void setActiveCorner(int idx);        // 0-3, -1 = none
    void setCornerPixel(int idx, QPointF px);  // pixel coords in original image
    void setClickMode(bool enabled);

signals:
    void clicked(QPointF normalisedPos);  // [0,1]x[0,1]

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    QSize sizeHint() const override { return {640, 480}; }

private:
    QImage m_frame;
    int    m_activeCorner{-1};
    bool   m_clickMode{false};
    // corner pixel positions in *original image* coords, (-1,-1) = not set
    std::array<QPointF, 4> m_corners;

    QPointF imageToWidget(QPointF imgPt) const;
    QRectF  imageRect() const;
};

// ── LaserCalibrationWidget ────────────────────────────────────────────────────
class LaserCalibrationWidget : public BaseWidget {
    Q_OBJECT
public:
    explicit LaserCalibrationWidget(QWidget* parent = nullptr);
    bool    initialize() override;
    QString displayName() const override { return "Laser Calibration"; }

protected:
    void keyPressEvent(QKeyEvent* e) override;

private slots:
    void onImageReceived(const QByteArray& data, int w, int h);
    void onStartCorner(int idx);
    void onConfirmCorner();
    void onSaveCalibration();
    void onClearCalibration();
    void onTestModeToggled(bool on);
    void onCanvasClicked(QPointF norm);
    void onServoMoveTimer();

private:
    void setupUI();
    void updateStatus();
    void loadCalibration();
    void pointLaserAt(double normX, double normY);

    // Bilinear interpolation: pixel (normX,normY) → (servo1, servo2) angles
    // Returns false if calibration is incomplete
    bool bilinearMap(double nx, double ny, double& s1, double& s2) const;

    CalibrationCanvas* m_canvas;
    QLabel*     m_statusLabel;
    QLabel*     m_servo1Label;
    QLabel*     m_servo2Label;
    QPushButton* m_cornerBtns[4];
    QPushButton* m_confirmBtn;
    QPushButton* m_saveBtn;
    QPushButton* m_clearBtn;
    QPushButton* m_testBtn;
    QPushButton* m_laserBtn;

    // Calibration state
    // corners[i] = {pixel_x, pixel_y, servo1_angle, servo2_angle}
    struct CornerData {
        QPointF pixel{-1, -1};   // in original image coords
        double  s1{90}, s2{90};
        bool    set{false};
    };
    std::array<CornerData, 4> m_corners;
    int    m_activeCorner{-1};   // which corner we are currently calibrating
    bool   m_calibrated{false};
    bool   m_testMode{false};
    bool   m_laserOn{false};

    // Current servo angles
    double m_servo1{90}, m_servo2{90};

    // Arrow-key servo step (degrees per keypress)
    static constexpr double STEP = 1.0;

    // Repeat-move timer for held arrow keys
    QTimer* m_moveTimer;
    int     m_pendingDelta1{0}, m_pendingDelta2{0};

    // Last frame size
    int m_imgW{640}, m_imgH{480};
};

#endif // LASERCALIBRATIONWIDGET_H
