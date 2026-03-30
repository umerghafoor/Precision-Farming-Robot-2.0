#ifndef ROBOTMODELWIDGET_H
#define ROBOTMODELWIDGET_H

#include "BaseWidget.h"
#include "OBJMesh.h"
#include <QElapsedTimer>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QStackedWidget>

class OBJLoader;

#ifdef HAVE_QT_OPENGL
class ModelGLView;
#endif

/**
 * @brief Dock widget that renders the robot 3D model and overlays
 *        component status (battery, wheel health) as color highlights.
 *
 * Uses QOpenGLWidget + a minimal OBJ parser (no external deps).
 * Loads the model asynchronously; shows a progress bar during load.
 *
 * Wheel objects are identified by their OBJ object names:
 *   Cube.048_Cube.080, Cube.046_Cube.063, Cube.039
 *
 * Swap WHEEL_OBJECT_NAMES and MODEL_PATH for the real robot model.
 */
class RobotModelWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit RobotModelWidget(QWidget *parent = nullptr);
    ~RobotModelWidget() override;

    bool    initialize() override;
    QString displayName() const override { return "Robot 3D Model"; }

private slots:
    void onLoadProgress(int percent);
    void onLoadFinished(QVector<OBJMeshData> meshes);
    void onLoadFailed(const QString &reason);
    void onTwinStateChanged();
    void onResetCamera();

private:
    void setupUI();
    void startLoading();
    void showGLView();
    void updateBatteryDisplay(double level);
    void updateWheelStatus(const QString &robotStatus);

    // ── Pages ─────────────────────────────────────────────────────────────
    QStackedWidget *m_stack        = nullptr;

    // Page 0 – loading
    QLabel        *m_loadLabel     = nullptr;
    QProgressBar  *m_progressBar   = nullptr;

    // Page 1 – 3-D view
#ifdef HAVE_QT_OPENGL
    ModelGLView   *m_glView        = nullptr;
#endif
    QPushButton   *m_resetBtn      = nullptr;

    // Page 2 – not supported
    QLabel        *m_unsupported   = nullptr;

    // ── Status bar ────────────────────────────────────────────────────────
    QProgressBar  *m_batteryBar    = nullptr;
    QLabel        *m_batteryLabel  = nullptr;
    QLabel        *m_wheelDots[3]  = {};  // FL FR RL

    // ── Loader ────────────────────────────────────────────────────────────
    OBJLoader     *m_loader        = nullptr;

    // ── Wheel rotation state ─────────────────────────────────────────────
    QElapsedTimer  m_wheelTimer;
    float          m_wheelAngle    = 0.0f;   // cumulative degrees
};

#endif // ROBOTMODELWIDGET_H
