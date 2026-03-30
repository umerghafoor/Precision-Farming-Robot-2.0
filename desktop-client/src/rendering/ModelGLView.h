#pragma once
#ifdef HAVE_QT_OPENGL

#include "OBJMesh.h"
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QMap>
#include <QPoint>
#include <QVector3D>
#include <QQuaternion>

// GPU-side representation of one OBJ object
struct GLMesh {
    QString   name;
    GLuint    vao          = 0;
    GLuint    vbo          = 0;
    int       vertexCount  = 0;
    QVector3D centroid;
    QVector3D defaultColor { 0.58f, 0.60f, 0.63f };  // neutral steel-gray
};

/**
 * @brief OpenGL 3.3 widget that renders loaded OBJ meshes.
 *
 * Performance model: update() is only called when state actually changes
 * (new mesh data, status color, wheel angle, or camera interaction).
 * No continuous render loop.
 */
class ModelGLView : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit ModelGLView(QWidget *parent = nullptr);
    ~ModelGLView() override;

    // Call from main thread after OBJLoader signals loadFinished
    void loadMeshes(const QVector<OBJMeshData> &meshes);

    // Per-object status color (RGB 0–1).  Pass {-1,0,0} or call clear to remove.
    void setColorOverride(const QString &objectName, const QVector3D &color);
    void clearColorOverride(const QString &objectName);

    // Cumulative rotation angle (degrees) around the wheel's own X axis
    void setWheelAngle(const QString &objectName, float angleDegrees);

    // Move/orient the robot on the ground plane (ROS world coords)
    void setRobotPose(const QVector3D &pos, const QQuaternion &ori);

    void resetCamera();

signals:
    void glInitialized();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

private:
    void uploadPendingMeshes();
    void drawMesh(const GLMesh &mesh);
    QMatrix4x4 wheelModelMatrix(const GLMesh &mesh) const;
    void setupGrid(float sceneSize, float groundY);

    QOpenGLShaderProgram  *m_program      = nullptr;
    bool                   m_glReady      = false;

    QVector<GLMesh>        m_meshes;

    // Pending data from loader (uploaded on first paintGL after arrival)
    QVector<OBJMeshData>   m_pendingMeshes;
    bool                   m_hasPending   = false;

    QMap<QString, QVector3D> m_colorOverrides;
    QMap<QString, float>     m_wheelAngles;

    // Ground-plane grid
    GLuint  m_gridVao       = 0;
    GLuint  m_gridVbo       = 0;
    int     m_gridLineVerts = 0;
    bool    m_gridReady     = false;

    // Robot world pose (updated via setRobotPose)
    QMatrix4x4  m_robotMatrix;      // cached transform applied to all robot meshes
    QVector3D   m_robotPos;
    QQuaternion m_robotOri;

    // Orbit camera state
    float     m_azimuth         =  30.0f;
    float     m_elevation       =  18.0f;
    float     m_distance        =  10.0f;
    QVector3D m_target;

    // Saved initial fit — restored by resetCamera()
    float     m_initialDistance =  10.0f;
    QVector3D m_initialTarget;

    QMatrix4x4 m_proj;
    QPoint     m_lastMouse;
    bool       m_dragging = false;
};

#endif // HAVE_QT_OPENGL
