#include "ModelGLView.h"
#ifdef HAVE_QT_OPENGL

#include "Logger.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QtMath>
#include <cmath>
#include <cstddef>   // offsetof
#include <algorithm> // std::initializer_list max

// ─── Embedded shaders ────────────────────────────────────────────────────────

static const char *VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;

uniform mat4 uPV;
uniform mat4 uModel;

out vec3 vNorm;

void main() {
    gl_Position = uPV * uModel * vec4(aPos, 1.0);
    // uModel is always rotation-only, so mat3(uModel) is the correct normal matrix
    vNorm = mat3(uModel) * aNorm;
}
)";

static const char *FRAG_SRC = R"(
#version 330 core
in vec3 vNorm;

uniform vec3 uColor;
uniform bool uUnlit;   // true → flat color (used for grid lines)

out vec4 FragColor;

void main() {
    if (uUnlit) {
        FragColor = vec4(uColor, 1.0);
        return;
    }
    vec3 n    = normalize(vNorm);
    // Two-light Phong: warm key from top-right, cool fill from left
    vec3 key  = normalize(vec3( 1.0, 1.8, 1.2));
    vec3 fill = normalize(vec3(-1.0, 0.4,-0.5));
    float kd  = max(dot(n, key),  0.0) * 0.62;
    float fd  = max(dot(n, fill), 0.0) * 0.18;
    float amb = 0.22;
    FragColor = vec4(uColor * (amb + kd + fd), 1.0);
}
)";

// ─── Constructor / Destructor ─────────────────────────────────────────────────

ModelGLView::ModelGLView(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setMouseTracking(false);
    setFocusPolicy(Qt::WheelFocus);   // receive wheel events without a prior click
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    setFormat(fmt);
}

ModelGLView::~ModelGLView()
{
    makeCurrent();
    for (auto &mesh : m_meshes) {
        if (mesh.vao) glDeleteVertexArrays(1, &mesh.vao);
        if (mesh.vbo) glDeleteBuffers(1, &mesh.vbo);
    }
    if (m_gridVao) glDeleteVertexArrays(1, &m_gridVao);
    if (m_gridVbo) glDeleteBuffers(1, &m_gridVbo);
    delete m_program;
    doneCurrent();
}

// ─── Public API ───────────────────────────────────────────────────────────────

void ModelGLView::loadMeshes(const QVector<OBJMeshData> &meshes)
{
    // Delete any previously uploaded meshes
    if (m_glReady) {
        makeCurrent();
        for (auto &mesh : m_meshes) {
            glDeleteVertexArrays(1, &mesh.vao);
            glDeleteBuffers(1, &mesh.vbo);
        }
        doneCurrent();
    }
    m_meshes.clear();

    m_pendingMeshes = meshes;
    m_hasPending    = true;

    // Auto-fit camera to scene bounds
    if (!meshes.isEmpty()) {
        QVector3D gmin( 1e9f,  1e9f,  1e9f);
        QVector3D gmax(-1e9f, -1e9f, -1e9f);
        for (const auto &m : meshes) {
            gmin = QVector3D(qMin(gmin.x(), m.boundsMin.x()),
                             qMin(gmin.y(), m.boundsMin.y()),
                             qMin(gmin.z(), m.boundsMin.z()));
            gmax = QVector3D(qMax(gmax.x(), m.boundsMax.x()),
                             qMax(gmax.y(), m.boundsMax.y()),
                             qMax(gmax.z(), m.boundsMax.z()));
        }
        m_target = (gmin + gmax) * 0.5f;
        float maxDim = qMax(qMax(gmax.x() - gmin.x(),
                                 gmax.y() - gmin.y()),
                                 gmax.z() - gmin.z());
        m_distance = maxDim * 1.4f;

        // Save for resetCamera()
        m_initialDistance = m_distance;
        m_initialTarget   = m_target;
    }

    update();
}

void ModelGLView::setColorOverride(const QString &name, const QVector3D &color)
{
    m_colorOverrides[name] = color;
    update();
}

void ModelGLView::clearColorOverride(const QString &name)
{
    m_colorOverrides.remove(name);
    update();
}

void ModelGLView::setWheelAngle(const QString &name, float angleDeg)
{
    m_wheelAngles[name] = angleDeg;
    // Caller is responsible for calling update() once per frame
}

void ModelGLView::setRobotPose(const QVector3D &pos, const QQuaternion &ori)
{
    m_robotPos = pos;
    m_robotOri = ori;

    m_robotMatrix.setToIdentity();
    // ROS convention: x=forward, y=left, z=up.
    // Map onto the GL ground plane (Y-up): ROS(x,y) → GL(x, 0, -y).
    m_robotMatrix.translate(pos.x(), 0.0f, -pos.y());
    // Extract yaw (rotation around ROS Z = GL Y)
    float yawRad = 2.0f * std::atan2(ori.z(), ori.scalar());
    m_robotMatrix.rotate(qRadiansToDegrees(yawRad), 0.0f, 1.0f, 0.0f);

    update();
}

void ModelGLView::resetCamera()
{
    m_azimuth   =  30.0f;
    m_elevation =  18.0f;
    m_distance  =  m_initialDistance;
    m_target    =  m_initialTarget;
    update();
}

// ─── OpenGL overrides ─────────────────────────────────────────────────────────

void ModelGLView::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glClearColor(0.11f, 0.11f, 0.13f, 1.0f);

    m_program = new QOpenGLShaderProgram(this);
    bool ok = true;
    ok &= m_program->addShaderFromSourceCode(QOpenGLShader::Vertex,   VERT_SRC);
    ok &= m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SRC);
    ok &= m_program->link();

    if (!ok) {
        Logger::instance().error("ModelGLView: shader compile/link failed: "
                                 + m_program->log());
    }

    m_glReady = true;

    if (m_hasPending)
        uploadPendingMeshes();

    emit glInitialized();
}

void ModelGLView::resizeGL(int w, int h)
{
    float aspect = float(w) / float(h > 0 ? h : 1);
    m_proj.setToIdentity();
    m_proj.perspective(45.0f, aspect, 0.05f, 5000.0f);
}

void ModelGLView::paintGL()
{
    if (m_hasPending)
        uploadPendingMeshes();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_glReady || m_meshes.isEmpty())
        return;

    // Build view from orbit parameters
    float az = qDegreesToRadians(m_azimuth);
    float el = qDegreesToRadians(m_elevation);
    QVector3D eye = m_target + QVector3D(
        m_distance * std::cos(el) * std::sin(az),
        m_distance * std::sin(el),
        m_distance * std::cos(el) * std::cos(az)
    );

    QMatrix4x4 view;
    view.lookAt(eye, m_target, {0.0f, 1.0f, 0.0f});

    QMatrix4x4 pv = m_proj * view;

    m_program->bind();
    m_program->setUniformValue("uPV", pv);

    // ── Ground plane grid ────────────────────────────────────────────────────
    if (m_gridReady) {
        QMatrix4x4 identity;
        m_program->setUniformValue("uModel",  identity);
        m_program->setUniformValue("uColor",  QVector3D(0.20f, 0.32f, 0.20f));
        m_program->setUniformValue("uUnlit",  true);
        glBindVertexArray(m_gridVao);
        glDrawArrays(GL_LINES, 0, m_gridLineVerts);
        glBindVertexArray(0);
        m_program->setUniformValue("uUnlit",  false);
    }

    // ── Robot meshes (with world pose applied) ───────────────────────────────
    for (const auto &mesh : m_meshes)
        drawMesh(mesh);

    m_program->release();
}

// ─── Mouse / wheel input ──────────────────────────────────────────────────────

void ModelGLView::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton || e->button() == Qt::RightButton) {
        m_lastMouse = e->pos();
        m_dragging  = true;
    }
}

void ModelGLView::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_dragging) return;
    QPoint delta = e->pos() - m_lastMouse;
    m_lastMouse  = e->pos();

    if (e->buttons() & Qt::LeftButton) {
        // Orbit
        m_azimuth   += delta.x() * 0.4f;
        m_elevation  = qBound(-89.0f, m_elevation - delta.y() * 0.4f, 89.0f);
    } else if (e->buttons() & Qt::RightButton) {
        // Pan: move target in camera's local XY plane
        float az = qDegreesToRadians(m_azimuth);
        QVector3D right(std::cos(az), 0, -std::sin(az));
        QVector3D up(0, 1, 0);
        float panScale = m_distance * 0.001f;
        m_target -= right * float(delta.x()) * panScale;
        m_target += up    * float(delta.y()) * panScale;
    }

    update();
}

void ModelGLView::mouseReleaseEvent(QMouseEvent *)
{
    m_dragging = false;
}

void ModelGLView::wheelEvent(QWheelEvent *e)
{
    int delta = e->angleDelta().y();
    if (delta == 0) delta = e->pixelDelta().y();   // trackpad fallback
    if (delta == 0) { e->ignore(); return; }

    float factor = delta > 0 ? 0.88f : 1.14f;
    m_distance   = qBound(0.01f, m_distance * factor, 10000.0f);
    e->accept();
    update();
}

// ─── Private helpers ──────────────────────────────────────────────────────────

void ModelGLView::uploadPendingMeshes()
{
    // Compute scene bounds to position the grid correctly
    QVector3D sMin( 1e9f,  1e9f,  1e9f);
    QVector3D sMax(-1e9f, -1e9f, -1e9f);
    for (const auto &data : m_pendingMeshes) {
        if (data.vertices.isEmpty()) continue;
        sMin = QVector3D(qMin(sMin.x(), data.boundsMin.x()),
                         qMin(sMin.y(), data.boundsMin.y()),
                         qMin(sMin.z(), data.boundsMin.z()));
        sMax = QVector3D(qMax(sMax.x(), data.boundsMax.x()),
                         qMax(sMax.y(), data.boundsMax.y()),
                         qMax(sMax.z(), data.boundsMax.z()));
    }

    for (const auto &data : m_pendingMeshes) {
        if (data.vertices.isEmpty()) continue;

        GLMesh mesh;
        mesh.name        = data.name;
        mesh.centroid    = data.centroid;
        mesh.vertexCount = data.vertices.size();

        glGenVertexArrays(1, &mesh.vao);
        glGenBuffers(1, &mesh.vbo);

        glBindVertexArray(mesh.vao);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     (GLsizeiptr)(data.vertices.size() * sizeof(OBJVertex)),
                     data.vertices.constData(),
                     GL_STATIC_DRAW);

        // attrib 0: position (px,py,pz)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              sizeof(OBJVertex),
                              (void*)offsetof(OBJVertex, px));
        glEnableVertexAttribArray(0);

        // attrib 1: normal (nx,ny,nz)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                              sizeof(OBJVertex),
                              (void*)offsetof(OBJVertex, nx));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
        m_meshes.append(mesh);
    }

    // Build ground grid now that we know the scene dimensions
    float sceneSize = qMax(sMax.x() - sMin.x(),
                      qMax(sMax.y() - sMin.y(),
                           sMax.z() - sMin.z()));
    setupGrid(sceneSize, sMin.y());

    m_pendingMeshes.clear();
    m_hasPending = false;
}

void ModelGLView::drawMesh(const GLMesh &mesh)
{
    QVector3D color = m_colorOverrides.value(mesh.name, mesh.defaultColor);

    // Base transform = robot world pose; wheels get an additional local spin
    QMatrix4x4 model = m_robotMatrix;
    if (m_wheelAngles.contains(mesh.name)) {
        float angle = m_wheelAngles[mesh.name];
        QMatrix4x4 spin;
        spin.translate( mesh.centroid);
        spin.rotate(angle, 1.0f, 0.0f, 0.0f);
        spin.translate(-mesh.centroid);
        model = model * spin;
    }

    m_program->setUniformValue("uModel", model);
    m_program->setUniformValue("uColor", color);

    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
    glBindVertexArray(0);
}

QMatrix4x4 ModelGLView::wheelModelMatrix(const GLMesh &mesh) const
{
    float angle = m_wheelAngles[mesh.name];
    QMatrix4x4 m;
    m.translate(mesh.centroid);
    m.rotate(angle, 1.0f, 0.0f, 0.0f);
    m.translate(-mesh.centroid);
    return m;
}

void ModelGLView::setupGrid(float sceneSize, float groundY)
{
    // Delete any previous grid buffers
    if (m_gridVao) { glDeleteVertexArrays(1, &m_gridVao); m_gridVao = 0; }
    if (m_gridVbo) { glDeleteBuffers(1, &m_gridVbo); m_gridVbo = 0; }

    // Cell size = ~25% of model extent; grid covers ±20 cells each way
    float cell    = qMax(sceneSize * 0.25f, 0.01f);
    int   half    = 20;
    float ext     = half * cell;
    float Y       = groundY - 0.001f;   // just below the model's feet

    QVector<OBJVertex> verts;
    verts.reserve((half * 2 + 1) * 4);

    for (int i = -half; i <= half; ++i) {
        float t = i * cell;
        // Line along X
        verts.push_back({ -ext, Y, t,   0.0f, 1.0f, 0.0f });
        verts.push_back({  ext, Y, t,   0.0f, 1.0f, 0.0f });
        // Line along Z
        verts.push_back({ t, Y, -ext,   0.0f, 1.0f, 0.0f });
        verts.push_back({ t, Y,  ext,   0.0f, 1.0f, 0.0f });
    }

    m_gridLineVerts = verts.size();

    glGenVertexArrays(1, &m_gridVao);
    glGenBuffers(1, &m_gridVbo);

    glBindVertexArray(m_gridVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(verts.size() * sizeof(OBJVertex)),
                 verts.constData(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(OBJVertex), (void*)offsetof(OBJVertex, px));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          sizeof(OBJVertex), (void*)offsetof(OBJVertex, nx));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    m_gridReady = true;
}

#endif // HAVE_QT_OPENGL
