#include "ModelGLView.h"
#ifdef HAVE_QT_OPENGL

#include "Logger.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QImage>
#include <QtMath>
#include <cmath>
#include <cstddef>   // offsetof
#include <algorithm> // std::initializer_list max

// ─── Embedded shaders ────────────────────────────────────────────────────────

static const char *VERT_SRC = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNorm;
layout(location = 2) in vec2 aUV;

uniform mat4 uPV;
uniform mat4 uModel;

out vec3 vNorm;
out vec2 vUV;

void main() {
    gl_Position = uPV * uModel * vec4(aPos, 1.0);
    vNorm = mat3(uModel) * aNorm;
    vUV   = aUV;
}
)";

static const char *FRAG_SRC = R"(
#version 330 core
in vec3 vNorm;
in vec2 vUV;

uniform vec3      uColor;
uniform bool      uUnlit;    // true → flat color (grid lines)
uniform sampler2D uTex;
uniform bool      uHasTex;   // true → sample uTex; false → use uColor

out vec4 FragColor;

void main() {
    if (uUnlit) {
        FragColor = vec4(uColor, 1.0);
        return;
    }
    vec3 n    = normalize(vNorm);
    vec3 key  = normalize(vec3( 1.0, 1.8, 1.2));
    vec3 fill = normalize(vec3(-1.0, 0.4,-0.5));
    float kd  = max(dot(n, key),  0.0) * 0.62;
    float fd  = max(dot(n, fill), 0.0) * 0.18;
    float amb = 0.22;
    vec3 base = uHasTex ? texture(uTex, vUV).rgb : uColor;
    FragColor = vec4(base * (amb + kd + fd), 1.0);
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
        if (mesh.vao)   glDeleteVertexArrays(1, &mesh.vao);
        if (mesh.vbo)   glDeleteBuffers(1, &mesh.vbo);
        if (mesh.texId) glDeleteTextures(1, &mesh.texId);
    }
    if (m_gridVao)  glDeleteVertexArrays(1, &m_gridVao);
    if (m_gridVbo)  glDeleteBuffers(1, &m_gridVbo);
    if (m_trailVao) glDeleteVertexArrays(1, &m_trailVao);
    if (m_trailVbo) glDeleteBuffers(1, &m_trailVbo);
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
            if (mesh.texId) glDeleteTextures(1, &mesh.texId);
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
    // The robot OBJ model's native forward is GL +Z; rotate -90° around Y
    // so the model visually faces GL +X (= ROS forward direction).
    m_robotMatrix.rotate(-90.0f, 0.0f, 1.0f, 0.0f);

    // Append to path trail (XZ only; Y applied at render time)
    QVector3D glPt(pos.x(), 0.0f, -pos.y());
    if (m_trailPts.isEmpty() ||
        (glPt - m_trailPts.last()).length() >= TRAIL_MIN_DIST)
    {
        if (m_trailPts.size() >= MAX_TRAIL_PTS)
            m_trailPts.removeFirst();   // oldest point out
        m_trailPts.append(glPt);
        m_trailDirty = true;
    }

    if (m_glReady && m_trailDirty)
        uploadTrail();

    update();
}

void ModelGLView::resetCamera()
{
    m_azimuth   =  30.0f;
    m_elevation =  18.0f;
    m_distance  =  m_initialDistance;
    m_target    =  m_initialTarget;
    m_cameraMode = CameraMode::Orbit;
    update();
}

void ModelGLView::setCameraMode(CameraMode mode)
{
    m_cameraMode = mode;
    // Give TopView a sensible starting height
    if (mode == CameraMode::TopView && m_distance < 2.0f)
        m_distance = 10.0f;
    update();
}

void ModelGLView::zoomIn()
{
    if (m_cameraMode == CameraMode::BackView) return;
    m_distance = qBound(0.5f, m_distance * 0.80f, 10000.0f);
    update();
}

void ModelGLView::zoomOut()
{
    if (m_cameraMode == CameraMode::BackView) return;
    m_distance = qBound(0.5f, m_distance * 1.25f, 10000.0f);
    update();
}

void ModelGLView::clearTrail()
{
    m_trailPts.clear();
    m_trailDirty = true;
    if (m_glReady) uploadTrail();
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

    // Allocate trail VAO/VBO (empty; filled dynamically as robot moves)
    glGenVertexArrays(1, &m_trailVao);
    glGenBuffers(1, &m_trailVbo);
    glBindVertexArray(m_trailVao);
    glBindBuffer(GL_ARRAY_BUFFER, m_trailVbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

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

    // ── Build view matrix based on camera mode ────────────────────────────────
    QMatrix4x4 view;
    // Robot position in GL space (ROS x→GL x, ROS y→GL -z)
    QVector3D robotGL(m_robotPos.x(), 0.0f, -m_robotPos.y());
    // Robot yaw in GL space
    float yawRad = 2.0f * std::atan2(m_robotOri.z(), m_robotOri.scalar());
    // GL forward vector for the robot: rotate(+X, yawRad, Y) = (cos(yaw), 0, -sin(yaw))
    QVector3D glForward(std::cos(yawRad), 0.0f, -std::sin(yawRad));

    // Reference point for snapping the infinite grid
    QVector3D gridSnapRef;

    if (m_cameraMode == CameraMode::Orbit) {
        float az = qDegreesToRadians(m_azimuth);
        float el = qDegreesToRadians(m_elevation);
        QVector3D eye = m_target + QVector3D(
            m_distance * std::cos(el) * std::sin(az),
            m_distance * std::sin(el),
            m_distance * std::cos(el) * std::cos(az)
        );
        view.lookAt(eye, m_target, {0.0f, 1.0f, 0.0f});
        gridSnapRef = m_target;

    } else if (m_cameraMode == CameraMode::TopView) {
        // Overhead bird's-eye, robot-following; +X (ROS forward) appears upward
        QVector3D eye = robotGL + QVector3D(0.0f, m_distance, 0.0f);
        view.lookAt(eye, robotGL, {1.0f, 0.0f, 0.0f});
        gridSnapRef = robotGL;

    } else { // BackView
        // Fixed behind and above the robot, looking toward robot + slightly ahead
        constexpr float BACK_DIST   = 3.0f;   // metres behind robot
        constexpr float BACK_HEIGHT = 1.5f;   // metres above ground
        QVector3D glBack = -glForward;
        QVector3D eye    = robotGL + glBack * BACK_DIST
                                   + QVector3D(0.0f, BACK_HEIGHT, 0.0f);
        // Look toward a point slightly ahead of the robot (not just its base)
        QVector3D lookAt = robotGL + glForward * 2.0f + QVector3D(0.0f, 0.4f, 0.0f);
        view.lookAt(eye, lookAt, {0.0f, 1.0f, 0.0f});
        gridSnapRef = robotGL;
    }

    QMatrix4x4 pv = m_proj * view;

    m_program->bind();
    m_program->setUniformValue("uPV", pv);

    // ── Ground plane grid (infinite: snap grid origin to nearest cell) ────────
    if (m_gridReady) {
        constexpr float CELL = 1.0f;   // must match setupGrid cell size
        float snapX = std::floor(gridSnapRef.x() / CELL) * CELL;
        float snapZ = std::floor(gridSnapRef.z() / CELL) * CELL;
        QMatrix4x4 gridModel;
        gridModel.translate(snapX, 0.0f, snapZ);   // Y is baked into vertices

        m_program->setUniformValue("uModel",  gridModel);
        m_program->setUniformValue("uColor",  QVector3D(0.20f, 0.32f, 0.20f));
        m_program->setUniformValue("uUnlit",  true);
        glBindVertexArray(m_gridVao);
        glDrawArrays(GL_LINES, 0, m_gridLineVerts);
        glBindVertexArray(0);
        m_program->setUniformValue("uUnlit",  false);
    }

    // ── Path trail ────────────────────────────────────────────────────────────
    if (m_trailVao && m_trailPts.size() >= 2) {
        // Lift the trail slightly above the grid plane
        QMatrix4x4 trailModel;
        trailModel.translate(0.0f, m_gridGroundY + 0.01f, 0.0f);

        m_program->setUniformValue("uModel",  trailModel);
        m_program->setUniformValue("uColor",  QVector3D(1.0f, 0.55f, 0.0f));  // orange
        m_program->setUniformValue("uUnlit",  true);
        m_program->setUniformValue("uHasTex", false);
        glLineWidth(2.0f);
        glBindVertexArray(m_trailVao);
        glDrawArrays(GL_LINE_STRIP, 0, m_trailPts.size());
        glBindVertexArray(0);
        glLineWidth(1.0f);
        m_program->setUniformValue("uUnlit", false);
    }

    // ── Robot meshes (with world pose applied) ───────────────────────────────
    for (const auto &mesh : m_meshes)
        drawMesh(mesh);

    m_program->release();
}

// ─── Mouse / wheel input ──────────────────────────────────────────────────────

void ModelGLView::mousePressEvent(QMouseEvent *e)
{
    if (m_cameraMode != CameraMode::Orbit) return;
    if (e->button() == Qt::LeftButton || e->button() == Qt::RightButton) {
        m_lastMouse = e->pos();
        m_dragging  = true;
    }
}

void ModelGLView::mouseMoveEvent(QMouseEvent *e)
{
    if (!m_dragging || m_cameraMode != CameraMode::Orbit) return;
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
    // BackView has a fixed camera position — zoom has no meaning there
    if (m_cameraMode == CameraMode::BackView) { e->ignore(); return; }

    int delta = e->angleDelta().y();
    if (delta == 0) delta = e->pixelDelta().y();   // trackpad fallback
    if (delta == 0) { e->ignore(); return; }

    float factor = delta > 0 ? 0.88f : 1.14f;
    m_distance   = qBound(0.5f, m_distance * factor, 10000.0f);
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

        // attrib 2: UV (tu,tv)
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE,
                              sizeof(OBJVertex),
                              (void*)offsetof(OBJVertex, tu));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);

        // Upload diffuse texture if available
        if (!data.texturePath.isEmpty()) {
            QImage img(data.texturePath);
            if (!img.isNull()) {
                img = img.convertToFormat(QImage::Format_RGBA8888).mirrored();
                glGenTextures(1, &mesh.texId);
                glBindTexture(GL_TEXTURE_2D, mesh.texId);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                             img.width(), img.height(), 0,
                             GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());
                glGenerateMipmap(GL_TEXTURE_2D);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glBindTexture(GL_TEXTURE_2D, 0);
                Logger::instance().info(
                    QString("ModelGLView: loaded texture for '%1'").arg(data.name));
            } else {
                Logger::instance().warning(
                    QString("ModelGLView: failed to load texture '%1'").arg(data.texturePath));
            }
        }

        m_meshes.append(mesh);
    }

    // Build ground grid at the model's floor level
    setupGrid(sMin.y());

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

    bool hasTex = (mesh.texId != 0) && !m_colorOverrides.contains(mesh.name);
    m_program->setUniformValue("uHasTex", hasTex);
    if (hasTex) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mesh.texId);
        m_program->setUniformValue("uTex", 0);
    }

    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
    glBindVertexArray(0);

    if (hasTex)
        glBindTexture(GL_TEXTURE_2D, 0);
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

void ModelGLView::uploadTrail()
{
    if (!m_trailVao) return;   // GL not ready yet
    makeCurrent();
    glBindBuffer(GL_ARRAY_BUFFER, m_trailVbo);
    glBufferData(GL_ARRAY_BUFFER,
                 (GLsizeiptr)(m_trailPts.size() * sizeof(QVector3D)),
                 m_trailPts.isEmpty() ? nullptr : m_trailPts.constData(),
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    doneCurrent();
    m_trailDirty = false;
}

void ModelGLView::setupGrid(float groundY)
{
    // Delete any previous grid buffers
    if (m_gridVao) { glDeleteVertexArrays(1, &m_gridVao); m_gridVao = 0; }
    if (m_gridVbo) { glDeleteBuffers(1, &m_gridVbo); m_gridVbo = 0; }

    // Fixed 1 m cells, ±60 m coverage.  The grid is rendered with an XZ
    // translation snapped to the nearest cell in paintGL() so it appears
    // to extend infinitely as the robot/camera moves.
    constexpr float cell = 1.0f;
    constexpr int   half = 60;
    constexpr float ext  = half * cell;

    m_gridGroundY = groundY - 0.001f;   // just below the model's feet
    const float Y = m_gridGroundY;

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
