#include "OBJLoader.h"
#include "Logger.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <array>
#include <QtMath>

OBJLoader::OBJLoader(const QString &path, QObject *parent)
    : QThread(parent), m_path(path) {}

OBJLoader::~OBJLoader()
{
    wait();
}

// Parse one face token (vi, vi/vt, vi//vn, vi/vt/vn) advancing *pp.
// Returns {posIndex, normalIndex} as 1-based, 0 = absent.
static std::array<int,2> parseFaceToken(const char **pp)
{
    const char *p = *pp;
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p || *p == '\n' || *p == '\r') { *pp = p; return {0, 0}; }

    int vi = atoi(p);
    while (*p && *p != '/' && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') ++p;

    int vn = 0;
    if (*p == '/') {
        ++p;
        if (*p != '/') {
            // skip texture index
            while (*p && *p != '/' && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') ++p;
        }
        if (*p == '/') {
            ++p;
            vn = atoi(p);
            while (*p && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') ++p;
        }
    }

    *pp = p;
    return {vi, vn};
}

void OBJLoader::run()
{
    FILE *f = fopen(m_path.toLocal8Bit().constData(), "r");
    if (!f) {
        emit loadFailed("Cannot open: " + m_path);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fileSize = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fileSize <= 0) {
        fclose(f);
        emit loadFailed("Empty or unreadable file");
        return;
    }

    // Raw attribute arrays (std::vector for minimal overhead during parse)
    std::vector<std::array<float,3>> positions;
    std::vector<std::array<float,3>> normals;
    positions.reserve(600000);
    normals.reserve(300000);

    QVector<OBJMeshData> meshes;
    meshes.reserve(64);
    int currentIdx = -1;

    long bytesRead = 0;
    int lastPct = -1;

    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        bytesRead += (long)strlen(line);

        int pct = (int)((bytesRead * 100LL) / fileSize);
        if (pct >= lastPct + 5) {
            lastPct = pct;
            emit progress(pct);
        }

        const char *p = line;
        while (*p == ' ' || *p == '\t') ++p;

        if (p[0] == 'v' && p[1] == ' ') {
            float x = 0, y = 0, z = 0;
            sscanf(p + 2, "%f %f %f", &x, &y, &z);
            positions.push_back({x, y, z});
        }
        else if (p[0] == 'v' && p[1] == 'n' && p[2] == ' ') {
            float x = 0, y = 0, z = 0;
            sscanf(p + 3, "%f %f %f", &x, &y, &z);
            normals.push_back({x, y, z});
        }
        else if (p[0] == 'o' && p[1] == ' ') {
            char name[256] = {};
            sscanf(p + 2, "%255s", name);
            meshes.append(OBJMeshData());
            meshes.last().name = QString::fromUtf8(name);
            currentIdx = meshes.size() - 1;
        }
        else if (p[0] == 'f' && p[1] == ' ') {
            if (currentIdx < 0) {
                meshes.append(OBJMeshData());
                meshes.last().name = QStringLiteral("default");
                currentIdx = 0;
            }

            // Collect all face vertices
            std::array<int,2> fverts[16];
            int fcount = 0;
            const char *fp = p + 2;
            while (fcount < 16) {
                auto tok = parseFaceToken(&fp);
                if (tok[0] == 0) break;
                fverts[fcount++] = tok;
            }
            if (fcount < 3) continue;

            auto &verts = meshes[currentIdx].vertices;
            const std::array<float,3> fallbackNorm = {0.0f, 1.0f, 0.0f};

            // Fan triangulate: (0,1,2), (0,2,3), ...
            for (int i = 1; i + 1 < fcount; ++i) {
                const std::array<int,2> tri[3] = { fverts[0], fverts[i], fverts[i+1] };
                for (int j = 0; j < 3; ++j) {
                    int pi = tri[j][0] - 1;
                    int ni = tri[j][1] - 1;
                    const auto &pos = (pi >= 0 && pi < (int)positions.size())
                                      ? positions[pi] : std::array<float,3>{0,0,0};
                    const auto &nor = (ni >= 0 && ni < (int)normals.size())
                                      ? normals[ni] : fallbackNorm;
                    verts.append({pos[0], pos[1], pos[2], nor[0], nor[1], nor[2]});
                }
            }
        }
    }
    fclose(f);

    // Compute per-mesh centroid and bounds; drop empty meshes
    QVector<OBJMeshData> result;
    result.reserve(meshes.size());

    for (auto &mesh : meshes) {
        if (mesh.vertices.isEmpty()) continue;

        QVector3D sum;
        QVector3D bmin( 1e9f,  1e9f,  1e9f);
        QVector3D bmax(-1e9f, -1e9f, -1e9f);

        for (const auto &v : mesh.vertices) {
            sum  += QVector3D(v.px, v.py, v.pz);
            bmin  = QVector3D(qMin(bmin.x(), v.px), qMin(bmin.y(), v.py), qMin(bmin.z(), v.pz));
            bmax  = QVector3D(qMax(bmax.x(), v.px), qMax(bmax.y(), v.py), qMax(bmax.z(), v.pz));
        }

        mesh.centroid   = sum / float(mesh.vertices.size());
        mesh.boundsMin  = bmin;
        mesh.boundsMax  = bmax;
        result.append(std::move(mesh));
    }

    Logger::instance().info(
        QString("OBJLoader: loaded %1 meshes from %2").arg(result.size()).arg(m_path));

    emit progress(100);
    emit loadFinished(result);
}
