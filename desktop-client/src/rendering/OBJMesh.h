#pragma once
#include <QString>
#include <QVector>
#include <QVector3D>

// Single vertex: position + normal, interleaved for VBO upload
struct OBJVertex {
    float px, py, pz;
    float nx, ny, nz;
};

// Per-object mesh produced by OBJLoader (flat triangles, no index buffer)
struct OBJMeshData {
    QString           name;
    QVector<OBJVertex> vertices;   // 3 vertices per triangle
    QVector3D         centroid;
    QVector3D         boundsMin;
    QVector3D         boundsMax;
};
