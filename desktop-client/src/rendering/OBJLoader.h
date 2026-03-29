#pragma once
#include "OBJMesh.h"
#include <QThread>

/**
 * @brief Parses an OBJ file on a background thread.
 *
 * Emits progress(percent) every ~5% and loadFinished(meshes) when done.
 * One OBJMeshData is produced per `o` declaration in the file.
 * Faces are fan-triangulated and deindexed into flat vertex arrays.
 */
class OBJLoader : public QThread
{
    Q_OBJECT
public:
    explicit OBJLoader(const QString &path, QObject *parent = nullptr);
    ~OBJLoader() override;

signals:
    void progress(int percent);
    void loadFinished(QVector<OBJMeshData> meshes);
    void loadFailed(const QString &reason);

protected:
    void run() override;

private:
    QString m_path;
};
