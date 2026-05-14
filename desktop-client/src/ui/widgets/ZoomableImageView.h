#ifndef ZOOMABLEIMAGEVIEW_H
#define ZOOMABLEIMAGEVIEW_H

#include <QWidget>
#include <QPixmap>

/**
 * @brief Image view that supports zooming and mouse dragging.
 */
class ZoomableImageView : public QWidget
{
    Q_OBJECT

public:
    explicit ZoomableImageView(QWidget *parent = nullptr);

    void setPixmap(const QPixmap& pixmap);
    void zoomIn();
    void zoomOut();
    void resetView();
    double zoomFactor() const { return m_zoomFactor; }

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void setZoomFactor(double zoomFactor);
    QPointF contentTopLeft() const;

    QPixmap m_pixmap;
    double m_zoomFactor;
    QPointF m_panOffset;
    bool m_dragging;
    QPoint m_lastMousePos;
};

#endif // ZOOMABLEIMAGEVIEW_H