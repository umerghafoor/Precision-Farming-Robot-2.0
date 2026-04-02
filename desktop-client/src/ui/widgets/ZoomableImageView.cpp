#include "ZoomableImageView.h"

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

namespace {
static constexpr double kMinZoom = 0.25;
static constexpr double kMaxZoom = 8.0;
static constexpr double kZoomStep = 1.25;
}

ZoomableImageView::ZoomableImageView(QWidget *parent)
    : QWidget(parent)
    , m_zoomFactor(1.0)
    , m_panOffset(0.0, 0.0)
    , m_dragging(false)
{
    setMinimumSize(200, 150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setCursor(Qt::ArrowCursor);
}

void ZoomableImageView::setPixmap(const QPixmap& pixmap)
{
    m_pixmap = pixmap;
    update();
}

void ZoomableImageView::zoomIn()
{
    setZoomFactor(m_zoomFactor * kZoomStep);
}

void ZoomableImageView::zoomOut()
{
    setZoomFactor(m_zoomFactor / kZoomStep);
}

void ZoomableImageView::resetView()
{
    m_zoomFactor = 1.0;
    m_panOffset = QPointF(0.0, 0.0);
    m_dragging = false;
    setCursor(Qt::ArrowCursor);
    update();
}

void ZoomableImageView::setZoomFactor(double zoomFactor)
{
    double clamped = qBound(kMinZoom, zoomFactor, kMaxZoom);
    if (qFuzzyCompare(m_zoomFactor, clamped)) {
        return;
    }

    m_zoomFactor = clamped;
    if (m_zoomFactor <= 1.0) {
        m_panOffset = QPointF(0.0, 0.0);
    }
    update();
}

QPointF ZoomableImageView::contentTopLeft() const
{
    if (m_pixmap.isNull()) {
        return QPointF();
    }

    QSizeF scaledSize = QSizeF(m_pixmap.size()) * m_zoomFactor;
    QPointF centered((width() - scaledSize.width()) / 2.0,
                     (height() - scaledSize.height()) / 2.0);
    return centered + m_panOffset;
}

void ZoomableImageView::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.fillRect(rect(), Qt::black);

    if (m_pixmap.isNull()) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "No Stream");
        return;
    }

    QSizeF scaledSize = QSizeF(m_pixmap.size()) * m_zoomFactor;
    QPointF topLeft = contentTopLeft();
    QRectF target(topLeft, scaledSize);
    painter.drawPixmap(target, m_pixmap, QRectF(m_pixmap.rect()));
}

void ZoomableImageView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_zoomFactor > 1.0 && !m_pixmap.isNull()) {
        m_dragging = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }

    QWidget::mousePressEvent(event);
}

void ZoomableImageView::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_dragging) {
        QWidget::mouseMoveEvent(event);
        return;
    }

    QPoint delta = event->pos() - m_lastMousePos;
    m_lastMousePos = event->pos();
    m_panOffset += QPointF(delta);
    update();
    event->accept();
}

void ZoomableImageView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && m_dragging) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }

    QWidget::mouseReleaseEvent(event);
}

void ZoomableImageView::wheelEvent(QWheelEvent *event)
{
    int delta = event->angleDelta().y();
    if (delta == 0) {
        delta = event->pixelDelta().y();
    }
    if (delta == 0) {
        event->ignore();
        return;
    }

    if (delta > 0) {
        zoomIn();
    } else {
        zoomOut();
    }
    event->accept();
}