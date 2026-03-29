#ifndef STATUSBADGE_H
#define STATUSBADGE_H

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QFrame>

/**
 * @brief Simple status badge widget showing a colored dot and text.
 *
 * Used in the title/menu bar to display connection or simulation state.
 */
class StatusBadge : public QWidget
{
    Q_OBJECT

public:
    explicit StatusBadge(const QString &text = QString(), QWidget *parent = nullptr);

    void setText(const QString &text);
    void setDotColor(const QColor &color);

private:
    QFrame *m_dot;
    QLabel *m_label;
};

#endif // STATUSBADGE_H
