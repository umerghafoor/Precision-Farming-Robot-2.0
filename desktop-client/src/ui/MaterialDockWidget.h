#ifndef MATERIALDOCKWIDGET_H
#define MATERIALDOCKWIDGET_H

#include <QDockWidget>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>
#include <QWidget>

/**
 * @brief M3-styled QDockWidget with a custom title bar.
 *
 * Replaces the native QDockWidget chrome with a Material You title bar:
 *   - drag grip indicator
 *   - Title Small label (14sp, medium weight)
 *   - float and close icon buttons (M3 icon button style)
 *
 * Drop-in replacement for QDockWidget. objectName and all Qt state
 * serialization (saveState/restoreState) are unaffected.
 */
class MaterialDockWidget : public QDockWidget
{
    Q_OBJECT

public:
    explicit MaterialDockWidget(const QString& title, QWidget* parent = nullptr);
    ~MaterialDockWidget() override = default;

    void setCardTitle(const QString& title);

private slots:
    void onFloatClicked();
    void onCloseClicked();

private:
    void buildTitleBar(const QString& title);

    QWidget*     m_titleBar   = nullptr;
    QLabel*      m_titleLabel = nullptr;
    QToolButton* m_floatButton = nullptr;
    QToolButton* m_closeButton = nullptr;
};

#endif // MATERIALDOCKWIDGET_H
