#ifndef QT_QUICK_OGRE_RENDER_WINDOW_WIDGET_H
#define QT_QUICK_OGRE_RENDER_WINDOW_WIDGET_H

#include <QObject>
#include <QQuickWidget>

#include "qt_ogre_render_window.h"

namespace rviz {

class QtQuickOgreRenderWindowWidget : public QQuickWidget
{
public:
    QtQuickOgreRenderWindowWidget(QWidget* parent = Q_NULLPTR);

    QtOgreRenderWindow* getRenderWindow() const;

private:
    void createOgreRenderWindow();

    QtOgreRenderWindow* render_window_;
};

} // namespace rviz

#endif // QT_QUICK_OGRE_RENDER_WINDOW_WIDGET_H
