#include "qt_quick_ogre_render_window_widget.h"

#include <QQmlEngine>
#include <QQmlComponent>
#include <QQmlContext>

#include <ros/console.h>

#include "qt_quick_ogre_render_window.h"

namespace rviz {

QtQuickOgreRenderWindowWidget::QtQuickOgreRenderWindowWidget(QWidget *parent)
  : QQuickWidget(parent)
  , render_window_(nullptr)
{
  setResizeMode(SizeViewToRootObject);
  createOgreRenderWindow();
}

QtOgreRenderWindow *QtQuickOgreRenderWindowWidget::getRenderWindow() const
{
  return render_window_;
}

void QtQuickOgreRenderWindowWidget::createOgreRenderWindow()
{
  QQmlComponent component(engine());

  component.setData("import QtQuick 2.0\n"
                    "Rectangle { color: \"red\"}", QUrl());
  auto item = qobject_cast<QQuickItem*>(component.create(rootContext()));
  auto render_window = new QtQuickOgreRenderWindow(item);
  render_window_ = render_window;

  // anchors.fill: parent
  qvariant_cast<QObject*>(
      render_window->property("anchors")
  )->setProperty("fill", QVariant::fromValue(item));
}

} // namespace rviz

