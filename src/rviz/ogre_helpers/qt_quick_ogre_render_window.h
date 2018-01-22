#ifndef QT_QUICK_OGRE_RENDER_WINDOW_H
#define QT_QUICK_OGRE_RENDER_WINDOW_H
#include "qt_ogre_render_window.h"

#include <QQuickItem>
#include <QOpenGLContext>

namespace rviz
{

class QtQuickOgreRenderWindow : public QQuickItem, public QtOgreRenderWindow
{
  Q_OBJECT

public:
  /** Constructor.
    @param parent The parent component.
   */
  QtQuickOgreRenderWindow( QQuickItem *parent = Q_NULLPTR );

  /** Destructor.  */
  virtual ~QtQuickOgreRenderWindow();

  void setFocus(Qt::FocusReason reason);
  QPoint mapFromGlobal(const QPoint &point) const;
  QPoint mapToGlobal(const QPoint &point) const;
  void setCursor(const QCursor &cursor);

  QRect rect() const;

protected:
  void updateScene();

private:
  void initializeOgre();
  void render();

  void activateOgreContext();
  void doneOgreContext();

  void onWindowChanged(QQuickWindow *window);

  QOpenGLContext* ogre_gl_context_;
  QOpenGLContext* qt_gl_context_;

Q_SIGNALS:
  void ogreInitialized();
};

} // namespace rviz

#endif // QT_QUICK_OGRE_RENDER_WINDOW_H
