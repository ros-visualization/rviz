#ifndef QT_QUICK_OGRE_RENDER_WINDOW_H
#define QT_QUICK_OGRE_RENDER_WINDOW_H

#include <Ogre.h>

#include <QQuickItem>
#include <QSGGeometry>
#include <QSGTextureMaterial>
#include <QTimer>

#include <OgreFrameListener.h>

#include "qt_ogre_render_window.h"

class QOpenGLContext;

namespace rviz
{

class QtQuickOgreRenderWindow : public QQuickItem, public QtOgreRenderWindow, public Ogre::FrameListener
{
  Q_OBJECT

public:
  /** Constructor.
    @param parent The parent component.
   */
  QtQuickOgreRenderWindow( QQuickItem *parent = Q_NULLPTR );

  /** Destructor.  */
  virtual ~QtQuickOgreRenderWindow();

  virtual void setFocus(Qt::FocusReason reason);
  virtual QPoint mapFromGlobal(const QPoint &point) const;
  virtual QPoint mapToGlobal(const QPoint &point) const;
  virtual void setCursor(const QCursor &cursor);
  virtual bool containsPoint(const QPoint &point) const;
  virtual double getWindowPixelRatio() const;

  virtual QRect rect() const;

  virtual void keyPressEvent( QKeyEvent* event);
  virtual void wheelEvent( QWheelEvent* event);
  virtual void mouseMoveEvent( QMouseEvent* event);
  virtual void mousePressEvent( QMouseEvent* event);
  virtual void mouseReleaseEvent( QMouseEvent* event);
  virtual void mouseDoubleClickEvent( QMouseEvent* event);

Q_SIGNALS:
  void ogreInitializing();
  void ogreInitialized();

protected:
  virtual void updateScene();

  virtual QSGNode *updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *);

  virtual bool frameStarted(const Ogre::FrameEvent&);

  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent&);
  virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent&);

  void updateFBO();

private:
  void initializeOgre();
  void render();

  void activateOgreContext();
  void doneOgreContext();

  void onWindowChanged(QQuickWindow* window);

  bool initialized_;
  QOpenGLContext* ogre_gl_context_;
  QOpenGLContext* qt_gl_context_;

  QSGTextureMaterial       material_;
  QSGOpaqueTextureMaterial material_opaque_;
  QSGGeometry              geometry_;
  QSize                    size_;
  QSGTexture              *texture_;
  Ogre::RenderTexture     *render_target_;

  QTimer update_timer_;

};

} // namespace rviz

#endif // QT_QUICK_OGRE_RENDER_WINDOW_H
