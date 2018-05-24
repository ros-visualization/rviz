#ifndef QT_QUICK_OGRE_RENDER_WINDOW_H
#define QT_QUICK_OGRE_RENDER_WINDOW_H

#include <QSGGeometry>
#include <QSGTextureMaterial>
#include <QTimer>
#include <QQuickItem>

#include <OgreFrameListener.h>

#include "qt_ogre_render_window.h"
#include <QMetaObject>

class QOpenGLContext;

namespace rviz
{
  class RenderSystem;

class QtQuickOgreRenderWindow : public QQuickItem, public QtOgreRenderWindow, public Ogre::FrameListener
{
  Q_OBJECT

public:
  /** Constructor.
    @param parent The parent component.
   */
  explicit QtQuickOgreRenderWindow( QQuickItem *parent = nullptr );

  /** Destructor.  */
  virtual ~QtQuickOgreRenderWindow() override;

  virtual void setFocus(Qt::FocusReason reason) override;
  virtual QPoint mapFromGlobal(const QPoint &point) const override;
  virtual QPoint mapToGlobal(const QPoint &point) const override;
  virtual void setCursor(const QCursor &cursor) override;
  virtual bool containsPoint(const QPoint &point) const override;
  virtual double getWindowPixelRatio() const override;
  virtual bool isVisible() const override;

  virtual QRect rect() const override;

  virtual void keyPressEvent( QKeyEvent* event) override;
  virtual void wheelEvent( QWheelEvent* event) override;
  virtual void mouseMoveEvent( QMouseEvent* event) override;
  virtual void mousePressEvent( QMouseEvent* event) override;
  virtual void mouseReleaseEvent( QMouseEvent* event) override;
  virtual void mouseDoubleClickEvent( QMouseEvent* event) override;

Q_SIGNALS:
  void ogreInitializing();
  void ogreInitialized();

protected:
  virtual void updateScene() override;

  virtual QSGNode *updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *) override;

  virtual bool frameStarted(const Ogre::FrameEvent&) override;

  virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent&) override;
  virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent&) override;

  void updateFBO();

private:
  void initializeOgre();
  void render();

  void activateOgreContext();
  void doneOgreContext();

  void onWindowChanged(QQuickWindow* window);

  RenderSystem *render_system_;
  Ogre::Root *ogre_root_;

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
