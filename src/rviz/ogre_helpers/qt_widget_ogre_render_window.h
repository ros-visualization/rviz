#ifndef QTWIDGETOGRERENDERWINDOW_H
#define QTWIDGETOGRERENDERWINDOW_H

#include <QPaintEngine>
#include <QWidget>

#include "render_system.h"

#include "qt_ogre_render_window.h"

namespace rviz
{
  class RenderSystem;

/**
 * Qt Ogre render window widget.  Similar in API to
 *  wxOgreRenderWindow from ogre_tools release 1.6, but with much of
 *  the guts replaced by new RenderSystem and RenderWidget classes
 *  inspired by the initialization sequence of Gazebo's renderer.
 */
class QtWidgetOgreRenderWindow : public QWidget, public QtOgreRenderWindow {
    Q_OBJECT

public:
  /** Constructor.
    @param parent The parent component.
   */
  QtWidgetOgreRenderWindow( QWidget* parent = Q_NULLPTR );

  /** Destructor.  */
  virtual ~QtWidgetOgreRenderWindow() override;

  /** Overrides the default implementation.
    This override is here for convenience. Returns a symbolic 320x240px size.
    @return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const override { return QSize( 320, 240 ); }

  virtual void setFocus(Qt::FocusReason reason) override;
  virtual QPoint mapFromGlobal(const QPoint &point) const override;
  virtual QPoint mapToGlobal(const QPoint &point) const override;
  virtual void setCursor(const QCursor &cursor) override;
  virtual bool containsPoint(const QPoint &point) const override;
  virtual double getWindowPixelRatio() const override;
  virtual bool isVisible() const override;

  virtual void keyPressEvent( QKeyEvent* event) override;
  virtual void wheelEvent( QWheelEvent* event) override;
  virtual void leaveEvent( QEvent* event) override;
  virtual void mouseMoveEvent( QMouseEvent* event) override;
  virtual void mousePressEvent( QMouseEvent* event) override;
  virtual void mouseReleaseEvent( QMouseEvent* event) override;
  virtual void mouseDoubleClickEvent( QMouseEvent* event) override;
  virtual void contextMenuEvent( QContextMenuEvent* event) override;

  virtual QRect rect() const override;

protected:
  virtual void moveEvent( QMoveEvent *event ) override;
  virtual void paintEvent( QPaintEvent* event ) override;
  virtual void resizeEvent( QResizeEvent* event ) override;

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  virtual QPaintEngine *paintEngine() const override { return 0; }
#endif

  virtual void updateScene() override;

protected:
  RenderSystem *render_system_;
  Ogre::Root *ogre_root_;
};

} // namespace rviz

#endif // QTWIDGETOGRERENDERWINDOW_H
