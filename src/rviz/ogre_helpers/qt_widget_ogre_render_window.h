#ifndef QTWIDGETOGRERENDERWINDOW_H
#define QTWIDGETOGRERENDERWINDOW_H

#include <QPaintEngine>
#include <QWidget>

#include "qt_ogre_render_window.h"

namespace rviz
{

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
  virtual ~QtWidgetOgreRenderWindow();

  /** Overrides the default implementation.
    This override is here for convenience. Returns a symbolic 320x240px size.
    @return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const { return QSize( 320, 240 ); }

  virtual void setFocus(Qt::FocusReason reason);
  virtual QPoint mapFromGlobal(const QPoint &point) const;
  virtual QPoint mapToGlobal(const QPoint &point) const;
  virtual void setCursor(const QCursor &cursor);
  virtual bool containsPoint(const QPoint &point) const;
  virtual double getWindowPixelRatio() const;

  virtual void keyPressEvent( QKeyEvent* event);
  virtual void wheelEvent( QWheelEvent* event);
  virtual void leaveEvent( QEvent* event);
  virtual void mouseMoveEvent( QMouseEvent* event);
  virtual void mousePressEvent( QMouseEvent* event);
  virtual void mouseReleaseEvent( QMouseEvent* event);
  virtual void mouseDoubleClickEvent( QMouseEvent* event);

  virtual QRect rect() const;

protected:
  virtual void moveEvent( QMoveEvent *event );
  virtual void paintEvent( QPaintEvent* event );
  virtual void resizeEvent( QResizeEvent* event );

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  virtual QPaintEngine *paintEngine() const { return 0; }
#endif

  virtual void updateScene();
};

} // namespace rviz

#endif // QTWIDGETOGRERENDERWINDOW_H
