#ifndef QTWIDGETOGRERENDERWINDOW_H
#define QTWIDGETOGRERENDERWINDOW_H
#include "qt_ogre_render_window.h"
#include "render_widget.h"

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
    @param parent The parent wxWindow component.
   */
  QtWidgetOgreRenderWindow( QWidget* parent = 0 );

  /** Destructor.  */
  virtual ~QtWidgetOgreRenderWindow();

  /** Overrides the default implementation.
    This override is here for convenience. Returns a symbolic 320x240px size.
    @return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const { return QSize( 320, 240 ); }

  void setFocus(Qt::FocusReason reason);
  QPoint mapFromGlobal(const QPoint &point) const;
  QPoint mapToGlobal(const QPoint &point) const;
  void setCursor(const QCursor &cursor);

  void keyPressEvent( QKeyEvent* event);
  void wheelEvent( QWheelEvent* event);
  void leaveEvent( QEvent* event);

  QRect rect() const;
  void updateScene();

protected:
  virtual void moveEvent( QMoveEvent *event );
  virtual void paintEvent( QPaintEvent* event );
  virtual void resizeEvent( QResizeEvent* event );

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
  QPaintEngine *paintEngine() const { return 0; }
#endif

  RenderSystem* render_system_;

  QFrame* render_frame;

};

} // namespace rviz

#endif // QTWIDGETOGRERENDERWINDOW_H
