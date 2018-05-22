#include "qt_widget_ogre_render_window.h"
#include "orthographic.h"
#include "render_system.h"

#include <QApplication>
#include <QWindow>

#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreStringConverter.h>
#include <OgreGpuProgramManager.h>
#include <OgreRenderTargetListener.h>

#include <ros/console.h>
#include <ros/assert.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
#include <stdlib.h>
#endif

namespace rviz
{

class RenderSystem;

QtWidgetOgreRenderWindow::QtWidgetOgreRenderWindow( QWidget* parent )
  : QWidget( parent )
  , render_system_( RenderSystem::get() )
  , ogre_root_( RenderSystem::get()->root() )
{
  setAttribute(Qt::WA_OpaquePaintEvent,true);
  setAttribute(Qt::WA_PaintOnScreen,true);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  // It is not clear to me why, but having this frame sub-widget
  // inside the main widget makes an important difference (under X at
  // least).  Without the frame and using this widget's winId()
  // below causes trouble when using RenderWidget as a child
  // widget.  The frame graphics are completely covered up by the 3D
  // render, so using it does not affect the appearance at all.
  QFrame* render_frame = new QFrame(this);
  render_frame->setLineWidth(1);
  render_frame->setFrameShadow(QFrame::Sunken);
  render_frame->setFrameShape(QFrame::Box);
  render_frame->show();

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins( 0, 0, 0, 0 );
  main_layout->addWidget(this->renderFrame);
  this->setLayout(main_layout);
#endif

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  RenderSystem::WindowIDType win_id = render_frame->winId();
#else
  RenderSystem::WindowIDType win_id = this->winId();
#endif
  QApplication::flush();

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  QApplication::syncX();
  double pixel_ratio = 1.0;
#else
  QWindow* window = windowHandle();
  double pixel_ratio = window ? window->devicePixelRatio() : 1.0;
#endif
  render_window_ = render_system_->makeRenderWindow(win_id, static_cast<quint32>(width()), static_cast<quint32>(height()), pixel_ratio);
  render_window_->setVisible(true);
  render_window_->setAutoUpdated(true);

  QtOgreRenderWindow::initialize();
}

QtWidgetOgreRenderWindow::~QtWidgetOgreRenderWindow()
{
}

void QtWidgetOgreRenderWindow::moveEvent(QMoveEvent *event)
{
  QWidget::moveEvent(event);

  if(event->isAccepted() && render_window_)
  {
    render_window_->windowMovedOrResized();
  }
}

//------------------------------------------------------------------------------
void QtWidgetOgreRenderWindow::paintEvent( QPaintEvent* )
{
  if( auto_render_ && render_window_ )
  {
    if( pre_render_callback_ )
    {
      pre_render_callback_();
    }

    if( ogre_root_->_fireFrameStarted() )
    {
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
      ogre_root_->_fireFrameRenderingQueued();
#endif

      render_window_->update();

      ogre_root_->_fireFrameEnded();
    }

    if ( post_render_callback_ )
    {
      post_render_callback_();
    }
  }
}

//------------------------------------------------------------------------------
void QtWidgetOgreRenderWindow::resizeEvent( QResizeEvent* )
{
  if( render_window_ )
  {
    // render_window_->writeContentsToFile() (used in
    // VisualizationFrame::onSaveImage()) does not work right for
    // window with an odd width, so here I just always force it to be
    // even.
    render_window_->resize( static_cast<quint32>(width() + (width() % 2)), static_cast<quint32>(height()) );
    render_window_->windowMovedOrResized();
  }

  if( render_window_ )
  {
    setCameraAspectRatio();

    if( auto_render_ )
    {
      updateScene();
    }
  }
}

void QtWidgetOgreRenderWindow::setFocus( Qt::FocusReason reason )
{
  QWidget::setFocus( reason );
}

QPoint QtWidgetOgreRenderWindow::mapFromGlobal( const QPoint &point ) const
{
  return QWidget::mapFromGlobal( point );
}

QPoint QtWidgetOgreRenderWindow::mapToGlobal( const QPoint &point ) const
{
  return QWidget::mapToGlobal( point );
}

void QtWidgetOgreRenderWindow::setCursor( const QCursor &cursor )
{
  QWidget::setCursor( cursor );
}

bool QtWidgetOgreRenderWindow::containsPoint(const QPoint &point) const
{
  QWidget *w = QApplication::widgetAt( point );
  while( w )
  {
    if( w == this )
    {
      return true;
    }
    w = w->parentWidget();
  }

  return false;
}

double QtWidgetOgreRenderWindow::getWindowPixelRatio() const
{
  return windowHandle()->devicePixelRatio();
}

bool QtWidgetOgreRenderWindow::isVisible() const {
  return QWidget::isVisible();
}

void QtWidgetOgreRenderWindow::keyPressEvent( QKeyEvent *event )
{
  emitKeyPressEvent( event );
}

void QtWidgetOgreRenderWindow::wheelEvent( QWheelEvent *event )
{
  emitWheelEvent( event );
}

void QtWidgetOgreRenderWindow::leaveEvent( QEvent *event )
{
  emitLeaveEvent( event );
}

void QtWidgetOgreRenderWindow::mouseMoveEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtWidgetOgreRenderWindow::mousePressEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtWidgetOgreRenderWindow::mouseReleaseEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtWidgetOgreRenderWindow::mouseDoubleClickEvent(QMouseEvent *event)
{
  emitMouseEvent( event );
}

void QtWidgetOgreRenderWindow::contextMenuEvent(QContextMenuEvent *event)
{
  emitContextMenuEvent( event );
}

QRect QtWidgetOgreRenderWindow::rect() const
{
  return QWidget::rect();
}

void QtWidgetOgreRenderWindow::updateScene()
{
  QWidget::update();
}

} // namespace rviz
