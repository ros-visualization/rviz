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

QtWidgetOgreRenderWindow::QtWidgetOgreRenderWindow( QWidget* parent )
  : QWidget( parent )
  , render_system_( RenderSystem::get() )
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
  this->renderFrame = new QFrame;
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setContentsMargins( 0, 0, 0, 0 );
  mainLayout->addWidget(this->renderFrame);
  this->setLayout(mainLayout);
#endif

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  rviz::RenderSystem::WindowIDType win_id = this->renderFrame->winId();
#else
  rviz::RenderSystem::WindowIDType win_id = this->winId();
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

  OgreViewportSupport::initialize();
}

QtWidgetOgreRenderWindow::~QtWidgetOgreRenderWindow()
{
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

QRect QtWidgetOgreRenderWindow::rect() const
{
  return QWidget::rect();
}

void QtWidgetOgreRenderWindow::updateScene()
{
  QWidget::update();
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

} // namespace rviz
