/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QApplication>
#include <QMenu>
#include <QTimer>
#include <QBoxLayout>
#include <QWindow>

#include <OgreSceneManager.h>
#include <OgreCamera.h>

#include "ogre_helpers/qt_widget_ogre_render_window.h"

#include "rviz/display.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"

#include "rviz/render_panel.h"

namespace rviz
{

RenderPanel::RenderPanel(QtOgreRenderWindow *render_window, QObject *parent )
  : QObject ( parent )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  , context_( nullptr )
  , scene_manager_( nullptr )
  , view_controller_( nullptr )
  , context_menu_visible_(false)
  , fake_mouse_move_event_timer_( new QTimer() )
  , default_camera_( nullptr )
  , render_window_( render_window )
{
  render_window_->setFocus( Qt::OtherFocusReason );
  render_window_->setKeyPressEventCallback([this] (QKeyEvent* event) { this->onKeyPressEvent(event); });
  render_window_->setWheelEventCallback([this] (QWheelEvent* event) { this->onWheelEvent(event); });
  render_window_->setLeaveEventCallack([this] (QEvent* event) { this->onLeaveEvent(event); });
  render_window_->setMouseEventCallback([this] (QMouseEvent *event) { this->onRenderWindowMouseEvents(event); });
  render_window_->setContextMenuEvent([this] (QContextMenuEvent *event) { this->onContextMenuEvent(event); });
}

RenderPanel::~RenderPanel()
{
  fake_mouse_move_event_timer_->deleteLater();

  if( scene_manager_ && default_camera_ )
  {
    scene_manager_->destroyCamera( default_camera_ );
  }
  if( scene_manager_ )
  {
    scene_manager_->removeListener( this );
  }
  scene_manager_ = nullptr;
}

void RenderPanel::initialize(Ogre::SceneManager* scene_manager, DisplayContext* context)
{
  context_ = context;
  scene_manager_ = scene_manager;
  scene_manager_->addListener( this );

  std::stringstream ss;
  static int count = 0;
  ss << "RenderPanelCamera" << count++;
  default_camera_ = scene_manager_->createCamera(ss.str());
  default_camera_->setNearClipDistance(0.01f);
  default_camera_->setPosition(0, 10, 15);
  default_camera_->lookAt(0, 0, 0);

  render_window_->setCamera( default_camera_ );

  connect( fake_mouse_move_event_timer_, SIGNAL( timeout() ), this, SLOT( sendMouseMoveEvent() ));
  fake_mouse_move_event_timer_->start( 33 /*milliseconds*/ );
}

void RenderPanel::sendMouseMoveEvent()
{
  QPoint cursor_pos = QCursor::pos();
  QPoint mouse_rel_widget = render_window_->mapFromGlobal( cursor_pos );
  if( render_window_->rect().contains( mouse_rel_widget ) && render_window_->isVisible() )
  {
    if( !render_window_->containsPoint(mouse_rel_widget) )
    {
      return;
    }

    QMouseEvent fake_event( QEvent::MouseMove,
                            mouse_rel_widget,
                            Qt::NoButton,
                            QApplication::mouseButtons(),
                            QApplication::keyboardModifiers() );
    onRenderWindowMouseEvents( &fake_event );
  }
}
void RenderPanel::onLeaveEvent ( QEvent * )
{
  render_window_->setCursor( Qt::ArrowCursor );
  if ( context_ )
  {
    context_->setStatus("");
  }
}

void RenderPanel::onRenderWindowMouseEvents( QMouseEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_ && render_window_->isVisible())
  {
    render_window_->setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, render_window_->getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::onWheelEvent( QWheelEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_ && render_window_->isVisible())
  {
    render_window_->setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, render_window_->getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::onKeyPressEvent( QKeyEvent* event )
{
  if( context_ && render_window_->isVisible() )
  {
    context_->handleChar( event, this );
  }
}

void RenderPanel::setViewController( ViewController* controller )
{
  view_controller_ = controller;

  if( view_controller_ )
  {
    render_window_->setCamera( view_controller_->getCamera() );
    view_controller_->activate();
  }
  else
  {
    render_window_->setCamera( nullptr );
  }
}

void RenderPanel::showContextMenu( boost::shared_ptr<QMenu> menu )
{
  boost::mutex::scoped_lock lock(context_menu_mutex_);
  context_menu_ = menu;
  context_menu_visible_ = true;

  QWidget* widget = dynamic_cast<QWidget*>(render_window_);
  if (widget) {
    QApplication::postEvent( widget, new QContextMenuEvent( QContextMenuEvent::Mouse, QPoint() ));
  }
}

void RenderPanel::onContextMenuHide()
{
  context_menu_visible_ = false;
}

bool RenderPanel::contextMenuVisible()
{
  return context_menu_visible_;
}

void RenderPanel::onContextMenuEvent( QContextMenuEvent* )
{
  boost::shared_ptr<QMenu> context_menu;
  {
    boost::mutex::scoped_lock lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if ( context_menu )
  {
    connect( context_menu.get(), SIGNAL( aboutToHide() ), this, SLOT( onContextMenuHide() ) );
    context_menu->exec( QCursor::pos() );
  }
}

void RenderPanel::sceneManagerDestroyed( Ogre::SceneManager* destroyed_scene_manager )
{
  if( destroyed_scene_manager == scene_manager_ )
  {
    scene_manager_ = nullptr;
    default_camera_ = nullptr;
    render_window_->setCamera( nullptr );
  }
}

void RenderPanel::setCursor( const QCursor &cursor )
{
  render_window_->setCursor( cursor );
}

double RenderPanel::getWindowPixelRatio()
{
  return render_window_->getWindowPixelRatio();
}

QPoint RenderPanel::mapFromGlobal( const QPoint &point ) const
{
  return render_window_->mapFromGlobal( point );
}

QPoint RenderPanel::mapToGlobal( const QPoint &point ) const
{
  return render_window_->mapToGlobal( point );
}

void RenderPanel::renderOneFrame()
{
  render_window_->updateScene();
}

} // namespace rviz
