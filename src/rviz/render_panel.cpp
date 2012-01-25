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

#include "render_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "display_wrapper.h"
#include "tools/tool.h"
#include "viewport_mouse_event.h"
#include "view_controller.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>

namespace rviz
{

RenderPanel::RenderPanel( QWidget* parent )
  : QtOgreRenderWindow( parent )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  , manager_( 0 )
  , scene_manager_( 0 )
  , camera_( 0 )
  , view_controller_( 0 )
  , fake_mouse_move_event_timer_( new QTimer() )
{
  setFocus( Qt::OtherFocusReason );
}

RenderPanel::~RenderPanel()
{
  delete fake_mouse_move_event_timer_;
  delete view_controller_;
  scene_manager_->destroyCamera(camera_);
}

void RenderPanel::initialize(Ogre::SceneManager* scene_manager, VisualizationManager* manager)
{
  manager_ = manager;
  scene_manager_ = scene_manager;

  std::stringstream ss;
  static int count = 0;
  ss << "RenderPanelCamera" << count++;
  camera_ = scene_manager_->createCamera(ss.str());

  setCamera( camera_ );

  connect( fake_mouse_move_event_timer_, SIGNAL( timeout() ), this, SLOT( sendMouseMoveEvent() ));
  fake_mouse_move_event_timer_->start( 33 /*milliseconds*/ );
}

void RenderPanel::sendMouseMoveEvent()
{
  QPoint cursor_pos = QCursor::pos();
  QPoint mouse_rel_widget = mapFromGlobal( cursor_pos );
  if( rect().contains( mouse_rel_widget ))
  {
    bool mouse_over_this = false;
    QWidget *w = QApplication::widgetAt( cursor_pos );
    while( w )
    {
      if( w == this )
      {
        mouse_over_this = true;
        break;
      }
      w = w->parentWidget();
    }
    if( !mouse_over_this )
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

void RenderPanel::onRenderWindowMouseEvents( QMouseEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (manager_)
  {
    setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    manager_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::wheelEvent( QWheelEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (manager_)
  {
    setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    manager_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::keyPressEvent( QKeyEvent* event )
{
  if( manager_ )
  {
    manager_->handleChar( event, this );
  }
}

void RenderPanel::setViewController(ViewController* controller)
{
  if (view_controller_)
  {
    view_controller_->deactivate();
  }

  delete view_controller_;
  view_controller_ = controller;

  view_controller_->activate(camera_, manager_ ? manager_->getTargetFrame() : "");
}

void RenderPanel::showContextMenu( boost::shared_ptr<QMenu> menu )
{
  boost::mutex::scoped_lock lock(context_menu_mutex_);
  context_menu_ = menu;

  QApplication::postEvent( this, new QContextMenuEvent( QContextMenuEvent::Mouse, QPoint() ));
}

void RenderPanel::contextMenuEvent( QContextMenuEvent* event )
{
  boost::shared_ptr<QMenu> context_menu;
  {
    boost::mutex::scoped_lock lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if ( context_menu )
  {
    context_menu->exec( QCursor::pos() );
  }
}

} // namespace rviz
