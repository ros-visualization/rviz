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
#include <utility>


#include <OgreSceneManager.h>
#include <OgreCamera.h>

#include <rviz/display.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <rviz/render_panel.h>

namespace rviz
{
RenderPanel::RenderPanel(QWidget* parent)
  : QtOgreRenderWindow(parent)
  , mouse_x_(0)
  , mouse_y_(0)
  , focus_on_mouse_move_(true)
  , context_(nullptr)
  , scene_manager_(nullptr)
  , view_controller_(nullptr)
  , context_menu_visible_(false)
  , default_camera_(nullptr)
{
  setFocusPolicy(Qt::WheelFocus);
  setFocus(Qt::OtherFocusReason);
  setMouseTracking(true);
}

RenderPanel::~RenderPanel()
{
  if (scene_manager_ && default_camera_)
  {
    scene_manager_->destroyCamera(default_camera_);
  }
  if (scene_manager_)
  {
    scene_manager_->removeListener(this);
  }
}

void RenderPanel::initialize(Ogre::SceneManager* scene_manager, DisplayContext* context)
{
  context_ = context;
  scene_manager_ = scene_manager;
  scene_manager_->addListener(this);

  std::stringstream ss;
  static int count = 0;
  ss << "RenderPanelCamera" << count++;
  default_camera_ = scene_manager_->createCamera(ss.str());
  default_camera_->setNearClipDistance(0.01f);
  default_camera_->setPosition(0, 10, 15);
  default_camera_->lookAt(0, 0, 0);

  setCamera(default_camera_);
}

void RenderPanel::leaveEvent(QEvent* /*event*/)
{
  setCursor(Qt::ArrowCursor);
  if (context_)
  {
    context_->setStatus("");
  }
}

void RenderPanel::onRenderWindowMouseEvents(QMouseEvent* event)
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_)
  {
    if (focus_on_mouse_move_)
    {
      setFocus(Qt::MouseFocusReason);
    }

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::wheelEvent(QWheelEvent* event)
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_)
  {
    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::keyPressEvent(QKeyEvent* event)
{
  if (context_)
  {
    context_->handleChar(event, this);
  }
}

void RenderPanel::setViewController(ViewController* controller)
{
  view_controller_ = controller;

  if (view_controller_)
  {
    setCamera(view_controller_->getCamera());
    view_controller_->activate();
  }
  else
  {
    setCamera(nullptr);
  }
}

void RenderPanel::showContextMenu(boost::shared_ptr<QMenu> menu)
{
  boost::mutex::scoped_lock lock(context_menu_mutex_);
  context_menu_ = std::move(menu);
  context_menu_visible_ = true;

  QApplication::postEvent(this, new QContextMenuEvent(QContextMenuEvent::Mouse, QPoint()));
}

void RenderPanel::onContextMenuHide()
{
  context_menu_visible_ = false;
}

bool RenderPanel::contextMenuVisible()
{
  return context_menu_visible_;
}

void RenderPanel::contextMenuEvent(QContextMenuEvent* /*event*/)
{
  boost::shared_ptr<QMenu> context_menu;
  {
    boost::mutex::scoped_lock lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if (context_menu)
  {
    connect(context_menu.get(), SIGNAL(aboutToHide()), this, SLOT(onContextMenuHide()));
    context_menu->exec(QCursor::pos());
  }
}

void RenderPanel::sceneManagerDestroyed(Ogre::SceneManager* destroyed_scene_manager)
{
  if (destroyed_scene_manager == scene_manager_)
  {
    scene_manager_ = nullptr;
    default_camera_ = nullptr;
    setCamera(nullptr);
  }
}

bool RenderPanel::getFocusOnMouseMove() const
{
  return focus_on_mouse_move_;
}

void RenderPanel::setFocusOnMouseMove(bool enabled)
{
  focus_on_mouse_move_ = enabled;
}

} // namespace rviz
