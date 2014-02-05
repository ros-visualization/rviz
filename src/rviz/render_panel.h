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

#ifndef RVIZ_RENDER_PANEL_H
#define RVIZ_RENDER_PANEL_H

#include "ogre_helpers/qt_ogre_render_window.h"

#include <OgreSceneManager.h>

#include <boost/thread/mutex.hpp>

#include <vector>
#include <map>

namespace Ogre
{
class Root;
class Camera;
class RaySceneQuery;
class ParticleSystem;
}

namespace ros
{
class Node;
}

class QMenu;
class QKeyEvent;
class PropertyTreeWidget;

namespace rviz
{

class Display;
class DisplayContext;
class ViewController;

/**
 * A widget which shows an OGRE-rendered scene in RViz.
 *
 * RenderPanel displays a scene and forwards mouse and key events to
 * the DisplayContext (which further forwards them to the active
 * Tool, etc.)
 */
class RenderPanel: public QtOgreRenderWindow, public Ogre::SceneManager::Listener
{
Q_OBJECT
public:
  /** Constructor.  Ogre::Root::createRenderWindow() is called within. */
  RenderPanel( QWidget* parent = 0 );
  virtual ~RenderPanel();

  /** This sets up the Ogre::Camera for this widget. */
  void initialize(Ogre::SceneManager* scene_manager, DisplayContext* manager);

  DisplayContext* getManager() { return context_; }

  ViewController* getViewController() { return view_controller_; }

  /** @brief Set the ViewController which should control the camera
   * position for this view. */
  void setViewController( ViewController* controller );

  /** Show the given menu as a context menu, positioned based on the
   * current mouse position.  This can be called from any thread. */
  void showContextMenu( boost::shared_ptr<QMenu> menu );

  /** Return true if the context menu for this panel is visible */
  bool contextMenuVisible();

  virtual void sceneManagerDestroyed( Ogre::SceneManager* source );

protected:
  // Override from QWidget
  void contextMenuEvent( QContextMenuEvent* event );

  /// Called when any mouse event happens inside the render window
  void onRenderWindowMouseEvents( QMouseEvent* event );

  // QWidget mouse events all get sent to onRenderWindowMouseEvents().
  // QMouseEvent.type() distinguishes them later.
  virtual void mouseMoveEvent( QMouseEvent* event ) { onRenderWindowMouseEvents( event ); }
  virtual void mousePressEvent( QMouseEvent* event ) { onRenderWindowMouseEvents( event ); }
  virtual void mouseReleaseEvent( QMouseEvent* event ) { onRenderWindowMouseEvents( event ); }
  virtual void mouseDoubleClickEvent( QMouseEvent* event ) { onRenderWindowMouseEvents( event ); }

  virtual void leaveEvent ( QEvent * event );

  /// Called when there is a mouse-wheel event.
  virtual void wheelEvent( QWheelEvent* event );

  virtual void keyPressEvent( QKeyEvent* event );

  // Mouse handling
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  DisplayContext* context_;
  Ogre::SceneManager* scene_manager_;

  ViewController* view_controller_;

  boost::shared_ptr<QMenu> context_menu_;
  boost::mutex context_menu_mutex_;

  bool context_menu_visible_;

  // Pointer to the Display which is using this render panel, or NULL
  // if this does not belong to a Display.
  Display* display_;

private Q_SLOTS:
  void sendMouseMoveEvent();
  void onContextMenuHide();

private:
  QTimer* fake_mouse_move_event_timer_;
  Ogre::Camera* default_camera_; ///< A default camera created in initialize().
};

} // namespace rviz

#endif

