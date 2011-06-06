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

#include "ogre_tools/wx_ogre_render_window.h"

#include <boost/thread/mutex.hpp>
#include <boost/signals/trackable.hpp>

#include <vector>
#include <map>

namespace Ogre
{
class Root;
class SceneManager;
class Camera;
class RaySceneQuery;
class ParticleSystem;
}

namespace ros
{
class Node;
}

class wxTimerEvent;
class wxKeyEvent;
class wxSizeEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

namespace rviz
{

class Display;
class VisualizationManager;
class ViewController;

/**
 * \class RenderPanel
 *
 */
class RenderPanel : public ogre_tools::wxOgreRenderWindow, public boost::signals::trackable
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  RenderPanel( wxWindow* parent, bool create_render_window = true );
  virtual ~RenderPanel();

  void initialize(Ogre::SceneManager* scene_manager, VisualizationManager* manager);

  VisualizationManager* getManager() { return manager_; }

  Ogre::Camera* getCamera() { return camera_; }
  ViewController* getViewController() { return view_controller_; }
  void setViewController(ViewController* controller);

  virtual void createRenderWindow();

  void setContextMenu( boost::shared_ptr<wxMenu> menu );
  void onContextMenu( wxContextMenuEvent& event );

protected:
  // wx Callbacks
  /// Called when a mouse event happens inside the render window
  void onRenderWindowMouseEvents( wxMouseEvent& event );
  /// Called when a key is pressed
  void onChar( wxKeyEvent& event );

  // Mouse handling
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  VisualizationManager* manager_;
  Ogre::SceneManager* scene_manager_;
  Ogre::Camera* camera_;

  ViewController* view_controller_;

  boost::shared_ptr<wxMenu> context_menu_;
  boost::mutex context_menu_mutex_;

private:
  void setCamera(Ogre::Camera*) {}
};

} // namespace rviz

#endif

