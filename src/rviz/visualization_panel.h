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

#ifndef OGRE_VISUALIZER_VISUALIZATION_PANEL_H
#define OGRE_VISUALIZER_VISUALIZATION_PANEL_H

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b ogre_display is a 3D visualization framework that is embeddable anywhere, as a wxPanel
 *
 */

#include "generated/visualization_panel_generated.h"

#include "boost/thread/mutex.hpp"

#include "wx/stopwatch.h"

#include <vector>
#include <map>

namespace ogre_tools
{
class wxOgreRenderWindow;
class FPSCamera;
class OrbitCamera;
class CameraBase;
class OrthoCamera;
}

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
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

namespace ogre_vis
{

class Display;
class VisualizationManager;
class Tool;

/**
 * \class VisualizationPanel
 * \brief A self-contained wxPanel for 3D visualization of pretty much anything.
 *
 */
class VisualizationPanel : public VisualizationPanelGenerated
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  VisualizationPanel( wxWindow* parent );
  virtual ~VisualizationPanel();

  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  /**
   * \brief Locks the renderer
   */
  void lockRender() { render_mutex_.lock(); }
  /**
   * \brief Unlocks the renderer
   */
  void unlockRender() { render_mutex_.unlock(); }

  ogre_tools::wxOgreRenderWindow* getRenderPanel() { return render_panel_; }
  wxPropertyGrid* getPropertyGrid() { return property_grid_; }
  VisualizationManager* getManager() { return manager_; }
  ogre_tools::CameraBase* getCurrentCamera() { return current_camera_; }
  const char* getCurrentCameraType();
  bool setCurrentCamera(const std::string& camera_type);
  void setCurrentCamera(int camera_type);

  void addTool( Tool* tool );
  void setTool( Tool* tool );

protected:
  /// Called when a mouse event happens inside the render window
  void onRenderWindowMouseEvents( wxMouseEvent& event );
  /// Called when our custom EVT_RENDER is fired
  void onRender( wxCommandEvent& event );
  /// Called when a property from the wxPropertyGrid is changing
  void onPropertyChanging( wxPropertyGridEvent& event );
  /// Called when a property from the wxProperty
  void onPropertyChanged( wxPropertyGridEvent& event );
  /// Called when a property is selected
  void onPropertySelected( wxPropertyGridEvent& event );
  /// Called when a tool is selected
  void onToolClicked( wxCommandEvent& event );
  /// Called when a key is pressed
  void onChar( wxKeyEvent& event );

  /// Called when a "view" (camera) is selected from the list
  virtual void onViewSelected( wxCommandEvent& event );
  /// Called when the "Reset Time" button is clicked
  virtual void onResetTime( wxCommandEvent& event );
  /// Called when the "New Display" button is pressed
  virtual void onNewDisplay( wxCommandEvent& event );
  /// Called when the "Delete Display" button is pressed
  virtual void onDeleteDisplay( wxCommandEvent& event );
  /// Called when the "Move Up" button is pressed
  virtual void onMoveUp( wxCommandEvent& event );
  /// Called when the "Move Down" button is pressed
  virtual void onMoveDown( wxCommandEvent& event );

  void onDisplayStateChanged( Display* display );

  wxPropertyGrid* property_grid_;                         ///< Display property grid

  ogre_tools::wxOgreRenderWindow* render_panel_;          ///< Render window

  ogre_tools::CameraBase* current_camera_;                ///< The current camera
  int current_camera_type_;
  ogre_tools::FPSCamera* fps_camera_;                     ///< FPS camera
  ogre_tools::OrbitCamera* orbit_camera_;                 ///< Orbit camera
  ogre_tools::OrthoCamera* top_down_ortho_;               ///< Top-down orthographic camera

  // Mouse handling
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  std::string target_frame_;                              ///< Target coordinate frame we're displaying everything in

  boost::mutex render_mutex_;                       ///< Render mutex

  VisualizationManager* manager_;

  Display* selected_display_;
};

} // namespace ogre_vis

#endif
