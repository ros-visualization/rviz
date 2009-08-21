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

#ifndef RVIZ_DISPLAYS_PANEL_H
#define RVIZ_DISPLAYS_PANEL_H

#include "generated/rviz_generated.h"

#include <boost/thread/mutex.hpp>
#include <boost/signals/trackable.hpp>
#include <boost/weak_ptr.hpp>

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

class wxTimerEvent;
class wxKeyEvent;
class wxSizeEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

namespace rviz
{

class VisualizationManager;
class Tool;

class Display;
class DisplayWrapper;
typedef std::vector<DisplayWrapper*> V_DisplayWrapper;

/**
 * \class DisplaysPanel
 *
 */
class DisplaysPanel : public DisplaysPanelGenerated, public boost::signals::trackable
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  DisplaysPanel( wxWindow* parent );
  virtual ~DisplaysPanel();

  void initialize(VisualizationManager* manager);

  wxPropertyGrid* getPropertyGrid() { return property_grid_; }
  VisualizationManager* getManager() { return manager_; }

protected:
  void setDisplayCategoryLabel(const DisplayWrapper* display, int index);
  void setDisplayCategoryColor(const DisplayWrapper* display);

  void sortDisplays();

  // wx callbacks
  /// Called when a property from the wxPropertyGrid is changing
  void onPropertyChanging( wxPropertyGridEvent& event );
  /// Called when a property from the wxProperty
  void onPropertyChanged( wxPropertyGridEvent& event );
  /// Called when a property is selected
  void onPropertySelected( wxPropertyGridEvent& event );

  /// Called when the "New Display" button is pressed
  virtual void onNewDisplay( wxCommandEvent& event );
  /// Called when the "Delete Display" button is pressed
  virtual void onDeleteDisplay( wxCommandEvent& event );
  /// Called when the "Move Up" button is pressed
  virtual void onMoveUp( wxCommandEvent& event );
  /// Called when the "Move Down" button is pressed
  virtual void onMoveDown( wxCommandEvent& event );

  // Other callbacks
  /// Called when a display is enabled or disabled
  void onDisplayStateChanged(Display* display);
  void onDisplayCreated(DisplayWrapper* display);
  void onDisplayDestroyed(DisplayWrapper* display);
  void onDisplayAdding(DisplayWrapper* display);
  void onDisplayAdded(DisplayWrapper* display);
  void onDisplayRemoving(DisplayWrapper* display);
  void onDisplayRemoved(DisplayWrapper* display);
  void onDisplaysRemoving(const V_DisplayWrapper& displays);
  void onDisplaysRemoved(const V_DisplayWrapper& displays);
  void onDisplaysConfigLoaded(const boost::shared_ptr<wxConfigBase>& config);
  void onDisplaysConfigSaving(const boost::shared_ptr<wxConfigBase>& config);

  wxPropertyGrid* property_grid_;                         ///< Display property grid
  VisualizationManager* manager_;
  DisplayWrapper* selected_display_;

  typedef std::map<DisplayWrapper*, uint32_t> M_DisplayToIndex;
  M_DisplayToIndex display_map_;
};

} // namespace rviz

#endif

