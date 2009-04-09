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

#ifndef RVIZ_SELECTION_PANEL_H
#define RVIZ_SELECTION_PANEL_H

#include "generated/rviz_generated.h"

#include <boost/thread/mutex.hpp>
#include <boost/signals/trackable.hpp>

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
class wxSizeEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

namespace rviz
{

class VisualizationManager;
class PropertyManager;

class SelectionAddedArgs;
class SelectionRemovedArgs;
class SelectionSettingArgs;
class SelectionSetArgs;

/**
 * \class SelectionPanel
 *
 */
class SelectionPanel : public wxPanel, public boost::signals::trackable
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  SelectionPanel( wxWindow* parent );
  virtual ~SelectionPanel();

  void initialize(VisualizationManager* manager);

  wxPropertyGrid* getPropertyGrid() { return property_grid_; }
  VisualizationManager* getManager() { return manager_; }

protected:
  // wx callbacks
  /// Called when a property from the wxPropertyGrid is changing
  void onPropertyChanging( wxPropertyGridEvent& event );
  /// Called when a property from the wxProperty
  void onPropertyChanged( wxPropertyGridEvent& event );
  /// Called when a property is selected
  void onPropertySelected( wxPropertyGridEvent& event );

  // Other callbacks
  void onSelectionSetting(const SelectionSettingArgs& args);
  void onSelectionSet(const SelectionSetArgs& args);
  void onSelectionAdded(const SelectionAddedArgs& args);
  void onSelectionRemoved(const SelectionRemovedArgs& args);

  /// Called from the refresh timer
  void onUpdate( wxTimerEvent& event );

  wxPropertyGrid* property_grid_;
  PropertyManager* property_manager_;

  VisualizationManager* manager_;

  wxTimer* refresh_timer_;

  bool setting_;
};

} // namespace rviz

#endif

