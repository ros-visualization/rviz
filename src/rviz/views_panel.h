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

#ifndef RVIZ_VIEWS_PANEL_H
#define RVIZ_VIEWS_PANEL_H

#include "generated/rviz_generated.h"

#include <boost/thread/mutex.hpp>
#include <boost/signals/trackable.hpp>

#include <vector>
#include <map>

namespace ros
{
class Node;
}

namespace ogre_tools
{
class CameraBase;
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
class Tool;

/**
 * \class ViewsPanel
 *
 */
class ViewsPanel : public ViewsPanelGenerated, public boost::signals::trackable
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  ViewsPanel( wxWindow* parent );
  virtual ~ViewsPanel();

  void initialize(VisualizationManager* manager);

  VisualizationManager* getManager() { return manager_; }

protected:
  struct View
  {
    std::string name_;
    std::string camera_type_;
    std::string camera_config_;
    std::string target_frame_;
  };
  typedef std::vector<View> V_View;

  void loadSelected();
  void save(const std::string& name);
  void addView(const View& view);

  // wx Callbacks
  /// Called when a camera type is selected from the list
  virtual void onCameraTypeSelected( wxCommandEvent& event );
  virtual void onViewsClicked( wxCommandEvent& event );
  virtual void onViewsDClicked( wxCommandEvent& event );
  virtual void onLoadClicked( wxCommandEvent& event );
  virtual void onSaveClicked( wxCommandEvent& event );
  virtual void onDeleteClicked( wxCommandEvent& event );

  // Other callbacks
  void onGeneralConfigLoaded(const boost::shared_ptr<wxConfigBase>& config);
  void onGeneralConfigSaving(const boost::shared_ptr<wxConfigBase>& config);
  void onCameraTypeAdded(ogre_tools::CameraBase* camera, const std::string& name);
  void onCameraTypeChanged(ogre_tools::CameraBase* camera);

  VisualizationManager* manager_;


  V_View views_;
};

} // namespace rviz

#endif // RVIZ_VIEWS_PANEL_H


