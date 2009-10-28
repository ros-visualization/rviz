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

#ifndef RVIZ_VISUALIZATION_FRAME_H
#define RVIZ_VISUALIZATION_FRAME_H

#include "window_manager_interface.h"

#include <wx/frame.h>

#include <string>
#include <deque>
#include <boost/shared_ptr.hpp>

class wxConfigBase;
class wxMenuBar;
class wxMenu;
class wxAuiManager;
class wxAuiManagerEvent;
class wxToolBar;

namespace rviz
{

class RenderPanel;
class DisplaysPanel;
class ViewsPanel;
class TimePanel;
class SelectionPanel;
class ToolPropertiesPanel;
class VisualizationManager;
class Tool;
class SplashScreen;

class VisualizationFrame : public wxFrame, public WindowManagerInterface
{
public:
  VisualizationFrame(wxWindow* parent);
  ~VisualizationFrame();

  void initialize(const std::string& display_config_file = "", const std::string& fixed_frame = "", const std::string& target_frame = "");

  VisualizationManager* getManager() { return manager_; }

  // overrides from WindowManagerInterface
  virtual wxWindow* getParentWindow();
  virtual void addPane(const std::string& name, wxWindow* panel);
  virtual void removePane(wxWindow* panel);
  virtual void showPane(wxWindow* panel);
  virtual void closePane(wxWindow* panel);

protected:
  void initConfigs();
  void initMenus();
  void loadDisplayConfig(const std::string& path);
  void saveConfigs();

  // wx Callbacks
  void onClose(wxCommandEvent& event);
  void onOpen(wxCommandEvent& event);
  void onSave(wxCommandEvent& event);
  /// Called when a tool is selected
  void onToolClicked( wxCommandEvent& event );
  void onPaneClosed(wxAuiManagerEvent& event);
  void onViewMenuItemSelected(wxCommandEvent& event);
  void onManagePlugins(wxCommandEvent& event);
  void onHelpWiki(wxCommandEvent& event);
  void onRecentConfigSelected(wxCommandEvent& event);

  // other Callbacks
  void onToolAdded(Tool* tool);
  void onToolChanged(Tool* tool);

  void onSplashLoadStatus(const std::string& status, SplashScreen* splash);

  void markRecentConfig(const std::string& path);
  void updateRecentConfigMenu();

  RenderPanel* render_panel_;
  DisplaysPanel* displays_panel_;
  ViewsPanel* views_panel_;
  TimePanel* time_panel_;
  SelectionPanel* selection_panel_;
  ToolPropertiesPanel* tool_properties_panel_;

  boost::shared_ptr<wxConfigBase> general_config_;
  boost::shared_ptr<wxConfigBase> display_config_;
  std::string config_dir_;
  std::string general_config_file_;
  std::string display_config_file_;
  std::string last_config_dir_;

  wxMenuBar* menubar_;
  wxMenu* file_menu_;
  wxMenu* recent_configs_menu_;
  wxMenu* view_menu_;
  wxMenu* plugins_menu_;
  wxMenu* help_menu_;

  wxToolBar* toolbar_;

  wxAuiManager* aui_manager_;

  VisualizationManager* manager_;

  std::string package_path_;

  SplashScreen* splash_;

  typedef std::deque<std::string> D_string;
  D_string recent_configs_;
};

}

#endif // RVIZ_VISUALIZATION_FRAME_H
