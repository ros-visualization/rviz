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

#ifndef OGRE_VISUALIZER_VISUALIZATION_FRAME_H
#define OGRE_VISUALIZER_VISUALIZATION_FRAME_H

#include <wx/wx.h>

#include <string>

class wxConfigBase;
class wxMenuBar;
class wxMenu;
class wxAuiManager;

namespace ogre_vis
{

class RenderPanel;
class DisplaysPanel;
class VisualizationManager;

class VisualizationFrame : public wxFrame
{
public:
  VisualizationFrame(wxWindow* parent);
  ~VisualizationFrame();

  void initialize();

protected:
  void initConfigs();
  void initMenus();
  void loadDisplayConfig(const std::string& path);
  void loadConfigMenus();
  void saveConfigs();

  void onClose(wxCommandEvent& event);
  void onOpen(wxCommandEvent& event);
  void onSave(wxCommandEvent& event);
  void onGlobalConfig(wxCommandEvent& event);
  void onLocalConfig(wxCommandEvent& event);

  RenderPanel* render_panel_;
  DisplaysPanel* displays_panel_;

  wxConfigBase* general_config_;
  wxConfigBase* display_config_;
  std::string config_dir_;
  std::string general_config_file_;
  std::string display_config_file_;
  std::string save_dir_;
  std::string global_config_dir_;

  wxMenuBar* menubar_;
  wxMenu* file_menu_;
  wxMenu* local_configs_menu_;
  wxMenu* global_configs_menu_;

  wxAuiManager* aui_manager_;

  VisualizationManager* manager_;
};

}

#endif // OGRE_VISUALIZER_VISUALIZATION_FRAME_H
