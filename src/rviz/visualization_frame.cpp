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

#include "visualization_frame.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "visualization_manager.h"

#include <ros/common.h>
#include <ros/console.h>

#include <ogre_tools/initialization.h>

#include <wx/config.h>
#include <wx/stdpaths.h>
#include <wx/menu.h>
#include <wx/aui/aui.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

#define CONFIG_WINDOW_X wxT("/Window/X")
#define CONFIG_WINDOW_Y wxT("/Window/Y")
#define CONFIG_WINDOW_WIDTH wxT("/Window/Width")
#define CONFIG_WINDOW_HEIGHT wxT("/Window/Height")
#define CONFIG_AUIMANAGER_PERSPECTIVE wxT("/AuiManagerPerspective")

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION

namespace rviz
{

VisualizationFrame::VisualizationFrame(wxWindow* parent)
: wxFrame(parent, wxID_ANY, wxT("Visualizer"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE)
, general_config_(NULL)
, display_config_(NULL)
, menubar_(NULL)
, file_menu_(NULL)
, local_configs_menu_(NULL)
, global_configs_menu_(NULL)
, aui_manager_(NULL)
{
  render_panel_ = new RenderPanel( this );
  displays_panel_ = new DisplaysPanel( this );

  std::string mediaPath = ros::getPackagePath( "gazebo_robot_description" );
  mediaPath += "/world/Media/";
  ogre_tools::V_string paths;
  paths.push_back( mediaPath );
  paths.push_back( mediaPath + "fonts" );
  paths.push_back( mediaPath + "materials" );
  paths.push_back( mediaPath + "materials/scripts" );
  paths.push_back( mediaPath + "materials/programs" );
  paths.push_back( mediaPath + "materials/textures" );
  paths.push_back( mediaPath + "models" );
  paths.push_back( mediaPath + "models/pr2" );

  ogre_tools::initializeResources( paths );

  std::string package_path = ros::getPackagePath("rviz");
  global_config_dir_ = (fs::path(package_path) / "configs").file_string();
}

VisualizationFrame::~VisualizationFrame()
{
  saveConfigs();

  render_panel_->Destroy();
  delete manager_;
}

void VisualizationFrame::initialize()
{
  aui_manager_ = new wxAuiManager(this);
  aui_manager_->AddPane(render_panel_, wxAuiPaneInfo().CenterPane().Name(wxT("Render")).Caption(wxT("Render")));
  aui_manager_->AddPane(displays_panel_, wxAuiPaneInfo().CloseButton(false).Left().Name(wxT("Displays")).Caption(wxT("Displays")));
  aui_manager_->Update();

  initConfigs();
  initMenus();

  wxPoint pos = GetPosition();
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();
  general_config_->Read(CONFIG_WINDOW_X, &pos.x, pos.x);
  general_config_->Read(CONFIG_WINDOW_Y, &pos.y, pos.y);
  general_config_->Read(CONFIG_WINDOW_WIDTH, &width, width);
  general_config_->Read(CONFIG_WINDOW_HEIGHT, &height, height);

  wxString auimanager_perspective;
  if (general_config_->Read(CONFIG_AUIMANAGER_PERSPECTIVE, &auimanager_perspective))
  {
    aui_manager_->LoadPerspective(auimanager_perspective);
    aui_manager_->Update();
  }

  SetPosition(pos);
  SetSize(wxSize(width, height));

  manager_ = new VisualizationManager(render_panel_, displays_panel_);
  render_panel_->initialize(manager_);
  displays_panel_->initialize(manager_);

  manager_->initialize();
  manager_->loadGeneralConfig(general_config_);
  manager_->loadDisplayConfig(display_config_);
}

void VisualizationFrame::initConfigs()
{
  config_dir_ = (const char*)wxStandardPaths::Get().GetUserConfigDir().fn_str();
  std::string old_dir = (fs::path(config_dir_) / ".standalone_visualizer").file_string();
  config_dir_ = (fs::path(config_dir_) / ".rviz").file_string();
  general_config_file_ = (fs::path(config_dir_) / "config").file_string();
  display_config_file_ = (fs::path(config_dir_) / "display_config").file_string();

  if (fs::exists(old_dir) && !fs::exists(config_dir_))
  {
    ROS_INFO("Migrating old config directory to new location ([%s] to [%s])", old_dir.c_str(), config_dir_.c_str());
    fs::rename(old_dir, config_dir_);
  }

  if (fs::is_regular_file(config_dir_))
  {
    ROS_INFO("Migrating old config file to new location ([%s] to [%s])", config_dir_.c_str(), general_config_file_.c_str());
    std::string backup_file = config_dir_ + "bak";

    fs::rename(config_dir_, backup_file);
    fs::create_directory(config_dir_);
    fs::rename(backup_file, general_config_file_);
  }
  else if (!fs::exists(config_dir_))
  {
    fs::create_directory(config_dir_);
  }

  if (fs::exists(general_config_file_) && !fs::exists(display_config_file_))
  {
    ROS_INFO("Creating display config from general config");
    fs::copy_file(general_config_file_, display_config_file_);
  }

  save_dir_ = (fs::path(config_dir_) / "saved").file_string();
  if (!fs::exists(save_dir_))
  {
    fs::create_directory(save_dir_);
  }

  ROS_INFO("Loading general config from [%s]", general_config_file_.c_str());
  general_config_ = new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(general_config_file_.c_str()));
  ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
  display_config_ = new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(display_config_file_.c_str()));
}

void VisualizationFrame::initMenus()
{
  menubar_ = new wxMenuBar();
  file_menu_ = new wxMenu(wxT(""));
  wxMenuItem* item = file_menu_->Append(wxID_OPEN, wxT("&Open Display Config\tCtrl-O"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onOpen), NULL, this);
  item = file_menu_->Append(wxID_SAVE, wxT("&Save Display Config\tCtrl-S"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onSave), NULL, this);

  local_configs_menu_ = new wxMenu(wxT(""));
  global_configs_menu_ = new wxMenu(wxT(""));
  file_menu_->Append(wxID_ANY, wxT("&Local Display Configs"), local_configs_menu_);
  file_menu_->Append(wxID_ANY, wxT("&Global Display Configs"), global_configs_menu_);

  file_menu_->AppendSeparator();
  item = file_menu_->Append(wxID_EXIT, wxT("&Quit"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onClose), NULL, this);

  menubar_->Append(file_menu_, wxT("&File"));

  SetMenuBar(menubar_);

  loadConfigMenus();
}

void VisualizationFrame::loadDisplayConfig(const std::string& path)
{
  manager_->removeAllDisplays();

  wxFileConfig config(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(path.c_str()));
  manager_->loadDisplayConfig(&config);
}

void VisualizationFrame::loadConfigMenus()
{
  // First clear the menus
  {
    wxMenuItemList items = local_configs_menu_->GetMenuItems();
    for (wxMenuItemList::Node* node = items.GetFirst(); node; node = node->GetNext())
    {
      wxMenuItem* item = node->GetData();
      local_configs_menu_->Destroy(item);
    }
  }

  {
    wxMenuItemList items = global_configs_menu_->GetMenuItems();
    for (wxMenuItemList::Node* node = items.GetFirst(); node; node = node->GetNext())
    {
      wxMenuItem* item = node->GetData();
      global_configs_menu_->Destroy(item);
    }
  }

  fs::directory_iterator dir_end;
  for (fs::directory_iterator it(save_dir_); it != dir_end; ++it)
  {
    fs::path path = it->path();
    if (fs::is_regular_file(path) && path.extension() == "."CONFIG_EXTENSION)
    {
      std::string name = path.stem();
      wxMenuItem* item = local_configs_menu_->Append(wxID_ANY, wxString::FromAscii(name.c_str()));
      Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onLocalConfig), NULL, this);
    }
  }

  for (fs::directory_iterator it(global_config_dir_); it != dir_end; ++it)
  {
    fs::path path = it->path();
    if (fs::is_regular_file(path) && path.extension() == "."CONFIG_EXTENSION)
    {
      std::string name = path.stem();
      wxMenuItem* item = global_configs_menu_->Append(wxID_ANY, wxString::FromAscii(name.c_str()));
      Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onGlobalConfig), NULL, this);
    }
  }
}

void VisualizationFrame::saveConfigs()
{
  ROS_INFO("Saving general config to [%s]", general_config_file_.c_str());
  general_config_->DeleteAll();
  wxPoint pos = GetPosition();
  wxSize size = GetSize();
  general_config_->Write(CONFIG_WINDOW_X, pos.x);
  general_config_->Write(CONFIG_WINDOW_Y, pos.y);
  general_config_->Write(CONFIG_WINDOW_WIDTH, size.GetWidth());
  general_config_->Write(CONFIG_WINDOW_HEIGHT, size.GetHeight());

  general_config_->Write(CONFIG_AUIMANAGER_PERSPECTIVE, aui_manager_->SavePerspective());

  manager_->saveGeneralConfig(general_config_);
  general_config_->Flush();

  ROS_INFO("Saving display config to [%s]", display_config_file_.c_str());
  display_config_->DeleteAll();
  manager_->saveDisplayConfig(display_config_);
  display_config_->Flush();
}


void VisualizationFrame::onClose(wxCommandEvent& event)
{
  Close();
}

void VisualizationFrame::onOpen(wxCommandEvent& event)
{
  wxString wxstr_file = wxFileSelector(wxT("Choose a file to open"), wxString::FromAscii(save_dir_.c_str()), wxEmptyString,
                                       wxT(CONFIG_EXTENSION), wxT(CONFIG_EXTENSION_WILDCARD), wxFD_OPEN|wxFD_FILE_MUST_EXIST, this);
  if (!wxstr_file.empty())
  {
    std::string filename = (const char*)wxstr_file.fn_str();
    loadDisplayConfig(filename);
  }
}

void VisualizationFrame::onSave(wxCommandEvent& event)
{
  wxString wxstr_file = wxFileSelector(wxT("Choose a file to save to"), wxString::FromAscii(save_dir_.c_str()), wxEmptyString,
                                       wxT(CONFIG_EXTENSION), wxT(CONFIG_EXTENSION_WILDCARD), wxFD_SAVE|wxFD_OVERWRITE_PROMPT, this);

  if (!wxstr_file.empty())
  {
    std::string filename = (const char*)wxstr_file.fn_str();
    fs::path path(filename);
    if (path.extension() != CONFIG_EXTENSION)
    {
      filename += "."CONFIG_EXTENSION;
    }

    wxFileConfig config(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(filename.c_str()));
    config.DeleteAll();

    manager_->saveDisplayConfig(&config);
    config.Flush();

    loadConfigMenus();
  }
}

void VisualizationFrame::onGlobalConfig(wxCommandEvent& event)
{
  wxMenuItem* item = menubar_->FindItem(event.GetId());
  std::string filename = (const char*)item->GetLabel().fn_str();

  fs::path path(global_config_dir_);
  path /= filename + "." + CONFIG_EXTENSION;

  loadDisplayConfig(path.file_string());
}

void VisualizationFrame::onLocalConfig(wxCommandEvent& event)
{
  wxMenuItem* item = menubar_->FindItem(event.GetId());
  std::string filename = (const char*)item->GetLabel().fn_str();

  fs::path path(save_dir_);
  path /= filename + "." + CONFIG_EXTENSION;

  loadDisplayConfig(path.file_string());
}


}
