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
#include "views_panel.h"
#include "time_panel.h"
#include "selection_panel.h"
#include "visualization_manager.h"
#include "tools/tool.h"
#include "plugin_manager_dialog.h"

#include <ros/package.h>
#include <ros/console.h>

#include <ogre_tools/initialization.h>

#include <wx/config.h>
#include <wx/confbase.h>
#include <wx/stdpaths.h>
#include <wx/menu.h>
#include <wx/toolbar.h>
#include <wx/aui/aui.h>
#include <wx/filedlg.h>
#include <wx/artprov.h>

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include <ros/node.h>

namespace fs = boost::filesystem;

#define CONFIG_WINDOW_X wxT("/Window/X")
#define CONFIG_WINDOW_Y wxT("/Window/Y")
#define CONFIG_WINDOW_WIDTH wxT("/Window/Width")
#define CONFIG_WINDOW_HEIGHT wxT("/Window/Height")
#define CONFIG_AUIMANAGER_PERSPECTIVE wxT("/AuiManagerPerspective")
#define CONFIG_AUIMANAGER_PERSPECTIVE_VERSION wxT("/AuiManagerPerspectiveVersion")

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
#define PERSPECTIVE_VERSION 1

namespace rviz
{

namespace toolbar_items
{
enum ToolbarItem
{
  Count,
};
}
typedef toolbar_items::ToolbarItem ToolbarItem;

VisualizationFrame::VisualizationFrame(wxWindow* parent)
: wxFrame(parent, wxID_ANY, wxT("RViz"), wxDefaultPosition, wxSize(1024, 768), wxDEFAULT_FRAME_STYLE)
, menubar_(NULL)
, file_menu_(NULL)
, local_configs_menu_(NULL)
, global_configs_menu_(NULL)
, aui_manager_(NULL)
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    ros::init(argc, 0, "rviz", ros::init_options::AnonymousName);
  }

  render_panel_ = new RenderPanel( this );
  displays_panel_ = new DisplaysPanel( this );
  views_panel_ = new ViewsPanel( this );
  time_panel_ = new TimePanel( this );
  selection_panel_ = new SelectionPanel( this );

  ogre_tools::V_string paths;
  ogre_tools::initializeResources( paths );

  std::string package_path = ros::package::getPath("rviz");
  global_config_dir_ = (fs::path(package_path) / "configs").file_string();

#if !defined(__WXMAC__)
  toolbar_ = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_TEXT|wxTB_NOICONS|wxNO_BORDER|wxTB_HORIZONTAL);
  toolbar_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationFrame::onToolClicked ), NULL, this );
#endif

  aui_manager_ = new wxAuiManager(this);
  aui_manager_->AddPane(render_panel_, wxAuiPaneInfo().CenterPane().Name(wxT("Render")));
  aui_manager_->AddPane(displays_panel_, wxAuiPaneInfo().Left().MinSize(270, -1).Name(wxT("Displays")).Caption(wxT("Displays")));
  aui_manager_->AddPane(selection_panel_, wxAuiPaneInfo().Right().MinSize(270, -1).Name(wxT("Selection")).Caption(wxT("Selection")));
  aui_manager_->AddPane(views_panel_, wxAuiPaneInfo().BestSize(230, 200).Right().Name(wxT("Views")).Caption(wxT("Views")));
  aui_manager_->AddPane(time_panel_, wxAuiPaneInfo().RightDockable(false).LeftDockable(false).Bottom().Name(wxT("Time")).Caption(wxT("Time")));
#if !defined(__WXMAC__)
  aui_manager_->AddPane(toolbar_, wxAuiPaneInfo().ToolbarPane().RightDockable(false).LeftDockable(false)/*.MinSize(-1, 40)*/.Top().Name(wxT("Tools")).Caption(wxT("Tools")));
#endif
  aui_manager_->Update();
}

VisualizationFrame::~VisualizationFrame()
{
  Disconnect(wxEVT_AUI_PANE_CLOSE, wxAuiManagerEventHandler(VisualizationFrame::onPaneClosed), NULL, this);
#if !defined(__WXMAC__)
  toolbar_->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationFrame::onToolClicked ), NULL, this );
#endif

  saveConfigs();

  manager_->removeAllDisplays();

  aui_manager_->UnInit();
  delete aui_manager_;

  render_panel_->Destroy();
  delete manager_;
}

void VisualizationFrame::initialize(const std::string& display_config_file, const std::string& fixed_frame, const std::string& target_frame)
{
  initConfigs();

  Connect(wxEVT_AUI_PANE_CLOSE, wxAuiManagerEventHandler(VisualizationFrame::onPaneClosed), NULL, this);

  wxPoint pos = GetPosition();
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();
  general_config_->Read(CONFIG_WINDOW_X, &pos.x, pos.x);
  general_config_->Read(CONFIG_WINDOW_Y, &pos.y, pos.y);
  general_config_->Read(CONFIG_WINDOW_WIDTH, &width, width);
  general_config_->Read(CONFIG_WINDOW_HEIGHT, &height, height);

  SetPosition(pos);
  SetSize(wxSize(width, height));

  manager_ = new VisualizationManager(render_panel_, this);
  render_panel_->initialize(manager_);
  displays_panel_->initialize(manager_);
  views_panel_->initialize(manager_);
  time_panel_->initialize(manager_);
  selection_panel_->initialize(manager_);

  manager_->getToolAddedSignal().connect( boost::bind( &VisualizationFrame::onToolAdded, this, _1 ) );
  manager_->getToolChangedSignal().connect( boost::bind( &VisualizationFrame::onToolChanged, this, _1 ) );

  manager_->initialize();
  manager_->loadGeneralConfig(general_config_);

  bool display_config_valid = !display_config_file.empty();
  if (display_config_valid && !fs::exists(display_config_file))
  {
    ROS_ERROR("File [%s] does not exist", display_config_file.c_str());
    display_config_valid = false;
  }

  if (!display_config_valid)
  {
    manager_->loadDisplayConfig(display_config_);
  }
  else
  {
    boost::shared_ptr<wxFileConfig> config(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxEmptyString, wxString::FromAscii(display_config_file.c_str()), wxCONFIG_USE_GLOBAL_FILE));
    manager_->loadDisplayConfig(config);
  }

  if (!fixed_frame.empty())
  {
    manager_->setFixedFrame(fixed_frame);
  }

  if (!target_frame.empty())
  {
    manager_->setTargetFrame(target_frame);
  }

  wxString auimanager_perspective;
  long version = 0;
  if (general_config_->Read(CONFIG_AUIMANAGER_PERSPECTIVE_VERSION, &version))
  {
    if (version >= PERSPECTIVE_VERSION)
    {
      if (general_config_->Read(CONFIG_AUIMANAGER_PERSPECTIVE, &auimanager_perspective))
      {
        aui_manager_->LoadPerspective(auimanager_perspective);
        aui_manager_->Update();
      }
    }
    else
    {
      ROS_INFO("Perspective version has changed (version [%d] is less than version [%d], resetting windows", (int)version, PERSPECTIVE_VERSION);
    }
  }

  initMenus();
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
  general_config_.reset(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(general_config_file_.c_str())));
  ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
  display_config_.reset(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(display_config_file_.c_str())));
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

  view_menu_ = new wxMenu(wxT(""));
  wxAuiPaneInfoArray& panes = aui_manager_->GetAllPanes();
  for (uint32_t i = 0; i < panes.GetCount(); ++i)
  {
    wxAuiPaneInfo& pane = panes.Item(i);

    if (pane.HasCloseButton())
    {
      item = view_menu_->AppendCheckItem(pane.window->GetId(), pane.caption);
      item->Check(pane.IsShown());
      Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onViewMenuItemSelected), NULL, this);
    }
  }

  menubar_->Append(view_menu_, wxT("&View"));

  plugins_menu_ = new wxMenu(wxT(""));
  item = plugins_menu_->Append(wxID_ANY, wxT("&Manage..."));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onManagePlugins), NULL, this);
  menubar_->Append(plugins_menu_, wxT("&Plugins"));

  SetMenuBar(menubar_);

  loadConfigMenus();
}

void VisualizationFrame::loadDisplayConfig(const std::string& path)
{
  manager_->removeAllDisplays();

  boost::shared_ptr<wxFileConfig> config(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxEmptyString, wxString::FromAscii(path.c_str()), wxCONFIG_USE_GLOBAL_FILE));
  manager_->loadDisplayConfig(config);
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
  general_config_->Write(CONFIG_AUIMANAGER_PERSPECTIVE_VERSION, PERSPECTIVE_VERSION);

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

void VisualizationFrame::onPaneClosed(wxAuiManagerEvent& event)
{
  wxAuiPaneInfo* pane = event.GetPane();
  wxWindow* window = pane->window;
  menubar_->Check(window->GetId(), false);
}

void VisualizationFrame::onViewMenuItemSelected(wxCommandEvent& event)
{
  wxAuiPaneInfoArray& panes = aui_manager_->GetAllPanes();
  for (uint32_t i = 0; i < panes.GetCount(); ++i)
  {
    wxAuiPaneInfo& pane = panes.Item(i);

    if (pane.window->GetId() == event.GetId())
    {
      pane.Show(event.IsChecked());

      aui_manager_->Update();

      break;
    }
  }
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
    if (path.extension() != "."CONFIG_EXTENSION)
    {
      filename += "."CONFIG_EXTENSION;
    }

    boost::shared_ptr<wxFileConfig> config(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(filename.c_str())));
    config->DeleteAll();

    manager_->saveDisplayConfig(config);
    config->Flush();

    loadConfigMenus();
  }
}

void VisualizationFrame::onGlobalConfig(wxCommandEvent& event)
{
  wxMenuItem* item = menubar_->FindItem(event.GetId());
  std::string filename = (const char*)item->GetLabel().fn_str();

  // wx(gtk?) for some reason adds an extra underscore for each one it finds
  size_t pos = filename.find("__");
  while (pos != std::string::npos)
  {
    filename.erase(pos, 1);
    pos = filename.find("__");
  }

  fs::path path(global_config_dir_);
  path /= filename + "." + CONFIG_EXTENSION;

  loadDisplayConfig(path.file_string());
}

void VisualizationFrame::onLocalConfig(wxCommandEvent& event)
{
  wxMenuItem* item = menubar_->FindItem(event.GetId());
  std::string filename = (const char*)item->GetLabel().fn_str();

  // wx(gtk?) for some reason adds an extra underscore for each one it finds
  size_t pos = filename.find("__");
  while (pos != std::string::npos)
  {
    filename.erase(pos, 1);
    pos = filename.find("__");
  }

  fs::path path(save_dir_);
  path /= filename + "." + CONFIG_EXTENSION;

  loadDisplayConfig(path.file_string());
}

void VisualizationFrame::onToolAdded(Tool* tool)
{
#if !defined(__WXMAC__)
  char ascii_str[2] = { tool->getShortcutKey(), 0 };
  wxString tooltip = wxString( wxT("Shortcut Key: ")) + wxString::FromAscii( ascii_str );
  toolbar_->AddRadioTool( toolbar_->GetToolsCount(), wxString::FromAscii( tool->getName().c_str() ), wxNullBitmap, wxNullBitmap, tooltip );

  wxAuiPaneInfo& pane = aui_manager_->GetPane(toolbar_);
  pane.MinSize(toolbar_->GetSize());
  aui_manager_->Update();
#endif
}

void VisualizationFrame::onToolChanged(Tool* tool)
{
#if !defined(__WXMAC__)
  int count = toolbar_->GetToolsCount();
  for ( int i = toolbar_items::Count; i < count; ++i )
  {
    if ( manager_->getTool( i - toolbar_items::Count ) == tool )
    {
      toolbar_->ToggleTool( i, true );
      break;
    }
  }
#endif
}

void VisualizationFrame::onToolClicked( wxCommandEvent& event )
{
  int id = event.GetId();
  if (id >= toolbar_items::Count)
  {
    Tool* tool = manager_->getTool( id - toolbar_items::Count );

    manager_->setCurrentTool( tool );
  }
  else
  {
    switch (id)
    {
    default:
      break;
    }
  }
}

void VisualizationFrame::onManagePlugins(wxCommandEvent& event)
{
  PluginManagerDialog dialog(this, manager_->getPluginManager());
  dialog.ShowModal();
}

wxWindow* VisualizationFrame::getParentWindow()
{
  return this;
}

void VisualizationFrame::addPane(const std::string& name, wxWindow* panel)
{
  aui_manager_->AddPane(panel, wxAuiPaneInfo().Float().BestSize(panel->GetSize()).Name(wxString::FromAscii(name.c_str())).Caption(wxString::FromAscii(name.c_str())).CloseButton(false).Show(false).Dockable(false));
  aui_manager_->Update();
}

void VisualizationFrame::removePane(wxWindow* panel)
{
  aui_manager_->DetachPane(panel);
  aui_manager_->Update();
}

void VisualizationFrame::showPane(wxWindow* panel)
{
  aui_manager_->GetPane(panel).Show(true);
  aui_manager_->Update();
}

void VisualizationFrame::closePane(wxWindow* panel)
{
  aui_manager_->GetPane(panel).Show(false);
  aui_manager_->Update();
}


}
