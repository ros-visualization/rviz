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
#include "tool_properties_panel.h"
#include "visualization_manager.h"
#include "tools/tool.h"
#include "plugin_manager_dialog.h"
#include "splash_screen.h"
#include "loading_dialog.h"
#include "common.h"

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
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

namespace fs = boost::filesystem;

#define CONFIG_WINDOW_X wxT("/Window/X")
#define CONFIG_WINDOW_Y wxT("/Window/Y")
#define CONFIG_WINDOW_WIDTH wxT("/Window/Width")
#define CONFIG_WINDOW_HEIGHT wxT("/Window/Height")
#define CONFIG_AUIMANAGER_PERSPECTIVE wxT("/AuiManagerPerspective")
#define CONFIG_AUIMANAGER_PERSPECTIVE_VERSION wxT("/AuiManagerPerspectiveVersion")
#define CONFIG_RECENT_CONFIGS wxT("/RecentConfigs")
#define CONFIG_LAST_DIR wxT("/LastConfigDir")

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
#define PERSPECTIVE_VERSION 2

#define RECENT_CONFIG_COUNT 10

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
, recent_configs_menu_(NULL)
, aui_manager_(NULL)
{
	wxInitAllImageHandlers();
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

void VisualizationFrame::onSplashLoadStatus(const std::string& status, SplashScreen* splash)
{
  splash->setState(status);
}

void VisualizationFrame::initialize(const std::string& display_config_file, const std::string& fixed_frame, const std::string& target_frame)
{
  initConfigs();

  wxPoint pos = GetPosition();
  wxSize size = GetSize();
  int width = size.GetWidth();
  int height = size.GetHeight();
  general_config_->Read(CONFIG_WINDOW_X, &pos.x, pos.x);
  general_config_->Read(CONFIG_WINDOW_Y, &pos.y, pos.y);
  general_config_->Read(CONFIG_WINDOW_WIDTH, &width, width);
  general_config_->Read(CONFIG_WINDOW_HEIGHT, &height, height);

  {
    wxString str;
    if (general_config_->Read(CONFIG_RECENT_CONFIGS, &str))
    {
      std::string recent = (const char*)str.char_str();

      boost::trim(recent);
      boost::split(recent_configs_, recent, boost::is_any_of (":"), boost::token_compress_on);
    }

    if (general_config_->Read(CONFIG_LAST_DIR, &str))
    {
      last_config_dir_ = (const char*)str.char_str();
    }
  }

  SetPosition(pos);
  SetSize(wxSize(width, height));

  package_path_ = ros::package::getPath("rviz");
  std::string splash_path = (fs::path(package_path_) / "images/splash.png").file_string();
  wxBitmap splash;
  splash.LoadFile(wxString::FromAscii(splash_path.c_str()));
  splash_ = new SplashScreen(this, splash);
  splash_->Show();
  splash_->setState("Initializing");

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
  tool_properties_panel_ = new ToolPropertiesPanel(this);

  splash_->setState("Initializing OGRE resources");
  ogre_tools::V_string paths;
  paths.push_back(package_path_ + "/ogre_media/textures");
  ogre_tools::initializeResources( paths );

#if !defined(__WXMAC__)
  toolbar_ = new wxToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_TEXT|wxTB_NOICONS|wxNO_BORDER|wxTB_HORIZONTAL);
  toolbar_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationFrame::onToolClicked ), NULL, this );
#endif

  aui_manager_ = new wxAuiManager(this);
  aui_manager_->AddPane(render_panel_, wxAuiPaneInfo().CenterPane().Name(wxT("Render")));
  aui_manager_->AddPane(displays_panel_, wxAuiPaneInfo().Left().MinSize(270, -1).Name(wxT("Displays")).Caption(wxT("Displays")));
  aui_manager_->AddPane(selection_panel_, wxAuiPaneInfo().Right().MinSize(270, -1).Name(wxT("Selection")).Caption(wxT("Selection")));
  aui_manager_->AddPane(views_panel_, wxAuiPaneInfo().BestSize(230, 200).Right().Name(wxT("Views")).Caption(wxT("Views")));
  aui_manager_->AddPane(tool_properties_panel_, wxAuiPaneInfo().BestSize(230, 200).Right().Name(wxT("Tool Properties")).Caption(wxT("Tool Properties")));
  aui_manager_->AddPane(time_panel_, wxAuiPaneInfo().RightDockable(false).LeftDockable(false).Bottom().Name(wxT("Time")).Caption(wxT("Time")));
#if !defined(__WXMAC__)
  aui_manager_->AddPane(toolbar_, wxAuiPaneInfo().ToolbarPane().RightDockable(false).LeftDockable(false)/*.MinSize(-1, 40)*/.Top().Name(wxT("Tools")).Caption(wxT("Tools")));
#endif
  aui_manager_->Update();

  Connect(wxEVT_AUI_PANE_CLOSE, wxAuiManagerEventHandler(VisualizationFrame::onPaneClosed), NULL, this);

  manager_ = new VisualizationManager(render_panel_, this);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  displays_panel_->initialize(manager_);
  views_panel_->initialize(manager_);
  time_panel_->initialize(manager_);
  selection_panel_->initialize(manager_);
  tool_properties_panel_->initialize(manager_);

  manager_->getToolAddedSignal().connect( boost::bind( &VisualizationFrame::onToolAdded, this, _1 ) );
  manager_->getToolChangedSignal().connect( boost::bind( &VisualizationFrame::onToolChanged, this, _1 ) );

  manager_->initialize();
  manager_->loadGeneralConfig(general_config_, boost::bind(&VisualizationFrame::onSplashLoadStatus, this, _1, splash_));

  bool display_config_valid = !display_config_file.empty();
  if (display_config_valid && !fs::exists(display_config_file))
  {
    ROS_ERROR("File [%s] does not exist", display_config_file.c_str());
    display_config_valid = false;
  }

  if (!display_config_valid)
  {
    manager_->loadDisplayConfig(display_config_, boost::bind(&VisualizationFrame::onSplashLoadStatus, this, _1, splash_));
  }
  else
  {
    boost::shared_ptr<wxFileConfig> config(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxEmptyString, wxString::FromAscii(display_config_file.c_str()), wxCONFIG_USE_GLOBAL_FILE|wxCONFIG_USE_RELATIVE_PATH));
    manager_->loadDisplayConfig(config, boost::bind(&VisualizationFrame::onSplashLoadStatus, this, _1, splash_));
  }

  if (!fixed_frame.empty())
  {
    manager_->setFixedFrame(fixed_frame);
  }

  if (!target_frame.empty())
  {
    manager_->setTargetFrame(target_frame);
  }

  splash_->setState("Loading perspective");

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
  updateRecentConfigMenu();
  if (display_config_valid)
  {
    markRecentConfig(display_config_file);
  }

  splash_->Destroy();
  splash_ = 0;

  manager_->startUpdate();
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

  ROS_INFO("Loading general config from [%s]", general_config_file_.c_str());
  general_config_.reset(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(general_config_file_.c_str())));
  ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
  display_config_.reset(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxString::FromAscii(display_config_file_.c_str())));
}

void VisualizationFrame::initMenus()
{
  menubar_ = new wxMenuBar();
  file_menu_ = new wxMenu(wxT(""));
  wxMenuItem* item = file_menu_->Append(wxID_OPEN, wxT("&Open Config\tCtrl-O"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onOpen), NULL, this);
  item = file_menu_->Append(wxID_SAVE, wxT("&Save Config\tCtrl-S"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onSave), NULL, this);

  recent_configs_menu_ = new wxMenu(wxT(""));
  file_menu_->Append(wxID_ANY, wxT("&Recent Configs"), recent_configs_menu_);

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

  help_menu_ = new wxMenu(wxT(""));
  item = help_menu_->Append(wxID_ANY, wxT("&Wiki"));
  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onHelpWiki), NULL, this);
  menubar_->Append(help_menu_, wxT("&Help"));

  SetMenuBar(menubar_);
}

void VisualizationFrame::updateRecentConfigMenu()
{
  // wtf.  no Clear method
  while (recent_configs_menu_->GetMenuItemCount() > 0)
  {
    wxMenuItem* item = recent_configs_menu_->FindItemByPosition(0);
    recent_configs_menu_->Remove(item);
  }

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it)
  {
    const std::string& path = *it;
    wxMenuItem* item = recent_configs_menu_->Append(wxID_ANY, wxString::FromAscii(path.c_str()));
    Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onRecentConfigSelected), NULL, this);
  }
}

void VisualizationFrame::markRecentConfig(const std::string& path)
{
  D_string::iterator it = std::find(recent_configs_.begin(), recent_configs_.end(), path);
  if (it != recent_configs_.end())
  {
    recent_configs_.erase(it);
  }

  recent_configs_.push_front(path);

  if (recent_configs_.size() > RECENT_CONFIG_COUNT)
  {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void VisualizationFrame::loadDisplayConfig(const std::string& path)
{
  if (!fs::exists(path))
  {
    wxString message = wxString::FromAscii(path.c_str()) + wxT(" does not exist!");
    wxMessageBox(message, wxT("Config Does Not Exist"), wxOK|wxICON_ERROR, this);
    return;
  }

  manager_->removeAllDisplays();

  LoadingDialog dialog(this);
  dialog.Show();

  boost::shared_ptr<wxFileConfig> config(new wxFileConfig(wxT("standalone_visualizer"), wxEmptyString, wxEmptyString, wxString::FromAscii(path.c_str()), wxCONFIG_USE_GLOBAL_FILE));
  manager_->loadDisplayConfig(config, boost::bind(&LoadingDialog::setState, &dialog, _1));

  markRecentConfig(path);
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

  {
    std::stringstream ss;
    D_string::iterator it = recent_configs_.begin();
    D_string::iterator end = recent_configs_.end();
    for (; it != end; ++it)
    {
      if (it != recent_configs_.begin())
      {
        ss << ":";
      }
      ss << *it;
    }

    wxString str = wxString::FromAscii(ss.str().c_str());
    general_config_->Write(CONFIG_RECENT_CONFIGS, str);
  }

  general_config_->Write(CONFIG_LAST_DIR, wxString::FromAscii(last_config_dir_.c_str()));

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
  wxString wxstr_file = wxFileSelector(wxT("Choose a file to open"), wxString::FromAscii(last_config_dir_.c_str()), wxEmptyString,
                                       wxT(CONFIG_EXTENSION), wxT(CONFIG_EXTENSION_WILDCARD), wxFD_OPEN|wxFD_FILE_MUST_EXIST, this);
  if (!wxstr_file.empty())
  {
    std::string filename = (const char*)wxstr_file.fn_str();
    loadDisplayConfig(filename);

    last_config_dir_ = fs::path(filename).parent_path().string();
  }
}

void VisualizationFrame::onSave(wxCommandEvent& event)
{
  wxString wxstr_file = wxFileSelector(wxT("Choose a file to save to"), wxString::FromAscii(last_config_dir_.c_str()), wxEmptyString,
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

    markRecentConfig(filename);

    last_config_dir_ = fs::path(filename).parent_path().string();
  }
}

void VisualizationFrame::onRecentConfigSelected(wxCommandEvent& event)
{
  wxString label = recent_configs_menu_->GetLabel(event.GetId());
  if (!label.IsEmpty())
  {
    std::string path = (const char*)label.char_str();

    // wx(gtk?) for some reason adds an extra underscore for each one it finds in a menu item
    size_t pos = path.find("__");
    while (pos != std::string::npos)
    {
      path.erase(pos, 1);
      pos = path.find("__", pos + 1);
    }

    loadDisplayConfig(path);
  }
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

void VisualizationFrame::onHelpWiki(wxCommandEvent& event)
{
  wxLaunchDefaultBrowser(wxT("http://www.ros.org/wiki/rviz"));
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
