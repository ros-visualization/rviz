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

#include <QApplication>
#include <QSplashScreen>
#include <QDockWidget>
#include <QDir>
#include <QCloseEvent>
#include <QToolBar>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QFileDialog>
#include <QDesktopServices>
#include <QUrl>
#include <QAction>

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <ros/package.h>
#include <ros/console.h>

#include <OGRE/OgreRenderWindow.h>

#include <ogre_helpers/initialization.h>

#include "visualization_frame.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "views_panel.h"
#include "time_panel.h"
#include "selection_panel.h"
#include "tool_properties_panel.h"
#include "visualization_manager.h"
#include "tools/tool.h"
#include "loading_dialog.h"
#include "config.h"
#include "panel_dock_widget.h"
#include "new_object_dialog.h"
#include "panel.h"
#include "screenshot_dialog.h"
#include "help_panel.h"

namespace fs = boost::filesystem;

#define CONFIG_WINDOW_X "/Window/X"
#define CONFIG_WINDOW_Y "/Window/Y"
#define CONFIG_WINDOW_WIDTH "/Window/Width"
#define CONFIG_WINDOW_HEIGHT "/Window/Height"
// I am not trying to preserve peoples' window layouts from wx to Qt,
// just saving the Qt layout in a new config tag.
#define CONFIG_QMAINWINDOW "/QMainWindow"
#define CONFIG_AUIMANAGER_PERSPECTIVE "/AuiManagerPerspective"
#define CONFIG_AUIMANAGER_PERSPECTIVE_VERSION "/AuiManagerPerspectiveVersion"
#define CONFIG_RECENT_CONFIGS "/RecentConfigs"
#define CONFIG_LAST_DIR "/LastConfigDir"

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
#define PERSPECTIVE_VERSION 2

#define RECENT_CONFIG_COUNT 10

#if BOOST_FILESYSTEM_VERSION == 3
#define BOOST_FILE_STRING string
#else
#define BOOST_FILE_STRING file_string
#endif

namespace rviz
{

VisualizationFrame::VisualizationFrame( QWidget* parent )
  : QMainWindow( parent )
  , render_panel_(NULL)
  , displays_panel_(NULL)
  , views_panel_(NULL)
  , time_panel_(NULL)
  , selection_panel_(NULL)
  , tool_properties_panel_(NULL)
  , help_panel_(NULL)
  , show_help_action_(NULL)
  , file_menu_(NULL)
  , recent_configs_menu_(NULL)
  , toolbar_(NULL)
  , manager_(NULL)
  , position_correction_( 0, 0 )
  , num_move_events_( 0 )
  , toolbar_actions_( NULL )
{
  setWindowTitle( "RViz" );

  panel_class_loader_ = new pluginlib::ClassLoader<Panel>( "rviz", "rviz::Panel" );
}

VisualizationFrame::~VisualizationFrame()
{
  if( manager_ )
  {
    manager_->removeAllDisplays();
  }

  delete render_panel_;
  delete manager_;

  M_PanelRecord::iterator pi;
  for( pi = custom_panels_.begin(); pi != custom_panels_.end(); pi++ )
  {
    delete (*pi).second.dock;
  }

  delete panel_class_loader_;
}

void VisualizationFrame::closeEvent( QCloseEvent* event )
{
  if( general_config_ )
  {
    saveConfigs();
  }
  event->accept();
}

void VisualizationFrame::changeMaster()
{
  if( general_config_ )
  {
    saveConfigs();
  }
  QApplication::exit( 255 );
}

void VisualizationFrame::setSplashStatus( const std::string& status )
{
  splash_->showMessage( QString::fromStdString( status ), Qt::AlignLeft | Qt::AlignBottom );
}

void VisualizationFrame::initialize(const std::string& display_config_file,
                                    const std::string& fixed_frame,
                                    const std::string& target_frame,
                                    const std::string& splash_path,
                                    const std::string& help_path,
                                    bool verbose,
                                    bool show_choose_new_master_option )
{
  show_choose_new_master_option_ = show_choose_new_master_option;

  initConfigs();

  {
    std::string recent;
    if( general_config_->get( CONFIG_RECENT_CONFIGS, &recent ))
    {
      boost::trim( recent );
      boost::split( recent_configs_, recent, boost::is_any_of (":"), boost::token_compress_on );
    }

    general_config_->get( CONFIG_LAST_DIR, &last_config_dir_ );
  }

  package_path_ = ros::package::getPath("rviz");

  std::string final_splash_path = splash_path;

  if ( splash_path.empty() )
  {
    final_splash_path = (fs::path(package_path_) / "images/splash.png").BOOST_FILE_STRING();
  }

  help_path_ = help_path;
  if ( help_path_.empty() )
  {
    help_path_ = (fs::path(package_path_) / "help/help.html").BOOST_FILE_STRING();
  }
  QPixmap splash_image( QString::fromStdString( final_splash_path ));
  splash_ = new QSplashScreen( splash_image );
  splash_->show();
  setSplashStatus( "Initializing" );

  if( !ros::isInitialized() )
  {
    int argc = 0;
    ros::init( argc, 0, "rviz", ros::init_options::AnonymousName );
  }

  render_panel_ = new RenderPanel( this );
  displays_panel_ = new DisplaysPanel( this );
  views_panel_ = new ViewsPanel( this );
  time_panel_ = new TimePanel( this );
  selection_panel_ = new SelectionPanel( this );
  tool_properties_panel_ = new ToolPropertiesPanel( this );

  setSplashStatus( "Initializing OGRE resources" );
  V_string paths;
  paths.push_back( package_path_ + "/ogre_media/textures" );
  initializeResources( paths );

  initMenus();
  toolbar_ = addToolBar( "Tools" );
  toolbar_->setObjectName( "Tools" );
  toolbar_actions_ = new QActionGroup( this );
  connect( toolbar_actions_, SIGNAL( triggered( QAction* )), this, SLOT( onToolbarActionTriggered( QAction* )));
  view_menu_->addAction( toolbar_->toggleViewAction() );

  setCentralWidget( render_panel_ );

  addPane( "Displays", displays_panel_, Qt::LeftDockWidgetArea, false );
  addPane( "Tool Properties", tool_properties_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Views", views_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Selection", selection_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Time", time_panel_, Qt::BottomDockWidgetArea, false );

  manager_ = new VisualizationManager( render_panel_, this );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  displays_panel_->initialize( manager_ );
  views_panel_->initialize( manager_ );
  time_panel_->initialize(manager_);
  selection_panel_->initialize( manager_ );
  tool_properties_panel_->initialize( manager_ );

  connect( manager_, SIGNAL( toolAdded( Tool* )), this, SLOT( addTool( Tool* )));
  connect( manager_, SIGNAL( toolChanged( Tool* )), this, SLOT( indicateToolIsCurrent( Tool* )));

  manager_->initialize( StatusCallback(), verbose );

  bool display_config_valid = !display_config_file.empty();
  if( display_config_valid && !fs::exists( display_config_file ))
  {
    ROS_ERROR("File [%s] does not exist", display_config_file.c_str());
    display_config_valid = false;
  }

  if( !display_config_valid )
  {
    loadDisplayConfig( display_config_, boost::bind( &VisualizationFrame::setSplashStatus, this, _1 ));
  }
  else
  {
    boost::shared_ptr<Config> config( new Config );
    config->readFromFile( display_config_file ); 
    loadDisplayConfig( config, boost::bind( &VisualizationFrame::setSplashStatus, this, _1 ));
  }

  if( !fixed_frame.empty() )
  {
    manager_->setFixedFrame( fixed_frame );
  }

  if( !target_frame.empty() )
  {
    manager_->setTargetFrame( target_frame );
  }

  setSplashStatus( "Loading perspective" );

  updateRecentConfigMenu();
  if( display_config_valid )
  {
    markRecentConfig( display_config_file );
  }

  delete splash_;
  splash_ = 0;

  manager_->startUpdate();
}

void VisualizationFrame::initConfigs()
{
  std::string home_dir = QDir::toNativeSeparators( QDir::homePath() ).toStdString();

  config_dir_ = (fs::path(home_dir) / ".rviz").BOOST_FILE_STRING();
  general_config_file_ = (fs::path(config_dir_) / "config").BOOST_FILE_STRING();
  display_config_file_ = (fs::path(config_dir_) / "display_config").BOOST_FILE_STRING();

  if( fs::is_regular_file( config_dir_ ))
  {
    ROS_ERROR("Moving file [%s] out of the way to recreate it as a directory.", config_dir_.c_str());
    std::string backup_file = config_dir_ + ".bak";

    fs::rename(config_dir_, backup_file);
    fs::create_directory(config_dir_);
  }
  else if (!fs::exists(config_dir_))
  {
    fs::create_directory(config_dir_);
  }

  ROS_INFO("Loading general config from [%s]", general_config_file_.c_str());
  general_config_.reset( new Config );
  general_config_->readFromFile( general_config_file_ );

  ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
  display_config_.reset( new Config );
  display_config_->readFromFile( display_config_file_ );
}

void VisualizationFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu( "&File" );
  file_menu_->addAction( "&Open Config", this, SLOT( onOpen() ), QKeySequence( "Ctrl+O" ));
  file_menu_->addAction( "&Save Config", this, SLOT( onSave() ), QKeySequence( "Ctrl+S" ));
  recent_configs_menu_ = file_menu_->addMenu( "&Recent Configs" );
  file_menu_->addAction( "Save &Image", this, SLOT( onSaveImage() ));
  if( show_choose_new_master_option_ )
  {
    file_menu_->addSeparator();
    file_menu_->addAction( "Change &Master", this, SLOT( changeMaster() ));
  }
  file_menu_->addSeparator();
  file_menu_->addAction( "&Quit", this, SLOT( close() ), QKeySequence( "Ctrl+Q" ));

  view_menu_ = menuBar()->addMenu( "&Panels" );
  view_menu_->addAction( "Add &New Panel", this, SLOT( openNewPanelDialog() ));
  delete_view_menu_ = view_menu_->addMenu( "&Delete Panel" );
  delete_view_menu_->setEnabled( false );
  view_menu_->addSeparator();

/////  plugins_menu_ = new wxMenu("");
/////  item = plugins_menu_->Append(wxID_ANY, "&Manage...");
/////  Connect(item->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(VisualizationFrame::onManagePlugins), NULL, this);
/////  menubar_->Append(plugins_menu_, "&Plugins");
/////

  QMenu* help_menu = menuBar()->addMenu( "&Help" );
  help_menu->addAction( "Show &Help panel", this, SLOT( showHelpPanel() ));
  help_menu->addAction( "Open rviz wiki in browser", this, SLOT( onHelpWiki() ));
}

void VisualizationFrame::openNewPanelDialog()
{
  std::string lookup_name;
  std::string display_name;

  NewObjectDialog* dialog = new NewObjectDialog( panel_class_loader_,
                                                 "Panel",
                                                 panel_names_,
                                                 &lookup_name,
                                                 &display_name );
  if( dialog->exec() == QDialog::Accepted )
  {
    addCustomPanel( display_name, lookup_name );
  }
  activateWindow(); // Force keyboard focus back on main window.
}

void VisualizationFrame::updateRecentConfigMenu()
{
  recent_configs_menu_->clear();

  D_string::iterator it = recent_configs_.begin();
  D_string::iterator end = recent_configs_.end();
  for (; it != end; ++it)
  {
    if( *it != "" )
    {
      recent_configs_menu_->addAction( QString::fromStdString( *it ), this, SLOT( onRecentConfigSelected() ));
    }
  }
}

void VisualizationFrame::markRecentConfig( const std::string& path )
{
  D_string::iterator it = std::find( recent_configs_.begin(), recent_configs_.end(), path );
  if( it != recent_configs_.end() )
  {
    recent_configs_.erase( it );
  }

  recent_configs_.push_front( path );

  if( recent_configs_.size() > RECENT_CONFIG_COUNT )
  {
    recent_configs_.pop_back();
  }

  updateRecentConfigMenu();
}

void VisualizationFrame::loadDisplayConfig( const std::string& path )
{
  if( !fs::exists( path ))
  {
    QString message = QString::fromStdString( path  ) + " does not exist!";
    QMessageBox::critical( this, "Config file does not exist", message );
    return;
  }

  manager_->removeAllDisplays();

  LoadingDialog dialog( this );
  dialog.show();

  boost::shared_ptr<Config> config( new Config );
  config->readFromFile( path );
  loadDisplayConfig( config, boost::bind( &LoadingDialog::setState, &dialog, _1 ));
  markRecentConfig(path);
}

void VisualizationFrame::loadDisplayConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb )
{
  manager_->loadDisplayConfig( config, cb );
  loadCustomPanels( config );
  loadWindowGeometry( config );
}

void VisualizationFrame::saveDisplayConfig( const boost::shared_ptr<Config>& config )
{
  manager_->saveDisplayConfig( config );
  saveCustomPanels( config );
  saveWindowGeometry( config );
}

void VisualizationFrame::loadCustomPanels( const boost::shared_ptr<Config>& config )
{
  // First destroy any existing custom panels.
  M_PanelRecord::iterator pi;
  for( pi = custom_panels_.begin(); pi != custom_panels_.end(); pi++ )
  {
    delete (*pi).second.dock;
    delete (*pi).second.delete_action;
  }
  custom_panels_.clear();

  // Then load the ones in the config.
  int i = 0;
  while( true )
  {
    std::stringstream panel_prefix, panel_name_ss, lookup_name_ss;
    panel_prefix << "Panel" << i;
    panel_name_ss << "Panel" << i << "/Name";
    lookup_name_ss << "Panel" << i << "/ClassLookupName";

    std::string panel_name, lookup_name;
    if( !config->get( panel_name_ss.str(), &panel_name ))
    {
      break;
    }

    if( !config->get( lookup_name_ss.str(), &lookup_name ))
    {
      break;
    }

    PanelDockWidget* dock = addCustomPanel( panel_name, lookup_name );
    if( dock )
    {
      Panel* panel = qobject_cast<Panel*>( dock->widget() );
      if( panel )
      {
        panel->loadFromConfig( panel_prefix.str(), config );
      }
    }

    ++i;
  }
}

void VisualizationFrame::saveCustomPanels( const boost::shared_ptr<Config>& config )
{
  int i = 0;
  M_PanelRecord::iterator pi;
  for( pi = custom_panels_.begin(); pi != custom_panels_.end(); pi++, i++ )
  {
    PanelRecord record = (*pi).second;
    std::stringstream panel_prefix, panel_name_key, lookup_name_key;
    panel_prefix << "Panel" << i;
    panel_name_key << "Panel" << i << "/Name";
    lookup_name_key << "Panel" << i << "/ClassLookupName";
    config->set( panel_name_key.str(), record.name );
    config->set( lookup_name_key.str(), record.lookup_name );
    record.panel->saveToConfig( panel_prefix.str(), config );
  }
}

void VisualizationFrame::moveEvent( QMoveEvent* event )
{
//  GdkRectangle rect;
//  GdkWindow* gdk_window = gdk_window_foreign_new( winId() ); 
//  gdk_window_get_frame_extents( gdk_window, &rect );
//  printf( "gdk x=%d, y=%d\n", rect.x, rect.y );
// the above works!  should I just use gdk??

  // HACK to work around a bug in Qt-for-X11.  The first time we get a
  // moveEvent, the position is that of the top-left corner of the
  // window frame.  The second time we get one, the position is the
  // top-left corner *inside* the frame.  There is no significant time
  // lag between the two calls, certainly no user events, so I just
  // remember the first position and diff it with the second position
  // and remember the diff as a corrective offset for future geometry
  // requests.
  //
  // This seems like it would be brittle to OS, code changes, etc, so
  // sometime I should get something better going here.  Maybe call
  // out to gdk (as above), which seems to work right.
  switch( num_move_events_ )
  {
  case 0:
    first_position_ = pos();
    num_move_events_++;
    break;
  case 1:
    position_correction_ = first_position_ - pos();
    num_move_events_++;
    break;
  }
}

QRect VisualizationFrame::hackedFrameGeometry()
{
  QRect geom = frameGeometry();
  geom.moveTopLeft( pos() + position_correction_ );
  return geom;
}

void VisualizationFrame::loadWindowGeometry( const boost::shared_ptr<Config>& config )
{
  int new_x, new_y, new_width, new_height;
  config->get( CONFIG_WINDOW_X, &new_x, x() );
  config->get( CONFIG_WINDOW_Y, &new_y, y() );
  config->get( CONFIG_WINDOW_WIDTH, &new_width, width() );
  config->get( CONFIG_WINDOW_HEIGHT, &new_height, height() );

  move( new_x, new_y );
  resize( new_width, new_height );

  std::string main_window_config;
  if( config->get( CONFIG_QMAINWINDOW, &main_window_config ))
  {
    restoreState( QByteArray::fromHex( main_window_config.c_str() ));
  }
}

void VisualizationFrame::saveWindowGeometry( const boost::shared_ptr<Config>& config )
{
  QRect geom = hackedFrameGeometry();
  config->set( CONFIG_WINDOW_X, geom.x() );
  config->set( CONFIG_WINDOW_Y, geom.y() );
  config->set( CONFIG_WINDOW_WIDTH, geom.width() );
  config->set( CONFIG_WINDOW_HEIGHT, geom.height() );

  QByteArray window_state = saveState().toHex();
  config->set( CONFIG_QMAINWINDOW, std::string( window_state.constData() ));
}

void VisualizationFrame::saveConfigs()
{
  ROS_INFO("Saving general config to [%s]", general_config_file_.c_str());
  general_config_->clear();
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

    general_config_->set( CONFIG_RECENT_CONFIGS, ss.str() );
  }

  general_config_->set( CONFIG_LAST_DIR, last_config_dir_ );

  general_config_->writeToFile( general_config_file_ );

  ROS_INFO( "Saving display config to [%s]", display_config_file_.c_str() );
  display_config_->clear();
  saveDisplayConfig( display_config_ );
  display_config_->writeToFile( display_config_file_ );
}

void VisualizationFrame::onOpen()
{
  QString filename = QFileDialog::getOpenFileName( this, "Choose a file to open",
                                                   QString::fromStdString( last_config_dir_ ),
                                                   "RViz config files (" CONFIG_EXTENSION_WILDCARD ")" );

  if( !filename.isEmpty() )
  {
    std::string filename_string = filename.toStdString();
    loadDisplayConfig( filename_string );
    last_config_dir_ = fs::path( filename_string ).parent_path().BOOST_FILE_STRING();
  }
}

void VisualizationFrame::onSave()
{
  QString q_filename = QFileDialog::getSaveFileName( this, "Choose a file to save to",
                                                     QString::fromStdString( last_config_dir_ ),
                                                     "RViz config files (" CONFIG_EXTENSION_WILDCARD ")" );

  if( !q_filename.isEmpty() )
  {
    std::string filename = q_filename.toStdString();
    fs::path path( filename );
    if( path.extension() != "."CONFIG_EXTENSION )
    {
      filename += "."CONFIG_EXTENSION;
    }

    boost::shared_ptr<Config> config( new Config() );
    saveDisplayConfig( config );
    config->writeToFile( filename );

    markRecentConfig( filename );

    last_config_dir_ = fs::path( filename ).parent_path().BOOST_FILE_STRING();
  }
}

void VisualizationFrame::onSaveImage()
{
  ScreenshotDialog* dialog = new ScreenshotDialog( this, render_panel_ );
  dialog->show();
}

void VisualizationFrame::onRecentConfigSelected()
{
  QAction* action = dynamic_cast<QAction*>( sender() );
  if( action )
  {
    std::string path = action->text().toStdString();
    if( !path.empty() )
    {
      loadDisplayConfig( path );
    }
  }
}

void VisualizationFrame::addTool( Tool* tool )
{
  QAction* action = new QAction( QString::fromStdString( tool->getName() ), toolbar_actions_ );
  action->setCheckable( true );
  action->setShortcut( QKeySequence( QString( tool->getShortcutKey() )));
  toolbar_->addAction( action );
  action_to_tool_map_[ action ] = tool;
  tool_to_action_map_[ tool ] = action;
}

void VisualizationFrame::onToolbarActionTriggered( QAction* action )
{
  Tool* tool = action_to_tool_map_[ action ];
  if( tool )
  {
    manager_->setCurrentTool( tool );
  }
}

void VisualizationFrame::indicateToolIsCurrent( Tool* tool )
{
  QAction* action = tool_to_action_map_[ tool ];
  if( action )
  {
    action->setChecked( true );
  }
}

void VisualizationFrame::showHelpPanel()
{
  if( !show_help_action_ )
  {
    help_panel_ = new HelpPanel( this );
    QDockWidget* dock = addPane( "Help", help_panel_ );
    show_help_action_ = dock->toggleViewAction();
  }
  else
  {
    // show_help_action_ is a toggle action, so trigger() changes its
    // state.  Therefore we must force it to the opposite state from
    // what we want before we call trigger().  (I think.)
    show_help_action_->setChecked( false );
    show_help_action_->trigger();
  }
  help_panel_->setHelpFile( help_path_ );
}

void VisualizationFrame::onHelpWiki()
{
  QDesktopServices::openUrl( QUrl( "http://www.ros.org/wiki/rviz" ));
}

QWidget* VisualizationFrame::getParentWindow()
{
  return this;
}

void VisualizationFrame::onDeletePanel()
{
  if( QAction* action = qobject_cast<QAction*>( sender() ))
  {
    std::string panel_name = action->text().toStdString();
    M_PanelRecord::iterator pi = custom_panels_.find( panel_name );
    if( pi != custom_panels_.end() )
    {
      delete (*pi).second.dock;
      custom_panels_.erase( pi );
    }
    action->deleteLater();
    if( delete_view_menu_->actions().size() == 1 &&
        delete_view_menu_->actions().first() == action )
    {
      delete_view_menu_->setEnabled( false );
    }
  }
}

PanelDockWidget* VisualizationFrame::addCustomPanel( const std::string& name,
                                                     const std::string& class_lookup_name,
                                                     Qt::DockWidgetArea area,
                                                     bool floating )
{
  try
  {
    Panel* panel = panel_class_loader_->createClassInstance( class_lookup_name );

    PanelRecord record;
    record.dock = addPane( name, panel, area, floating );
    record.lookup_name = class_lookup_name;
    record.panel = panel;
    record.name = name;
    record.delete_action = delete_view_menu_->addAction( QString::fromStdString( name ), this, SLOT( onDeletePanel() ));
    custom_panels_[ name ] = record;
    delete_view_menu_->setEnabled( true );

    record.panel->initialize( manager_ );

    return record.dock;
  }
  catch( pluginlib::LibraryLoadException ex )
  {
    ROS_ERROR( "Failed to load library for Panel plugin class: %s", ex.what() );
    return NULL;
  }
}

PanelDockWidget* VisualizationFrame::addPane( const std::string& name, QWidget* panel, Qt::DockWidgetArea area, bool floating )
{
  std::pair<std::set<std::string>::iterator, bool> insert_result = panel_names_.insert( name );
  if( insert_result.second == false )
  {
    ROS_ERROR( "VisualizationFrame::addPane( %s ): name already in use.", name.c_str() );
    return 0;
  }

  QString q_name = QString::fromStdString( name );
  PanelDockWidget *dock;
  dock = new PanelDockWidget( q_name, this );
  dock->setWidget( panel );
  dock->setFloating( floating );
  dock->setObjectName( q_name );
  addDockWidget( area, dock );
  view_menu_->addAction( dock->toggleViewAction() );

  connect( dock, SIGNAL( destroyed( QObject* )), this, SLOT( onPanelRemoved( QObject* )));

  return dock;
}

void VisualizationFrame::onPanelRemoved( QObject* dock )
{
  std::string name = dock->objectName().toStdString();
  panel_names_.erase( name );
}

}
