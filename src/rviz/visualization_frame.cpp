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

#include <fstream>

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
#include <QTimer>

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
#include "widget_geometry_change_detector.h"
#include "properties/forwards.h" // for StatusCallback

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
#define CONFIG_LAST_IMAGE_DIR "/LastImageDir"

#define CONFIG_EXTENSION "vcg"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
#define PERSPECTIVE_VERSION 2

#define RECENT_CONFIG_COUNT 10

#if BOOST_FILESYSTEM_VERSION == 3
#define BOOST_FILENAME_STRING filename().string
#define BOOST_FILE_STRING string
#else
#define BOOST_FILENAME_STRING filename
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
  , initialized_( false )
  , geom_change_detector_( new WidgetGeometryChangeDetector( this ))
  , loading_( false )
  , post_load_timer_( new QTimer( this ))
{
  panel_class_loader_ = new pluginlib::ClassLoader<Panel>( "rviz", "rviz::Panel" );

  installEventFilter( geom_change_detector_ );
  connect( geom_change_detector_, SIGNAL( changed() ), this, SLOT( setDisplayConfigModified() ));

  post_load_timer_->setSingleShot( true );
  connect( post_load_timer_, SIGNAL( timeout() ), this, SLOT( markLoadingDone() ));
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
  if( prepareToExit() )
  {
    event->accept();
  }
  else
  {
    event->ignore();
  }
}

void VisualizationFrame::changeMaster()
{
  if( prepareToExit() )
  {
    QApplication::exit( 255 );
  }
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

  initConfigs( display_config_file );

  loadGeneralConfig();

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

  connect( manager_, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));
  connect( manager_, SIGNAL( toolAdded( Tool* )), this, SLOT( addTool( Tool* )));
  connect( manager_, SIGNAL( toolChanged( Tool* )), this, SLOT( indicateToolIsCurrent( Tool* )));
  connect( views_panel_, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));

  manager_->initialize( StatusCallback(), verbose );

  loadDisplayConfig( display_config_file_ );

  if( !fixed_frame.empty() )
  {
    manager_->setFixedFrame( fixed_frame );
  }

  if( !target_frame.empty() )
  {
    manager_->setTargetFrame( target_frame );
  }

  setSplashStatus( "Loading perspective" );

  delete splash_;
  splash_ = 0;

  manager_->startUpdate();
  initialized_ = true;
}

void VisualizationFrame::initConfigs( const std::string& display_config_file_override )
{
  home_dir_ = QDir::toNativeSeparators( QDir::homePath() ).toStdString();

  config_dir_ = (fs::path(home_dir_) / ".rviz").BOOST_FILE_STRING();
  general_config_file_ = (fs::path(config_dir_) / "config").BOOST_FILE_STRING();
  default_display_config_file_ = (fs::path(config_dir_) / "display_config").BOOST_FILE_STRING();
  std::string display_config_file = default_display_config_file_;

  if( display_config_file_override != "" )
  {
    if( !fs::exists( display_config_file_override ))
    {
      ROS_ERROR("File [%s] does not exist", display_config_file_override.c_str());
    }
    else
    {
      display_config_file = display_config_file_override;
      ROS_INFO("Loading display config from [%s]", display_config_file_.c_str());
    }
  }
  setDisplayConfigFile( display_config_file );

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
}

void VisualizationFrame::loadGeneralConfig()
{
  ROS_INFO("Loading general config from [%s]", general_config_file_.c_str());
  Config general_config;
  general_config.readFromFile( general_config_file_ );

  std::string recent;
  if( general_config.get( CONFIG_RECENT_CONFIGS, &recent ))
  {
    boost::trim( recent );
    boost::split( recent_configs_, recent, boost::is_any_of (":"), boost::token_compress_on );
  }

  general_config.get( CONFIG_LAST_DIR, &last_config_dir_ );
  general_config.get( CONFIG_LAST_IMAGE_DIR, &last_image_dir_ );
}

void VisualizationFrame::saveGeneralConfig()
{
  ROS_INFO("Saving general config to [%s]", general_config_file_.c_str());
  Config general_config;
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
    general_config.set( CONFIG_RECENT_CONFIGS, ss.str() );
  }
  general_config.set( CONFIG_LAST_DIR, last_config_dir_ );
  general_config.set( CONFIG_LAST_IMAGE_DIR, last_image_dir_ );
  general_config.writeToFile( general_config_file_ );
}

void VisualizationFrame::initMenus()
{
  file_menu_ = menuBar()->addMenu( "&File" );
  file_menu_->addAction( "&Open Config", this, SLOT( onOpen() ), QKeySequence( "Ctrl+O" ));
  file_menu_->addAction( "&Save Config", this, SLOT( save() ), QKeySequence( "Ctrl+S" ));
  file_menu_->addAction( "Save Config &As", this, SLOT( saveAs() ));
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
                                                 &display_name,
                                                 this );
  if( dialog->exec() == QDialog::Accepted )
  {
    addCustomPanel( display_name, lookup_name );
  }
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
      std::string display_name = *it;
      if( display_name == default_display_config_file_ )
      {
        display_name += " (default)";
      }
      if( display_name.find( home_dir_ ) == 0 )
      {
        display_name = ("~" / fs::path( display_name.substr( home_dir_.size() ))).BOOST_FILE_STRING();
      }
      QString qdisplay_name = QString::fromStdString( display_name );
      QAction* action = new QAction( qdisplay_name, this );
      action->setData( QString::fromStdString( *it ));
      connect( action, SIGNAL( triggered() ), this, SLOT( onRecentConfigSelected() ));
      recent_configs_menu_->addAction( action );
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

  // Check if we have unsaved changes to the current config the same
  // as we do during exit, with the same option to cancel.
  if( !prepareToExit() )
  {
    return;
  }

  setWindowModified( false );
  loading_ = true;

  manager_->removeAllDisplays();

  StatusCallback cb;
  LoadingDialog* dialog = NULL;
  if( !initialized_ )
  {
    // If this is running during initial load, don't show a loading
    // dialog.
    cb = boost::bind( &VisualizationFrame::setSplashStatus, this, _1 );
  }
  else
  {
    dialog = new LoadingDialog( this );
    dialog->show();
    cb = boost::bind( &LoadingDialog::setState, dialog, _1 );
  }

  boost::shared_ptr<Config> config( new Config );
  config->readFromFile( path );

  manager_->loadDisplayConfig( config, cb );
  loadCustomPanels( config );
  loadWindowGeometry( config );

  markRecentConfig( path );

  setDisplayConfigFile( path );

  last_config_dir_ = fs::path( path ).parent_path().BOOST_FILE_STRING();

  delete dialog;

  post_load_timer_->start( 1000 );
}

void VisualizationFrame::markLoadingDone()
{
  loading_ = false;
}

void VisualizationFrame::setImageSaveDirectory( const QString& directory )
{
  last_image_dir_ = directory.toStdString();
}

void VisualizationFrame::setDisplayConfigModified()
{
  if( !loading_ )
  {
    setWindowModified( true );
  }
}

void VisualizationFrame::setDisplayConfigFile( const std::string& path )
{
  display_config_file_ = path;

  std::string title;
  if( path == default_display_config_file_ )
  {
    title = "RViz[*]";
  }
  else
  {
    title = fs::path( path ).BOOST_FILENAME_STRING() + "[*] - RViz";
  }
  setWindowTitle( QString::fromStdString( title ));
}

void VisualizationFrame::saveDisplayConfig( const std::string& path )
{
  ROS_INFO( "Saving display config to [%s]", path.c_str() );

  boost::shared_ptr<Config> config( new Config );

  manager_->saveDisplayConfig( config );
  saveCustomPanels( config );
  saveWindowGeometry( config );

  config->writeToFile( path );

  setWindowModified( false );
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

bool VisualizationFrame::prepareToExit()
{
  if( !initialized_ )
  {
    return true;
  }

  saveGeneralConfig();

  if( isWindowModified() )
  {
    if( fileIsWritable( display_config_file_ ))
    {
      QMessageBox box( this );
      box.setText( "There are unsaved changes." );
      box.setInformativeText( QString::fromStdString( "Save changes to " + display_config_file_ + "?" ));
      box.setStandardButtons( QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel );
      box.setDefaultButton( QMessageBox::Save );
      int result = box.exec();
      switch( result )
      {
      case QMessageBox::Save:
        saveDisplayConfig( display_config_file_ );
        return true;
      case QMessageBox::Discard:
        return true;
      default:
        return false;
      }
    }
    else
    {
      QMessageBox box( this );
      box.setText( "There are unsaved changes but file is read-only." );
      box.setInformativeText( QString::fromStdString( "Save new version of " + display_config_file_ + " to another file?" ));
      box.setStandardButtons( QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel );
      box.setDefaultButton( QMessageBox::Save );
      int result = box.exec();
      switch( result )
      {
      case QMessageBox::Save:
        saveAs();
        return true;
      case QMessageBox::Discard:
        return true;
      default:
        return false;
      }
    }
  }
  else
  {
    return true;
  }
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
  }
}

bool VisualizationFrame::fileIsWritable( const std::string& path )
{
  std::fstream test_stream( path.c_str(), std::fstream::app );
  bool writable = test_stream.is_open();
  return writable;
}

void VisualizationFrame::save()
{
  if( !initialized_ )
  {
    return;
  }

  saveGeneralConfig();

  if( fileIsWritable( display_config_file_ ))
  {
    saveDisplayConfig( display_config_file_ );
  }
  else
  {
    QMessageBox box( this );
    box.setText( "Config file is read-only." );
    box.setInformativeText( QString::fromStdString( "Save new version of " + display_config_file_ + " to another file?" ));
    box.setStandardButtons( QMessageBox::Save | QMessageBox::Cancel );
    box.setDefaultButton( QMessageBox::Save );
    if( box.exec() == QMessageBox::Save )
    {
      saveAs();
    }
  }
}

void VisualizationFrame::saveAs()
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

    saveDisplayConfig( filename );

    markRecentConfig( filename );
    last_config_dir_ = fs::path( filename ).parent_path().BOOST_FILE_STRING();
    setDisplayConfigFile( filename );
  }
}

void VisualizationFrame::onSaveImage()
{
  ScreenshotDialog* dialog = new ScreenshotDialog( this, render_panel_, QString::fromStdString( last_image_dir_ ));
  connect( dialog, SIGNAL( savedInDirectory( const QString& )),
           this, SLOT( setImageSaveDirectory( const QString& )));
  dialog->show();
}

void VisualizationFrame::onRecentConfigSelected()
{
  QAction* action = dynamic_cast<QAction*>( sender() );
  if( action )
  {
    std::string path = action->data().toString().toStdString();
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
      setDisplayConfigModified();
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
    connect( panel, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));

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
  dock = new PanelDockWidget( q_name, panel );
  dock->setWidget( panel );
  dock->setFloating( floating );
  dock->setObjectName( q_name );
  addDockWidget( area, dock );
  QAction* toggle_action = dock->toggleViewAction();
  view_menu_->addAction( toggle_action );

  connect( dock, SIGNAL( destroyed( QObject* )), this, SLOT( onPanelRemoved( QObject* )));

  // There is a small tricky bug here.  If this is changed from
  // triggered(bool) to toggled(bool), minimizing the rviz window
  // causes a call to setDisplayConfigModified(), which is wrong.
  // With this AS IS, it does not call setDisplayConfigModified() when
  // a floating PanelDockWidget is closed by clicking the "x" close
  // button in the top right corner.  Probably the solution is to
  // leave this as is and implement a custom top bar widget for
  // PanelDockWidget which catches the "x" button click separate from
  // the visibility change event.  Sigh.
  connect( toggle_action, SIGNAL( triggered( bool )), this, SLOT( setDisplayConfigModified() ));

  dock->installEventFilter( geom_change_detector_ );

  return dock;
}

void VisualizationFrame::onPanelRemoved( QObject* dock )
{
  std::string name = dock->objectName().toStdString();
  panel_names_.erase( name );
}

}
