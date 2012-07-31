/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDesktopServices>
#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QUrl>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <yaml-cpp/emitter.h>
#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>

#include <ros/console.h>
#include <ros/package.h>

#include <OGRE/OgreRenderWindow.h>

#include <ogre_helpers/initialization.h>

#include "rviz/displays_panel.h"
#include "rviz/failed_panel.h"
#include "rviz/help_panel.h"
#include "rviz/loading_dialog.h"
#include "rviz/new_object_dialog.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/properties/yaml_helpers.h"
#include "rviz/render_panel.h"
#include "rviz/screenshot_dialog.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/selection_panel.h"
#include "rviz/splash_screen.h"
#include "rviz/time_panel.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/tool_properties_panel.h"
#include "rviz/views_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/widget_geometry_change_detector.h"
#include "rviz/load_resource.h"

#include "rviz/visualization_frame.h"

namespace fs = boost::filesystem;

#define CONFIG_EXTENSION "rviz"
#define CONFIG_EXTENSION_WILDCARD "*."CONFIG_EXTENSION
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
  , splash_( NULL )
  , position_correction_( 0, 0 )
  , num_move_events_( 0 )
  , toolbar_actions_( NULL )
  , show_choose_new_master_option_( false )
  , add_tool_action_( NULL )
  , remove_tool_menu_( NULL )
  , initialized_( false )
  , geom_change_detector_( new WidgetGeometryChangeDetector( this ))
  , loading_( false )
  , post_load_timer_( new QTimer( this ))
{
  panel_factory_ = new PluginlibFactory<Panel>( "rviz", "rviz::Panel" );

  installEventFilter( geom_change_detector_ );
  connect( geom_change_detector_, SIGNAL( changed() ), this, SLOT( setDisplayConfigModified() ));

  post_load_timer_->setSingleShot( true );
  connect( post_load_timer_, SIGNAL( timeout() ), this, SLOT( markLoadingDone() ));

  package_path_ = ros::package::getPath("rviz");
  help_path_ = (fs::path(package_path_) / "help/help.html").BOOST_FILE_STRING();
  splash_path_ = QString::fromStdString( (fs::path(package_path_) / "images/splash.png").BOOST_FILE_STRING() );
}

VisualizationFrame::~VisualizationFrame()
{
  delete render_panel_;
  delete manager_;

  for( int i = 0; i < custom_panels_.size(); i++ )
  {
    delete custom_panels_[ i ].dock;
  }

  delete panel_factory_;
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

void VisualizationFrame::setShowChooseNewMaster( bool show )
{
  show_choose_new_master_option_ = show;
}

void VisualizationFrame::setHelpPath( const QString& help_path )
{
  help_path_ = help_path.toStdString();
}

void VisualizationFrame::setSplashPath( const QString& splash_path )
{
  splash_path_ = splash_path;
}

void VisualizationFrame::initialize(const QString& display_config_file )
{
  initConfigs();

  loadPersistentSettings();

  QIcon app_icon( QString::fromStdString( (fs::path(package_path_) / "icons/package.png").BOOST_FILE_STRING() ) );
  setWindowIcon( app_icon );

  if( splash_path_ != "" )
  {
    QPixmap splash_image( splash_path_ );
    splash_ = new SplashScreen( splash_image );
    splash_->show();
    connect( this, SIGNAL( statusUpdate( const QString& )), splash_, SLOT( showMessage( const QString& )));
  }
  Q_EMIT statusUpdate( "Initializing" );

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
  toolbar_->setToolButtonStyle( Qt::ToolButtonTextBesideIcon );
  toolbar_actions_ = new QActionGroup( this );
  connect( toolbar_actions_, SIGNAL( triggered( QAction* )), this, SLOT( onToolbarActionTriggered( QAction* )));
  view_menu_->addAction( toolbar_->toggleViewAction() );

  add_tool_action_ = new QAction( "", toolbar_actions_ );
  add_tool_action_->setToolTip( "Add a new tool" );
  add_tool_action_->setIcon( loadPixmap( "package://rviz/icons/plus.png" ) );
  toolbar_->addAction( add_tool_action_ );
  connect( add_tool_action_, SIGNAL( triggered() ), this, SLOT( openNewToolDialog() ));

  remove_tool_menu_ = new QMenu();
  QToolButton* remove_tool_button = new QToolButton();
  remove_tool_button->setMenu( remove_tool_menu_ );
  remove_tool_button->setPopupMode( QToolButton::InstantPopup );
  remove_tool_button->setToolTip( "Remove a tool from the toolbar" );
  remove_tool_button->setIcon( loadPixmap( "package://rviz/icons/minus.png" ) );
  toolbar_->addWidget( remove_tool_button );
  connect( remove_tool_menu_, SIGNAL( triggered( QAction* )), this, SLOT( onToolbarRemoveTool( QAction* )));

  setCentralWidget( render_panel_ );

  addPane( "Displays", displays_panel_, Qt::LeftDockWidgetArea, false );
  addPane( "Tool Properties", tool_properties_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Views", views_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Selection", selection_panel_, Qt::RightDockWidgetArea, false );
  addPane( "Time", time_panel_, Qt::BottomDockWidgetArea, false );

  manager_ = new VisualizationManager( render_panel_, this );
  connect( manager_, SIGNAL( statusUpdate( const QString& )), this, SIGNAL( statusUpdate( const QString& )));

  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  displays_panel_->initialize( manager_ );
  views_panel_->initialize( manager_ );
  time_panel_->initialize(manager_);
  selection_panel_->initialize( manager_ );
  tool_properties_panel_->initialize( manager_ );

  ToolManager* tool_man = manager_->getToolManager();

  connect( manager_, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));
  connect( tool_man, SIGNAL( toolAdded( Tool* )), this, SLOT( addTool( Tool* )));
  connect( tool_man, SIGNAL( toolRemoved( Tool* )), this, SLOT( removeTool( Tool* )));
  connect( tool_man, SIGNAL( toolChanged( Tool* )), this, SLOT( indicateToolIsCurrent( Tool* )));
  connect( views_panel_, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));

  manager_->initialize();

  if( display_config_file != "" )
  {
    loadDisplayConfig( display_config_file );
  }
  else
  {
    loadDisplayConfig( QString::fromStdString( default_display_config_file_ ));
  }

  delete splash_;
  splash_ = 0;

  manager_->startUpdate();
  initialized_ = true;
}

void VisualizationFrame::initConfigs()
{
  home_dir_ = QDir::toNativeSeparators( QDir::homePath() ).toStdString();

  config_dir_ = (fs::path(home_dir_) / ".rviz").BOOST_FILE_STRING();
  persistent_settings_file_ = (fs::path(config_dir_) / "persistent_settings").BOOST_FILE_STRING();
  default_display_config_file_ = (fs::path(config_dir_) / "default."CONFIG_EXTENSION).BOOST_FILE_STRING();

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

void VisualizationFrame::loadPersistentSettings()
{
  std::ifstream in( persistent_settings_file_.c_str() );
  if( in )
  {
    ROS_INFO("Loading persistent settings from [%s]", persistent_settings_file_.c_str());
    YAML::Parser parser( in );
    YAML::Node yaml_node;
    parser.GetNextDocument( yaml_node );
    
    if( const YAML::Node *last_dir_node = yaml_node.FindValue( "Last Config Dir" ))
    {
      *last_dir_node >> last_config_dir_;
    }
    if( const YAML::Node *last_image_node = yaml_node.FindValue( "Last Image Dir" ))
    {
      *last_image_node >> last_image_dir_;
    }
    
    if( const YAML::Node *recent_configs_node = yaml_node.FindValue( "Recent Configs" ))
    {
      if( recent_configs_node->Type() != YAML::NodeType::Sequence )
      {
        printf( "VisualizationFrame::loadPersistentSettings() TODO: error handling - unexpected non-Sequence YAML type.\n" );
        return;
      }
      recent_configs_.clear();
      for( YAML::Iterator it = recent_configs_node->begin(); it != recent_configs_node->end(); ++it )
      {
        std::string file;
        *it >> file;
        recent_configs_.push_back( file );
      }
    }
  }
  else
  {
    ROS_ERROR( "Failed to open file [%s] for reading", persistent_settings_file_.c_str() );
  }
}

void VisualizationFrame::savePersistentSettings()
{
  std::ofstream out( persistent_settings_file_.c_str() );
  if( out )
  {
    ROS_INFO("Saving persistent settings to [%s]", persistent_settings_file_.c_str());

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "Last Config Dir" << YAML::Value << last_config_dir_;
    emitter << YAML::Key << "Last Image Dir" << YAML::Value << last_image_dir_;
    emitter << YAML::Key << "Recent Configs";
    emitter << YAML::Value;
    {
      emitter << YAML::BeginSeq;
      for( D_string::iterator it = recent_configs_.begin(); it != recent_configs_.end(); ++it )
      {
        emitter << *it;
      }
      emitter << YAML::EndSeq;
    }
    emitter << YAML::EndMap;
    out << emitter.c_str() << std::endl;
  }
  else
  {
    ROS_ERROR( "Failed to open file [%s] for writing", persistent_settings_file_.c_str() );
  }
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

  QMenu* help_menu = menuBar()->addMenu( "&Help" );
  help_menu->addAction( "Show &Help panel", this, SLOT( showHelpPanel() ));
  help_menu->addAction( "Open rviz wiki in browser", this, SLOT( onHelpWiki() ));
}

void VisualizationFrame::openNewPanelDialog()
{
  QString class_id;
  QString display_name;
  QStringList empty;

  NewObjectDialog* dialog = new NewObjectDialog( panel_factory_,
                                                 "Panel",
                                                 empty,
                                                 empty,
                                                 &class_id,
                                                 &display_name,
                                                 this );
  if( dialog->exec() == QDialog::Accepted )
  {
    addCustomPanel( display_name, class_id );
  }
}

void VisualizationFrame::openNewToolDialog()
{
  QString class_id;
  QStringList empty;
  ToolManager* tool_man = manager_->getToolManager();

  NewObjectDialog* dialog = new NewObjectDialog( tool_man->getFactory(),
                                                 "Tool",
                                                 empty,
                                                 tool_man->getToolClasses(),
                                                 &class_id );
  if( dialog->exec() == QDialog::Accepted )
  {
    tool_man->addTool( class_id );
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

void VisualizationFrame::loadDisplayConfig( const QString& qpath )
{
  std::string path = qpath.toStdString();
  std::string actual_load_path = path;
  if( !fs::exists( path ))
  {
    actual_load_path = (fs::path(package_path_) / "default.rviz").BOOST_FILE_STRING();      
    if( !fs::exists( actual_load_path ))
    {
      ROS_ERROR( "Default display config '%s' not found.  RViz will be very empty at first.", actual_load_path.c_str() );
      return;
    }
  }

  // Check if we have unsaved changes to the current config the same
  // as we do during exit, with the same option to cancel.
  if( !prepareToExit() )
  {
    return;
  }

  setWindowModified( false );
  loading_ = true;

  LoadingDialog* dialog = NULL;
  if( initialized_ )
  {
    dialog = new LoadingDialog( this );
    dialog->show();
    connect( this, SIGNAL( statusUpdate( const QString& )), dialog, SLOT( showMessage( const QString& )));
  }

  std::ifstream in( actual_load_path.c_str() );
  if( in )
  {
    YAML::Parser parser( in );
    YAML::Node node;
    parser.GetNextDocument( node );
    load( node );
  }

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

void VisualizationFrame::saveDisplayConfig( const QString& qpath )
{
  std::string path = qpath.toStdString();
  std::ofstream out( path.c_str() );
  if( out )
  {
    ROS_INFO( "Saving display config to [%s]", path.c_str() );

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    save( emitter );
    emitter << YAML::EndMap;
    out << emitter.c_str() << std::endl;

    setWindowModified( false );
  }
  else
  {
    ROS_ERROR( "Failed to open file [%s] for writing", path.c_str() );
  }
}

void VisualizationFrame::save( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Visualization Manager";
  emitter << YAML::Value;
  manager_->save( emitter );

  emitter << YAML::Key << "Panels";
  emitter << YAML::Value;
  {
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "Displays";
    emitter << YAML::Value;
    displays_panel_->save( emitter );

    emitter << YAML::EndMap;
  }

  saveCustomPanels( emitter );
  saveWindowGeometry( emitter );
}

void VisualizationFrame::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "VisualizationFrame::load() TODO: error handling - unexpected YAML type.\n" );
    return;
  }

  if( const YAML::Node *name_node = yaml_node.FindValue( "Visualization Manager" ))
  {
    manager_->load( *name_node );
  }

  if( const YAML::Node *panels_node = yaml_node.FindValue( "Panels" ))
  {
    if( const YAML::Node *displays_node = panels_node->FindValue( "Displays" ))
    {
      displays_panel_->load( *displays_node );
    }
  }

  loadCustomPanels( yaml_node );
  loadWindowGeometry( yaml_node );
}

void VisualizationFrame::loadWindowGeometry( const YAML::Node& yaml_node )
{
  if( const YAML::Node *name_node = yaml_node.FindValue( "Window Geometry" ))
  {
    int x, y, width, height;
    (*name_node)[ "X" ] >> x;
    (*name_node)[ "Y" ] >> y;
    (*name_node)[ "Width" ] >> width;
    (*name_node)[ "Height" ] >> height;
    move( x, y );
    resize( width, height );
    
    std::string main_window_config;
    (*name_node)[ "QMainWindow State" ] >> main_window_config;
    restoreState( QByteArray::fromHex( main_window_config.c_str() ));
  }
}

void VisualizationFrame::saveWindowGeometry( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Window Geometry";
  emitter << YAML::Value;
  {
    emitter << YAML::BeginMap;
    QRect geom = hackedFrameGeometry();
    emitter << YAML::Key << "X" << YAML::Value << geom.x();
    emitter << YAML::Key << "Y" << YAML::Value << geom.y();
    emitter << YAML::Key << "Width" << YAML::Value << geom.width();
    emitter << YAML::Key << "Height" << YAML::Value << geom.height();

    QByteArray window_state = saveState().toHex();
    emitter << YAML::Key << "QMainWindow State" << YAML::Value << window_state.constData();
    emitter << YAML::EndMap;
  }
}

void VisualizationFrame::loadCustomPanels( const YAML::Node& yaml_node )
{
  // First destroy any existing custom panels.
  for( int i = 0; i < custom_panels_.size(); i++ )
  {
    delete custom_panels_[ i ].dock;
    delete custom_panels_[ i ].delete_action;
  }
  custom_panels_.clear();

  // Then load the ones in the config.
  if( const YAML::Node *panels_node = yaml_node.FindValue( "Custom Panels" ))
  {
    if( panels_node->Type() != YAML::NodeType::Sequence )
    {
      printf( "VisualizationFrame::loadCustomPanels() TODO: error handling - unexpected non-Sequence YAML type.\n" );
      return;
    }
    for( YAML::Iterator it = panels_node->begin(); it != panels_node->end(); ++it )
    {
      const YAML::Node& panel_node = *it;
      QString class_id, name;
      panel_node[ "Class" ] >> class_id;
      panel_node[ "Name" ] >> name;

      QDockWidget* dock = addCustomPanel( name, class_id );
      // This is kind of ridiculous - should just be something like
      // createPanel() and addPanel() so I can do load() without this
      // qobject_cast.
      if( dock )
      {
        Panel* panel = qobject_cast<Panel*>( dock->widget() );
        if( panel )
        {
          panel->load( panel_node );
        }
      }
    }
  }
}

void VisualizationFrame::saveCustomPanels( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Custom Panels";
  emitter << YAML::Value;
  {
    emitter << YAML::BeginSeq;
    for( int i = 0; i < custom_panels_.size(); i++ )
    {
      custom_panels_[ i ].panel->save( emitter );
    }
    emitter << YAML::EndSeq;
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

bool VisualizationFrame::prepareToExit()
{
  if( !initialized_ )
  {
    return true;
  }

  savePersistentSettings();

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
        saveDisplayConfig( QString::fromStdString( display_config_file_ ));
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
    std::string path = filename.toStdString();

    if( !fs::exists( path ))
    {
      QString message = filename + " does not exist!";
      QMessageBox::critical( this, "Config file does not exist", message );
      return;
    }

    loadDisplayConfig( filename );
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

  savePersistentSettings();

  if( fileIsWritable( display_config_file_ ))
  {
    saveDisplayConfig( QString::fromStdString( display_config_file_ ));
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

    saveDisplayConfig( QString::fromStdString( filename ));

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
      if( !fs::exists( path ))
      {
        QString message = QString::fromStdString( path  ) + " does not exist!";
        QMessageBox::critical( this, "Config file does not exist", message );
        return;
      }

      loadDisplayConfig( QString::fromStdString( path ));
    }
  }
}

void VisualizationFrame::addTool( Tool* tool )
{
  QAction* action = new QAction( tool->getName(), toolbar_actions_ );
  action->setIcon( tool->getIcon() );
  action->setIconText( tool->getName() );
  action->setCheckable( true );
  action->setShortcut( QKeySequence( QString( tool->getShortcutKey() )));
  toolbar_->insertAction( add_tool_action_, action );
  action_to_tool_map_[ action ] = tool;
  tool_to_action_map_[ tool ] = action;

  remove_tool_menu_->addAction( tool->getName() );
}

void VisualizationFrame::onToolbarActionTriggered( QAction* action )
{
  Tool* tool = action_to_tool_map_[ action ];
  if( tool )
  {
    manager_->getToolManager()->setCurrentTool( tool );
  }
}

void VisualizationFrame::onToolbarRemoveTool( QAction* remove_tool_menu_action )
{
  QString name = remove_tool_menu_action->text();
  for( int i = 0; i < manager_->getToolManager()->numTools(); i++ )
  {
    Tool* tool = manager_->getToolManager()->getTool( i );
    if( tool->getName() == name )
    {
      manager_->getToolManager()->removeTool( i );
      return;
    }
  }
}

void VisualizationFrame::removeTool( Tool* tool )
{
  QAction* action = tool_to_action_map_[ tool ];
  if( action )
  {
    toolbar_actions_->removeAction( action );
    toolbar_->removeAction( action );
    tool_to_action_map_.erase( tool );
    action_to_tool_map_.erase( action );
  }
  QString tool_name = tool->getName();
  QList<QAction*> remove_tool_actions = remove_tool_menu_->actions();
  for( int i = 0; i < remove_tool_actions.size(); i++ )
  {
    QAction* removal_action = remove_tool_actions.at( i );
    if( removal_action->text() == tool_name )
    {
      remove_tool_menu_->removeAction( removal_action );
      break;
    }
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
  // This should only be called as a SLOT from a QAction in the
  // "delete panel" submenu, so the sender will be one of the QActions
  // stored as "delete_action" in a PanelRecord.  This code looks for
  // a delete_action in custom_panels_ matching sender() and removes
  // the panel associated with it.
  if( QAction* action = qobject_cast<QAction*>( sender() ))
  {
    for( int i = 0; i < custom_panels_.size(); i++ )
    {
      if( custom_panels_[ i ].delete_action == action )
      {
        delete custom_panels_[ i ].dock;
        custom_panels_.removeAt( i );
        setDisplayConfigModified();
        action->deleteLater();
        if( delete_view_menu_->actions().size() == 1 &&
            delete_view_menu_->actions().first() == action )
        {
          delete_view_menu_->setEnabled( false );
        }
        return;
      }
    }
  }
}

QDockWidget* VisualizationFrame::addCustomPanel( const QString& name,
                                                 const QString& class_id,
                                                 Qt::DockWidgetArea area,
                                                 bool floating )
{
  QString error;
  Panel* panel = panel_factory_->make( class_id, &error );
  if( !panel )
  {
    panel = new FailedPanel( class_id, error );
  }
  panel->setName( name );
  connect( panel, SIGNAL( configChanged() ), this, SLOT( setDisplayConfigModified() ));

  PanelRecord record;
  record.dock = addPane( name, panel, area, floating );
  record.panel = panel;
  record.name = name;
  record.delete_action = delete_view_menu_->addAction( name, this, SLOT( onDeletePanel() ));
  custom_panels_.append( record );
  delete_view_menu_->setEnabled( true );

  record.panel->initialize( manager_ );

  return record.dock;
}

QDockWidget* VisualizationFrame::addPane( const QString& name, QWidget* panel, Qt::DockWidgetArea area, bool floating )
{
  PanelDockWidget *dock;
  dock = new PanelDockWidget( name );
  dock->setWidget( panel );
  dock->setFloating( floating );
  dock->setObjectName( name ); // QMainWindow::saveState() needs objectName to be set.
  addDockWidget( area, dock );
  QAction* toggle_action = dock->toggleViewAction();
  view_menu_->addAction( toggle_action );

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

} // end namespace rviz
