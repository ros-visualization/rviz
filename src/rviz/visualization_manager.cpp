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

#include <algorithm>

#include <QApplication>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreLight.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderWindow.h>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>

#include <tf/transform_listener.h>

#include <ros/package.h>

#include "rviz/display_group.h"
#include "rviz/display.h"
#include "rviz/displays_panel.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/pluginlib_display_factory.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/tool.h"
#include "rviz/view_controller.h"
#include "rviz/view_controllers/fixed_orientation_ortho_view_controller.h"
#include "rviz/view_controllers/fps_view_controller.h"
#include "rviz/view_controllers/orbit_view_controller.h"
#include "rviz/view_controllers/xy_orbit_view_controller.h"
#include "rviz/viewport_mouse_event.h"

#include "rviz/visualization_manager.h"

#define CAMERA_TYPE "Camera Type"
#define CAMERA_CONFIG "Camera Config"

namespace rviz
{

VisualizationManager::VisualizationManager( RenderPanel* render_panel, WindowManagerInterface* wm )
: ogre_root_( Ogre::Root::getSingletonPtr() )
, update_timer_(0)
, shutting_down_(false)
, current_tool_( NULL )
, render_panel_( render_panel )
, time_update_timer_(0.0f)
, frame_update_timer_(0.0f)
, view_controller_(0)
, render_requested_(1)
, frame_count_(0)
, window_manager_(wm)
, disable_update_(false)
{
  frame_manager_ = FrameManager::instance();

  render_panel->setAutoRender(false);

  threaded_nh_.setCallbackQueue(&threaded_queue_);

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  target_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( -1, 0, -1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  root_display_group_ = new DisplayGroup();
  root_display_group_->setEnabled( true );
  display_property_tree_model_ = new PropertyTreeModel( root_display_group_ );
  
  tool_property_tree_model_ = new PropertyTreeModel( new Property() );

  //connect( property_manager_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
  //connect( tool_property_manager_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));

  global_options_ = new Property( "Global Options", QVariant(), "", root_display_group_ );

  fixed_frame_property_ = new TfFrameProperty( "Fixed Frame", "/map",
                                               "Frame into which all data is transformed before being displayed.",
                                               global_options_, frame_manager_->getTFClient(), false,
                                               SLOT( updateFixedFrame() ), this );

  target_frame_property_ = new TfFrameProperty( "Target Frame", TfFrameProperty::FIXED_FRAME_STRING,
                                                "Reference frame for the 3D camera view.",
                                                global_options_, frame_manager_->getTFClient(), true,
                                                SLOT( updateTargetFrame() ), this );

  background_color_property_ = new ColorProperty( "Background Color", Qt::black,
                                                  "Background color for the 3D view.",
                                                  global_options_, SLOT( updateBackgroundColor() ), this );
  updateBackgroundColor();

  global_status_ = new StatusList( "Global Status", root_display_group_ );

  //CategoryPropertyPtr cat_prop = options_category.lock();
  //cat_prop->collapse();

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  threaded_queue_threads_.create_thread(boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));

  display_class_loader_ = new pluginlib::ClassLoader<Display>( "rviz", "rviz::Display" );
  display_factory_ = new PluginlibDisplayFactory( display_class_loader_ );

  tool_class_loader_ = new pluginlib::ClassLoader<Tool>( "rviz", "rviz::Tool" );
}

VisualizationManager::~VisualizationManager()
{
  delete update_timer_;
  delete idle_timer_;

  shutting_down_ = true;
  threaded_queue_threads_.join_all();

  if(selection_manager_)
  {
    selection_manager_->setSelection(M_Picked());
  }

  delete display_property_tree_model_;

  V_ToolRecord::iterator tool_it = tools_.begin();
  V_ToolRecord::iterator tool_end = tools_.end();
  for ( ; tool_it != tool_end; ++tool_it )
  {
    delete (*tool_it).tool;
  }
  tools_.clear();

  delete display_factory_;
  delete display_class_loader_;
  delete tool_property_tree_model_;
  delete selection_manager_;

  scene_manager_->destroySceneNode( target_scene_node_ );

  if(ogre_root_)
  {
    ogre_root_->destroySceneManager( scene_manager_ );
  }
}

void VisualizationManager::initialize(const StatusCallback& cb, bool verbose)
{
  if(cb)
  {
    cb("Initializing TF");
  }

  render_panel_->getCamera()->setPosition(0, 10, 15);
  render_panel_->getCamera()->setNearClipDistance(0.01f);
  render_panel_->getCamera()->lookAt(0, 0, 0);

  addViewController(XYOrbitViewController::getClassNameStatic(), "XYOrbit");
  addViewController(OrbitViewController::getClassNameStatic(), "Orbit");
  addViewController(FPSViewController::getClassNameStatic(), "FPS");
  addViewController(FixedOrientationOrthoViewController::getClassNameStatic(), "TopDownOrtho");
  setCurrentViewControllerType(OrbitViewController::getClassNameStatic());

  selection_manager_->initialize( verbose );

  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();
}

void VisualizationManager::startUpdate()
{
  update_timer_ = new QTimer;
  connect( update_timer_, SIGNAL( timeout() ), this, SLOT( onUpdate() ));
  update_timer_->start( 33 );

  idle_timer_ = new QTimer;
  connect( idle_timer_, SIGNAL( timeout() ), this, SLOT( onIdle() ));
  idle_timer_->start( 33 );
}

void createColorMaterial(const std::string& name, const Ogre::ColourValue& color)
{
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create( name, ROS_PACKAGE_NAME );
  mat->setAmbient(color * 0.5f);
  mat->setDiffuse(color);
  mat->setSelfIllumination(color);
  mat->setLightingEnabled(true);
  mat->setReceiveShadows(false);
}

void VisualizationManager::createColorMaterials()
{
  createColorMaterial("RVIZ/Red", Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f));
  createColorMaterial("RVIZ/Green", Ogre::ColourValue(0.0f, 1.0f, 0.0f, 1.0f));
  createColorMaterial("RVIZ/Blue", Ogre::ColourValue(0.0f, 0.0f, 1.0f, 1.0f));
  createColorMaterial("RVIZ/Cyan", Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f));
}

void VisualizationManager::queueRender()
{
  render_requested_ = 1;
}

void VisualizationManager::onUpdate()
{
  if(disable_update_)
  {
    return;
  }

  disable_update_ = true;

  //process pending mouse events

  std::deque<ViewportMouseEvent> event_queue;
  {
    boost::mutex::scoped_lock lock(vme_queue_mutex_);
    event_queue.swap( vme_queue_ );
  }

  std::deque<ViewportMouseEvent>::iterator event_it;

  for ( event_it= event_queue.begin(); event_it!=event_queue.end(); event_it++ )
  {
    ViewportMouseEvent &vme = *event_it;
    int flags = 0;
    if( current_tool_ )
    {
      flags = current_tool_->processMouseEvent(vme);
    }

    if( flags & Tool::Render )
    {
      queueRender();
    }

    if( flags & Tool::Finished )
    {
      setCurrentTool( getDefaultTool() );
    }
  }


  ros::WallTime update_start = ros::WallTime::now();

  ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
  ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
  float wall_dt = wall_diff.toSec();
  float ros_dt = ros_diff.toSec();

  if(ros_dt < 0.0)
  {
    resetTime();
  }

  frame_manager_->update();

  ros::spinOnce();

  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();

  root_display_group_->update( wall_dt, ros_dt );

  view_controller_->update(wall_dt, ros_dt);

  time_update_timer_ += wall_dt;

  if( time_update_timer_ > 0.1f )
  {
    time_update_timer_ = 0.0f;

    updateTime();
  }

  frame_update_timer_ += wall_dt;

  if(frame_update_timer_ > 1.0f)
  {
    frame_update_timer_ = 0.0f;

    updateFrames();
  }

  selection_manager_->update();

  if( current_tool_ )
  {
    current_tool_->update(wall_dt, ros_dt);
  }

  disable_update_ = false;
}

void VisualizationManager::onIdle()
{
  ros::WallTime cur = ros::WallTime::now();
  double dt = (cur - last_render_).toSec();

  if(dt > 0.1f)
  {
    render_requested_ = 1;
  }

  // Cap at 60fps
  if(render_requested_ && dt > 0.016f)
  {
    render_requested_ = 0;
    last_render_ = cur;
    frame_count_++;

    boost::mutex::scoped_lock lock(render_mutex_);

//    ros::WallTime start = ros::WallTime::now();
    ogre_root_->renderOneFrame();
//    ros::WallTime end = ros::WallTime::now();
//    ros::WallDuration d = end - start;
//    ROS_INFO("Render took [%f] msec", d.toSec() * 1000.0f);
  }
}

void VisualizationManager::updateTime()
{
  if( ros_time_begin_.isZero() )
  {
    ros_time_begin_ = ros::Time::now();
  }

  ros_time_elapsed_ = ros::Time::now() - ros_time_begin_;

  if( wall_clock_begin_.isZero() )
  {
    wall_clock_begin_ = ros::WallTime::now();
  }

  wall_clock_elapsed_ = ros::WallTime::now() - wall_clock_begin_;

  Q_EMIT timeChanged();
}

void VisualizationManager::updateFrames()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  frame_manager_->getTFClient()->getFrameStrings( frames );

  // Check the fixed frame to see if it's ok
  std::string error;
  if(frame_manager_->frameHasProblems(fixed_frame_, ros::Time(), error))
  {
    if(frames.empty())
    {
      // fixed_prop->setToWarn();
      std::stringstream ss;
      ss << "No tf data.  Actual error: " << error;
      global_status_->setStatus( StatusProperty::Warn, "Fixed Frame", QString::fromStdString( ss.str() ));
    }
    else
    {
      // fixed_prop->setToError();
      global_status_->setStatus( StatusProperty::Error, "Fixed Frame", QString::fromStdString( error ));
    }
  }
  else
  {
    // fixed_prop->setToOK();
    global_status_->setStatus( StatusProperty::Ok, "Fixed Frame", "OK" );
  }

  if(frame_manager_->transformHasProblems(target_frame_, ros::Time(), error))
  {
    // target_prop->setToError();
    global_status_->setStatus( StatusProperty::Error, "Target Frame", QString::fromStdString( error ));
  }
  else
  {
    // target_prop->setToOK();
    global_status_->setStatus( StatusProperty::Ok, "Target Frame", "OK" );
  }
}

tf::TransformListener* VisualizationManager::getTFClient() const
{
  return frame_manager_->getTFClient();
}

void VisualizationManager::resetTime()
{
  root_display_group_->reset();
  frame_manager_->getTFClient()->clear();

  ros_time_begin_ = ros::Time();
  wall_clock_begin_ = ros::WallTime();

  queueRender();
}

void VisualizationManager::addDisplay( Display* display, bool enabled )
{
  display->setParentProperty( root_display_group_ );
  display->setFixedFrame( getFixedFrame() );
  display->setEnabled( enabled );
}

void VisualizationManager::removeAllDisplays()
{
  root_display_group_->clear();
}

void VisualizationManager::setCurrentTool( Tool* tool )
{
  if( current_tool_ )
  {
    current_tool_->deactivate();
  }

  current_tool_ = tool;
  if( current_tool_ )
  {
    current_tool_->activate();
  }

  Q_EMIT toolChanged( tool );
}

void VisualizationManager::setDefaultTool( Tool* tool )
{
  default_tool_ = tool;
}

Tool* VisualizationManager::getTool( int index )
{
  ROS_ASSERT( index >= 0 );
  ROS_ASSERT( index < (int)tools_.size() );

  return tools_[ index ].tool;
}

void VisualizationManager::removeTool( int index )
{
  ToolRecord record = tools_[ index ];
  tools_.erase( tools_.begin() + index );
  if( current_tool_ == record.tool )
  {
    current_tool_ = NULL;
  }
  if( default_tool_ == record.tool )
  {
    default_tool_ = NULL;
  }
  Q_EMIT toolRemoved( record.tool );
  delete record.tool;
}

std::set<std::string> VisualizationManager::getToolClasses()
{
  std::set<std::string> class_names;
  V_ToolRecord::iterator tool_it;
  for ( tool_it = tools_.begin(); tool_it != tools_.end(); ++tool_it )
  {
    class_names.insert( (*tool_it).lookup_name );
  }
  return class_names;
}

void VisualizationManager::load( const YAML::Node& yaml_node, const StatusCallback& cb )
{
  disable_update_ = true;

  if(cb)
  {
    cb("Creating displays");
  }

  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "VisualizationManager::load() TODO: error handling - unexpected YAML type.\n" );
    return;
  }
  
  if( const YAML::Node *global_options_node = yaml_node.FindValue( "Global Options" ))
  {
    global_options_->load( *global_options_node );
  }
  
  root_display_group_->loadDisplays( yaml_node );

  if(cb)
  {
    cb("Creating tools");
  }
  ///// std::string tool_class_names;
  ///// config->get( "Tools", &tool_class_names,
  /////              "rviz/MoveCamera,rviz/Interact,rviz/Select,rviz/SetGoal,rviz/SetInitialPose" );
  ///// std::istringstream iss( tool_class_names );
  ///// std::string tool_class_lookup_name;
  ///// while( std::getline( iss, tool_class_lookup_name, ',' ))
  ///// {
  /////   addTool( tool_class_lookup_name );
  ///// }
  ///// port the above code, then remove the hard-coded line below:
  addTool( "rviz/MoveCamera" );

  ///// tool_property_manager_->load( config, cb );

  ///// // Load view controller
  ///// std::string camera_type;
  ///// if(config->get(CAMERA_TYPE, &camera_type))
  ///// {
  /////   if(setCurrentViewControllerType(camera_type))
  /////   {
  /////     std::string camera_config;
  /////     if(config->get(CAMERA_CONFIG, &camera_config))
  /////     {
  /////       view_controller_->fromString(camera_config);
  /////     }
  /////   }
  ///// }

  disable_update_ = false;
}

void VisualizationManager::addTool( const std::string& tool_class_lookup_name )
{
  try
  {
    ToolRecord record;
    record.tool = tool_class_loader_->createUnmanagedInstance( tool_class_lookup_name );
    record.lookup_name = tool_class_lookup_name;
    tools_.push_back( record );
    record.tool->initialize( this );

    Q_EMIT toolAdded( record.tool );

    // If the tool we just added was the first ever, set it as the
    // default and current.
    if( tools_.size() == 1 )
    {
      setDefaultTool( record.tool );
      setCurrentTool( record.tool );
    }
  }
  catch( pluginlib::PluginlibException& ex )
  {
    ROS_ERROR( "The plugin for class '%s' failed to load.  Error: %s",
               tool_class_lookup_name.c_str(), ex.what() );
  }
}

void VisualizationManager::save( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Global Options";
  emitter << YAML::Value;
  global_options_->save( emitter );

  root_display_group_->saveDisplays( emitter );

  ///// std::stringstream tool_class_names;
  ///// for ( int i = 0; i < tools_.size(); i++ )
  ///// {
  /////   if( i != 0 )
  /////   {
  /////     tool_class_names << ',';
  /////   }
  /////   tool_class_names << tools_[ i ].lookup_name;
  ///// }
  ///// config->set( "Tools", tool_class_names.str() );
  ///// 
  ///// tool_property_manager_->save( config );
  ///// 
  ///// if(view_controller_)
  ///// {
  /////   config->set(CAMERA_TYPE, view_controller_->getClassName());
  /////   config->set(CAMERA_CONFIG, view_controller_->toString());
  ///// }
}

Display* VisualizationManager::createDisplay( const QString& class_lookup_name,
                                              const QString& name,
                                              bool enabled )
{
  Display* new_display = getDisplayFactory()->createDisplay( class_lookup_name );
  new_display->setName( name );
  addDisplay( new_display, enabled );
  return new_display;
}

double VisualizationManager::getWallClock()
{
  return ros::WallTime::now().toSec();
}

double VisualizationManager::getROSTime()
{
  return (ros_time_begin_ + ros_time_elapsed_).toSec();
}

double VisualizationManager::getWallClockElapsed()
{
  return wall_clock_elapsed_.toSec();
}

double VisualizationManager::getROSTimeElapsed()
{
  return ros_time_elapsed_.toSec();
}

void VisualizationManager::updateBackgroundColor()
{
  render_panel_->setBackgroundColor( qtToOgre( background_color_property_->getColor() ));

  queueRender();
}

void VisualizationManager::handleChar( QKeyEvent* event, RenderPanel* panel )
{
  if( event->key() == Qt::Key_Escape )
  {
    setCurrentTool( getDefaultTool() );

    return;
  }
  if( current_tool_ )
  {
    current_tool_->processKeyEvent( event, panel );
  }
}

void VisualizationManager::addViewController(const std::string& class_name, const std::string& name)
{
  Q_EMIT viewControllerTypeAdded( class_name, name );
}

bool VisualizationManager::setCurrentViewControllerType(const std::string& type)
{
  if(view_controller_ && (view_controller_->getClassName() == type || view_controller_->getName() == type))
  {
    return true;
  }

  bool found = true;
  // hack hack hack hack until this becomes truly plugin based
  if(type == "rviz::OrbitViewController" || type == "Orbit")
  {
    view_controller_ = new OrbitViewController(this, "Orbit",target_scene_node_);
  }
  else if(type == "rviz::XYOrbitViewController" || type == "XYOrbit" ||
           type == "rviz::SimpleOrbitViewController" || type == "SimpleOrbit" /* the old class name */) 
  {
    view_controller_ = new XYOrbitViewController(this, "XYOrbit",target_scene_node_);
  }
  else if(type == "rviz::FPSViewController" || type == "FPS")
  {
    view_controller_ = new FPSViewController(this, "FPS",target_scene_node_);
  }
  else if(type == "rviz::FixedOrientationOrthoViewController" || type == "TopDownOrtho" || type == "Top-down Orthographic")
  {
    view_controller_ = new FixedOrientationOrthoViewController(this, "TopDownOrtho",target_scene_node_);
  }
  else if(!view_controller_)
  {
    view_controller_ = new OrbitViewController(this, "Orbit",target_scene_node_);
  }
  else
  {
    found = false;
  }

  if(found)
  {
    // RenderPanel::setViewController() deletes the old
    // ViewController, so don't do it here or it will crash!
    render_panel_->setViewController(view_controller_);
    view_controller_->setTargetFrame( target_frame_ );
    connect( view_controller_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
    Q_EMIT viewControllerChanged( view_controller_ );
    Q_EMIT configChanged();
  }

  return found;
}

std::string VisualizationManager::getCurrentViewControllerType()
{
  return view_controller_->getClassName();
}

void VisualizationManager::handleMouseEvent(ViewportMouseEvent& vme)
{
  boost::mutex::scoped_lock lock( vme_queue_mutex_ );
  vme_queue_.push_back(vme);
}

void VisualizationManager::threadedQueueThreadFunc()
{
  while (!shutting_down_)
  {
    threaded_queue_.callOne(ros::WallDuration(0.1));
  }
}

void VisualizationManager::notifyConfigChanged()
{
  Q_EMIT configChanged();
}

void VisualizationManager::updateFixedFrame()
{
  QString frame = fixed_frame_property_->getValue().toString();

  frame_manager_->setFixedFrame( frame.toStdString() );
  root_display_group_->setFixedFrame( frame );

  updateTargetFrame(); // in case it is FIXED_FRAME_STRING
}

void VisualizationManager::updateTargetFrame()
{
  if( view_controller_ )
  {
    view_controller_->setTargetFrame( getTargetFrame().toStdString() );
  }
}

QString VisualizationManager::getTargetFrame() const
{
  QString target = target_frame_property_->getValue().toString();
  if( target == TfFrameProperty::FIXED_FRAME_STRING )
  {
    target = getFixedFrame();
  }
  return target;
}

QString VisualizationManager::getFixedFrame() const
{
  return fixed_frame_property_->getValue().toString();
}

void VisualizationManager::setFixedFrame( const QString& frame )
{
  fixed_frame_property_->setValue( frame );
}

void VisualizationManager::setTargetFrame( const QString& frame )
{
  target_frame_property_->setValue( frame );
}

} // namespace rviz
