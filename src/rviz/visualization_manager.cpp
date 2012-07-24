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

#include "rviz/display.h"
#include "rviz/display_factory.h"
#include "rviz/display_group.h"
#include "rviz/displays_panel.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/render_panel.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/icon_cache.h"

#include "rviz/visualization_manager.h"

namespace rviz
{

VisualizationManager::VisualizationManager( RenderPanel* render_panel, WindowManagerInterface* wm )
: ogre_root_( Ogre::Root::getSingletonPtr() )
, update_timer_(0)
, shutting_down_(false)
, render_panel_( render_panel )
, time_update_timer_(0.0f)
, frame_update_timer_(0.0f)
, render_requested_(1)
, frame_count_(0)
, window_manager_(wm)
, disable_update_(false)
{
  frame_manager_ = new FrameManager();

  render_panel->setAutoRender(false);

  threaded_nh_.setCallbackQueue(&threaded_queue_);

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( -1, 0, -1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  root_display_group_ = new DisplayGroup();
  root_display_group_->setName( "root" );
  display_property_tree_model_ = new PropertyTreeModel( root_display_group_ );
  display_property_tree_model_->setDragDropClass( "display" );
  connect( display_property_tree_model_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
  
  tool_manager_ = new ToolManager( this );
  connect( tool_manager_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));

  view_manager_ = new ViewManager( this );

  global_options_ = new Property( "Global Options", QVariant(), "", root_display_group_ );

  fixed_frame_property_ = new TfFrameProperty( "Fixed Frame", "/map",
                                               "Frame into which all data is transformed before being displayed.",
                                               global_options_, frame_manager_, false,
                                               SLOT( updateFixedFrame() ), this );

  background_color_property_ = new ColorProperty( "Background Color", Qt::black,
                                                  "Background color for the 3D view.",
                                                  global_options_, SLOT( updateBackgroundColor() ), this );

  root_display_group_->initialize( this ); // only initialize() a Display after its sub-properties are created.
  root_display_group_->setEnabled( true );

  updateFixedFrame();
  updateBackgroundColor();

  global_status_ = new StatusList( "Global Status", root_display_group_ );

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  threaded_queue_threads_.create_thread(boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));

  display_factory_ = new DisplayFactory();

  icon_cache_ = new IconCache();
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
  delete tool_manager_;
  delete display_factory_;
  delete selection_manager_;

  if(ogre_root_)
  {
    ogre_root_->destroySceneManager( scene_manager_ );
  }
  delete frame_manager_;
}

void VisualizationManager::initialize(const StatusCallback& cb, bool verbose)
{
  if(cb)
  {
    cb("Initializing TF");
  }

  view_manager_->initialize();
  selection_manager_->initialize( verbose );
  tool_manager_->initialize();

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
    if( tool_manager_->getCurrentTool() )
    {
      flags = tool_manager_->getCurrentTool()->processMouseEvent(vme);
      event_it->panel->setCursor( tool_manager_->getCurrentTool()->getCursor( vme ) );
    }

    if( flags & Tool::Render )
    {
      queueRender();
    }

    if( flags & Tool::Finished )
    {
      tool_manager_->setCurrentTool( tool_manager_->getDefaultTool() );
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

  view_manager_->update(wall_dt, ros_dt);

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

  if( tool_manager_->getCurrentTool() )
  {
    tool_manager_->getCurrentTool()->update(wall_dt, ros_dt);
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
  if( frame_manager_->frameHasProblems( getFixedFrame().toStdString(), ros::Time(), error ))
  {
    if( frames.empty() )
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
  root_display_group_->addDisplay( display );
  display->initialize( this );
  display->setEnabled( enabled );
}

void VisualizationManager::removeAllDisplays()
{
  root_display_group_->removeAllDisplays();
}

void VisualizationManager::load( const YAML::Node& yaml_node, const StatusCallback& cb )
{
  disable_update_ = true;

  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "VisualizationManager::load() TODO: error handling - unexpected YAML type.\n" );
    disable_update_ = false;
    return;
  }
  
  if(cb)
  {
    cb("Creating displays");
  }
  root_display_group_->load( yaml_node );

  if(cb)
  {
    cb("Creating tools");
  }
  if( const YAML::Node *tools_node = yaml_node.FindValue( "Tools" ))
  {
    tool_manager_->load( *tools_node );
  }

  if(cb)
  {
    cb("Creating views");
  }
  if( const YAML::Node *views_node = yaml_node.FindValue( "Views" ))
  {
    view_manager_->load( *views_node );
  }

  disable_update_ = false;
}

void VisualizationManager::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;

  root_display_group_->saveChildren( emitter );

  emitter << YAML::Key << "Tools";
  emitter << YAML::Value;
  tool_manager_->save( emitter );

  emitter << YAML::Key << "Views";
  emitter << YAML::Value;
  view_manager_->save( emitter );

  emitter << YAML::EndMap;
}

Display* VisualizationManager::createDisplay( const QString& class_lookup_name,
                                              const QString& name,
                                              bool enabled )
{
  Display* new_display = root_display_group_->createDisplay( class_lookup_name );
  addDisplay( new_display, enabled );
  new_display->setName( name );
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

void VisualizationManager::handleMouseEvent( const ViewportMouseEvent& vme )
{
  boost::mutex::scoped_lock lock( vme_queue_mutex_ );
  vme_queue_.push_back(vme);
}

void VisualizationManager::handleChar( QKeyEvent* event, RenderPanel* panel )
{
  tool_manager_->handleChar( event, panel );
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
  QString frame = fixed_frame_property_->getFrame();

  frame_manager_->setFixedFrame( frame.toStdString() );
  root_display_group_->setFixedFrame( frame );
}

QString VisualizationManager::getFixedFrame() const
{
  return fixed_frame_property_->getFrame();
}

void VisualizationManager::setFixedFrame( const QString& frame )
{
  fixed_frame_property_->setValue( frame );
}

} // namespace rviz
