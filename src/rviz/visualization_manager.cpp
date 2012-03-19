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

#include "visualization_manager.h"
#include "selection/selection_manager.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "viewport_mouse_event.h"
#include "frame_manager.h"

#include "view_controller.h"
#include "view_controllers/xy_orbit_view_controller.h"
#include "view_controllers/orbit_view_controller.h"
#include "view_controllers/fps_view_controller.h"
#include "view_controllers/fixed_orientation_ortho_view_controller.h"

#include "display.h"
#include "display_wrapper.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "properties/forwards.h"
///// #include "new_display_dialog.h"

#include "tools/tool.h"
#include "tools/move_tool.h"
#include "tools/goal_tool.h"
#include "tools/initial_pose_tool.h"
#include "tools/selection_tool.h"
#include "tools/interaction_tool.h"

#include <ogre_helpers/qt_ogre_render_window.h>

#include <tf/transform_listener.h>

#include <ros/package.h>

#include "config.h"

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreLight.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderWindow.h>

#include <algorithm>

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
, target_frame_is_fixed_frame_(false)
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

  property_manager_ = new PropertyManager();
  tool_property_manager_ = new PropertyManager();

  connect( property_manager_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
  connect( tool_property_manager_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));

  CategoryPropertyWPtr options_category = property_manager_->createCategory( ".Global Options", "", CategoryPropertyWPtr(), this );
  target_frame_property_ = property_manager_->createProperty<TFFrameProperty>( "Target Frame", "", boost::bind( &VisualizationManager::getTargetFrame, this ),
                                                                              boost::bind( &VisualizationManager::setTargetFrame, this, _1 ), options_category, this );
  setPropertyHelpText(target_frame_property_, "Reference frame for the 3D camera view.");
  fixed_frame_property_ = property_manager_->createProperty<EditEnumProperty>( "Fixed Frame", "", boost::bind( &VisualizationManager::getFixedFrame, this ),
                                                                             boost::bind( &VisualizationManager::setFixedFrame, this, _1 ), options_category, this );
  setPropertyHelpText(fixed_frame_property_, "Frame into which all data is transformed before being displayed.");
  background_color_property_ = property_manager_->createProperty<ColorProperty>( "Background Color", "", boost::bind( &VisualizationManager::getBackgroundColor, this ),
                                                                             boost::bind( &VisualizationManager::setBackgroundColor, this, _1 ), options_category, this );
  setPropertyHelpText(background_color_property_, "Background color for the 3D view.");
  status_property_ = property_manager_->createStatus(".Global Status", "", CategoryPropertyWPtr(), this);

  CategoryPropertyPtr cat_prop = options_category.lock();
  cat_prop->collapse();

  setBackgroundColor(Color(0.0f, 0.0f, 0.0f));

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  threaded_queue_threads_.create_thread(boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));

  display_class_loader_ = new pluginlib::ClassLoader<Display>( "rviz", "rviz::Display" );
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

  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for (; it != end; ++it)
  {
    delete *it;
  }
  displays_.clear();

  V_Tool::iterator tool_it = tools_.begin();
  V_Tool::iterator tool_end = tools_.end();
  for ( ; tool_it != tool_end; ++tool_it )
  {
    delete *tool_it;
  }
  tools_.clear();

  delete display_class_loader_;
  delete property_manager_;
  delete tool_property_manager_;

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

  setFixedFrame("/map");
  setTargetFrame(FIXED_FRAME_STRING);

  render_panel_->getCamera()->setPosition(0, 10, 15);
  render_panel_->getCamera()->setNearClipDistance(0.01f);
  render_panel_->getCamera()->lookAt(0, 0, 0);

  addViewController(XYOrbitViewController::getClassNameStatic(), "XYOrbit");
  addViewController(OrbitViewController::getClassNameStatic(), "Orbit");
  addViewController(FPSViewController::getClassNameStatic(), "FPS");
  addViewController(FixedOrientationOrthoViewController::getClassNameStatic(), "TopDownOrtho");
  setCurrentViewControllerType(OrbitViewController::getClassNameStatic());

  MoveTool *move_tool = createTool< MoveTool >( "Move Camera", 'm' );
  setCurrentTool( move_tool );
  setDefaultTool( move_tool );

  createTool< InteractionTool >( "Interact", 'i' );
  createTool< SelectionTool >( "Select", 's' );
  createTool< GoalTool >( "2D Nav Goal", 'g' );
  createTool< InitialPoseTool >( "2D Pose Estimate", 'p' );

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

void VisualizationManager::getDisplayNames(S_string& displays)
{
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    displays.insert((*vis_it)->getName());
  }
}

std::string VisualizationManager::getTargetFrame()
{
  if(target_frame_is_fixed_frame_)
  {
    return FIXED_FRAME_STRING;
  }

  return target_frame_;
}

void VisualizationManager::queueRender()
{
// I believe the QTimer with zero duration (idle_timer_) makes this unnecessary.  The timer is always "awake".
//  if(!render_requested_)
//  {
//    w xWakeUpIdle();
//  }
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
    int flags = getCurrentTool()->processMouseEvent(vme);

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

  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it)->getDisplay();

    if( display && display->isEnabled() )
    {
      display->update( wall_dt, ros_dt );
    }
  }

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

  if(frame_count_ % 6 == 0)
  {
    property_manager_->update();
    tool_property_manager_->update();
  }

  current_tool_->update(wall_dt, ros_dt);

  disable_update_ = false;

// I believe the QTimer with zero duration (idle_timer_) makes this unnecessary.  The timer is always "awake".
//  w xWakeUpIdle();
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
  std::sort(frames.begin(), frames.end());

  EditEnumPropertyPtr target_prop = target_frame_property_.lock();
  EditEnumPropertyPtr fixed_prop = fixed_frame_property_.lock();
  StatusPropertyPtr status_prop = status_property_.lock();
  ROS_ASSERT(target_prop);
  ROS_ASSERT(fixed_prop);
  ROS_ASSERT(status_prop);

  if(frames != available_frames_)
  {
    fixed_prop->clear();

    V_string::iterator it = frames.begin();
    V_string::iterator end = frames.end();
    for (; it != end; ++it)
    {
      const std::string& frame = *it;

      if(frame.empty())
      {
        continue;
      }

      fixed_prop->addOption(frame);
    }

    available_frames_ = frames;
  }

  // Check the fixed frame to see if it's ok
  std::string error;
  if(frame_manager_->frameHasProblems(fixed_frame_, ros::Time(), error))
  {
    if(frames.empty())
    {
      fixed_prop->setToWarn();
      std::stringstream ss;
      ss << "No tf data.  Actual error: " << error;
      status_prop->setStatus(status_levels::Warn, "Fixed Frame", ss.str());
    }
    else
    {
      fixed_prop->setToError();
      status_prop->setStatus(status_levels::Error, "Fixed Frame", error);
    }
  }
  else
  {
    fixed_prop->setToOK();
    status_prop->setStatus(status_levels::Ok, "Fixed Frame", "OK");
  }

  if(frame_manager_->transformHasProblems(target_frame_, ros::Time(), error))
  {
    target_prop->setToError();
    status_prop->setStatus(status_levels::Error, "Target Frame", error);
  }
  else
  {
    target_prop->setToOK();
    status_prop->setStatus(status_levels::Ok, "Target Frame", "OK");
  }
}

tf::TransformListener* VisualizationManager::getTFClient()
{
  return frame_manager_->getTFClient();
}

void VisualizationManager::resetTime()
{
  resetDisplays();
  frame_manager_->getTFClient()->clear();

  ros_time_begin_ = ros::Time();
  wall_clock_begin_ = ros::WallTime();

  queueRender();
}

void VisualizationManager::onDisplayCreated(DisplayWrapper* wrapper)
{
  Display* display = wrapper->getDisplay();
  display->setRenderCallback( boost::bind( &VisualizationManager::queueRender, this ) );
  display->setLockRenderCallback( boost::bind( &VisualizationManager::lockRender, this ) );
  display->setUnlockRenderCallback( boost::bind( &VisualizationManager::unlockRender, this ) );

  display->setFixedFrame( fixed_frame_ );
}

bool VisualizationManager::addDisplay(DisplayWrapper* wrapper, bool enabled)
{
  if(getDisplayWrapper(wrapper->getName()))
  {
    ROS_ERROR("Display of name [%s] already exists", wrapper->getName().c_str());
    return false;
  }

  Q_EMIT displayAdding( wrapper );
  displays_.push_back(wrapper);

  connect( wrapper, SIGNAL( displayCreated( DisplayWrapper* )), this, SLOT( onDisplayCreated( DisplayWrapper* )));
  wrapper->setPropertyManager( property_manager_, CategoryPropertyWPtr() );
  wrapper->createDisplay();

  Q_EMIT displayAdded( wrapper );

  wrapper->setEnabled(enabled);

  return true;
}

void VisualizationManager::removeDisplay( DisplayWrapper* display )
{
  V_DisplayWrapper::iterator it = std::find(displays_.begin(), displays_.end(), display);
  ROS_ASSERT( it != displays_.end() );

  Q_EMIT displayRemoving( display );

  displays_.erase( it );

  Q_EMIT displayRemoved( display );

  delete display;

  queueRender();
}

void VisualizationManager::removeAllDisplays()
{
  Q_EMIT displaysRemoving( displays_ );

  while (!displays_.empty())
  {
    removeDisplay(displays_.back());
  }

  Q_EMIT displaysRemoved( V_DisplayWrapper() );
}

void VisualizationManager::removeDisplay( const std::string& name )
{
  DisplayWrapper* display = getDisplayWrapper( name );

  if( !display )
  {
    return;
  }

  removeDisplay( display );
}

void VisualizationManager::resetDisplays()
{
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it)->getDisplay();

    if(display)
    {
      display->reset();
    }
  }
}

void VisualizationManager::addTool( Tool* tool )
{
  tools_.push_back( tool );

  Q_EMIT toolAdded( tool );
}

void VisualizationManager::setCurrentTool( Tool* tool )
{
  if( current_tool_ )
  {
    current_tool_->deactivate();
  }

  current_tool_ = tool;
  current_tool_->activate();

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

  return tools_[ index ];
}

DisplayWrapper* VisualizationManager::getDisplayWrapper( const std::string& name )
{
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    DisplayWrapper* wrapper = *vis_it;
    if( wrapper->getName() == name )
    {
      return wrapper;
    }
  }

  return 0;
}

DisplayWrapper* VisualizationManager::getDisplayWrapper( Display* display )
{
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    DisplayWrapper* wrapper = *vis_it;
    if( wrapper->getDisplay() == display )
    {
      return wrapper;
    }
  }

  return 0;
}

// Make a map from class name (like "rviz::GridDisplay") to lookup
// name (like "rviz/Grid").  This is here because of a mismatch
// between the old rviz plugin system and pluginlib (the new one).
void makeClassNameToLookupNameMap( pluginlib::ClassLoader<Display>* class_loader,
                                   std::map<std::string, std::string>* map )
{
  std::vector<std::string> lookup_names = class_loader->getDeclaredClasses();

  std::vector<std::string>::const_iterator ni;
  for( ni = lookup_names.begin(); ni != lookup_names.end(); ni++ )
  {
    std::string class_name = class_loader->getClassType( (*ni) );
    (*map)[ class_name ] = (*ni);
  }
}

void VisualizationManager::loadDisplayConfig( const boost::shared_ptr<Config>& config, const StatusCallback& cb )
{
  disable_update_ = true;

  if(cb)
  {
    cb("Creating displays");
  }

  std::map<std::string, std::string> class_name_to_lookup_name;
  makeClassNameToLookupNameMap( display_class_loader_, &class_name_to_lookup_name );

  int i = 0;
  while (1)
  {
    std::string error = "";

    std::stringstream name, class_name, type;
    name << "Display" << i << "/Name";
    class_name << "Display" << i << "/ClassName";
    type << "Display" << i << "/Type";

    std::string vis_name, vis_class, vis_type, lookup_name;
    if(!config->get(name.str(), &vis_name))
    {
      break;
    }

    if(!config->get(class_name.str(), &vis_class))
    {
      if( config->get( type.str(), &vis_type ))
      {
        error = "This config file uses an old format with 'Type=" + vis_type +
          "'.  The new format uses the C++ class name, for example 'ClassName=rviz::GridDisplay' for a Grid display.";
        lookup_name = vis_type;
      }
      else
      {
        error = "This display has no 'ClassName' entry, so it cannot be created.";
      }
    }

    if( error == "" )
    {
      // TODO: should just read class-lookup-name from config file, but
      // that would not be consistent with the old (v1.6) config file format.
      lookup_name = class_name_to_lookup_name[ vis_class ];
      if( lookup_name == "" )
      {
        lookup_name = vis_class;
        error = "The class named '" + vis_class + "' was not found in rviz or any other plugin.";
      }
    }

    // Call createDisplay() even if there was an error so we can show
    // the name and the error in the Displays panel.
    DisplayWrapper* wrapper = createDisplay( lookup_name, vis_name, false);
    if( wrapper && error != "")
    {
      CategoryPropertyWPtr cat = wrapper->getCategory();
      setPropertyHelpText( cat, error );
    }

    ++i;
  }

  property_manager_->load( config, cb );
  tool_property_manager_->load( config, cb );

  std::string camera_type;
  if(config->get(CAMERA_TYPE, &camera_type))
  {
    if(setCurrentViewControllerType(camera_type))
    {
      std::string camera_config;
      if(config->get(CAMERA_CONFIG, &camera_config))
      {
        view_controller_->fromString(camera_config);
      }
    }
  }

  Q_EMIT displaysConfigLoaded( config );

  disable_update_ = false;
}

void VisualizationManager::saveDisplayConfig( const boost::shared_ptr<Config>& config )
{
  int i = 0;
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it, ++i )
  {
    DisplayWrapper* wrapper = *vis_it;

    std::stringstream name, class_name_key;
    name << "Display" << i << "/Name";
    class_name_key << "Display" << i << "/ClassName";
    config->set( name.str(), wrapper->getName() );
    std::string lookup_name = wrapper->getClassLookupName();
    // TODO: should just write class-lookup-name to config file, but
    // that would not be consistent with the old (v1.6) config file format.
    std::string class_name = display_class_loader_->getClassType( lookup_name );
    config->set( class_name_key.str(), class_name );
  }

  property_manager_->save( config );
  tool_property_manager_->save( config );

  if(view_controller_)
  {
    config->set(CAMERA_TYPE, view_controller_->getClassName());
    config->set(CAMERA_CONFIG, view_controller_->toString());
  }

  Q_EMIT displaysConfigSaved( config );
}

DisplayWrapper* VisualizationManager::createDisplay( const std::string& class_lookup_name,
                                                     const std::string& name,
                                                     bool enabled )
{
//  PluginPtr plugin = plugin_manager_->getPluginByPackage(package);
//  if(!plugin)
//  {
//    ROS_ERROR("Package [%s] does not have any plugins loaded available, for display of type [%s], and name [%s]", package.c_str(), class_name.c_str(), name.c_str());
//  }

  DisplayWrapper* wrapper = new DisplayWrapper( class_lookup_name, display_class_loader_, name, this);
  if(addDisplay(wrapper, enabled))
  {
    return wrapper;
  }
  else
  {
    delete wrapper;
    return 0;
  }
}

void VisualizationManager::setTargetFrame( const std::string& _frame )
{
  target_frame_is_fixed_frame_ = false;
  std::string frame = _frame;
  if(frame == FIXED_FRAME_STRING)
  {
    frame = fixed_frame_;
    target_frame_is_fixed_frame_ = true;
  }

  std::string remapped_name = frame_manager_->getTFClient()->resolve(frame);

  if(target_frame_ == remapped_name)
  {
    return;
  }

  target_frame_ = remapped_name;

  propertyChanged(target_frame_property_);

  if(view_controller_)
  {
    view_controller_->setTargetFrame(target_frame_);
  }
}

void VisualizationManager::setFixedFrame( const std::string& frame )
{
  std::string remapped_name = frame_manager_->getTFClient()->resolve(frame);

  if(fixed_frame_ == remapped_name)
  {
    return;
  }

  fixed_frame_ = remapped_name;

  frame_manager_->setFixedFrame(fixed_frame_);

  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    Display* display = (*it)->getDisplay();

    if(display)
    {
      display->setFixedFrame(fixed_frame_);
    }
  }

  propertyChanged(fixed_frame_property_);

  if(target_frame_is_fixed_frame_)
  {
    setTargetFrame(FIXED_FRAME_STRING);
  }
}

bool VisualizationManager::isValidDisplay(const DisplayWrapper* display)
{
  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if(display == (*it))
    {
      return true;
    }
  }

  return false;
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

void VisualizationManager::setBackgroundColor(const Color& c)
{
  background_color_ = c;

  render_panel_->setBackgroundColor(Ogre::ColourValue(c.r_, c.g_, c.b_, 1.0f));

  propertyChanged(background_color_property_);

  queueRender();
}

const Color& VisualizationManager::getBackgroundColor()
{
  return background_color_;
}

void VisualizationManager::handleChar( QKeyEvent* event, RenderPanel* panel )
{
  if( event->key() == Qt::Key_Escape )
  {
    setCurrentTool( getDefaultTool() );

    return;
  }
  getCurrentTool()->processKeyEvent( event, panel );
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

void VisualizationManager::onPluginUnloading(const PluginStatus& status)
{
  // We need to force an update of the property manager here, because the memory allocated for properties is done inside their shared objects.
  // If a plugin is unloaded and then later the update is called, the weak pointers can cause crashes, because the objects they point to are
  // no longer valid.
  property_manager_->update();
}

void VisualizationManager::notifyConfigChanged()
{
  Q_EMIT configChanged();
}

} // namespace rviz
