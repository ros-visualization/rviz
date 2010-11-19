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

#include "visualization_manager.h"
#include "selection/selection_manager.h"
#include "plugin/plugin_manager.h"
#include "plugin/display_type_info.h"
#include "render_panel.h"
#include "displays_panel.h"
#include "viewport_mouse_event.h"
#include "frame_manager.h"
#include "view_controller.h"
#include "view_controllers/orbit_view_controller.h"
#include "view_controllers/fps_view_controller.h"
#include "view_controllers/fixed_orientation_ortho_view_controller.h"

#include "display.h"
#include "display_wrapper.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "common.h"
#include "new_display_dialog.h"

#include "tools/tool.h"
#include "tools/move_tool.h"
#include "tools/goal_tool.h"
#include "tools/initial_pose_tool.h"
#include "tools/selection_tool.h"

#include <ogre_tools/wx_ogre_render_window.h>

#include <tf/transform_listener.h>

#include <ros/package.h>

#include <wx/timer.h>
#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>

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
  initializeCommon();

  frame_manager_ = FrameManager::instance();

  render_panel->setAutoRender(false);

  threaded_nh_.setCallbackQueue(&threaded_queue_);

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  property_manager_ = new PropertyManager();
  tool_property_manager_ = new PropertyManager();

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

  Ogre::ResourceGroupManager::getSingleton().createResourceGroup(ROS_PACKAGE_NAME );

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  threaded_queue_threads_.create_thread(boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));

  plugin_manager_ = new PluginManager;
  std::string rviz_path = ros::package::getPath("rviz");
  plugin_manager_->loadDescription(rviz_path + "/lib/default_plugin.yaml");
  plugin_manager_->loadDescriptions();

  {
    const L_Plugin& plugins = plugin_manager_->getPlugins();
    L_Plugin::const_iterator it = plugins.begin();
    L_Plugin::const_iterator end = plugins.end();
    for (; it != end; ++it)
    {
      const PluginPtr& plugin = *it;
      plugin->getUnloadingSignal().connect(boost::bind(&VisualizationManager::onPluginUnloading, this, _1));
    }
  }
}

VisualizationManager::~VisualizationManager()
{
  if (update_timer_)
  {
    Disconnect( wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );
    update_timer_->Stop();
    delete update_timer_;
  }

  shutting_down_ = true;
  threaded_queue_threads_.join_all();

  if (selection_manager_)
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

  delete plugin_manager_;
  delete property_manager_;
  delete tool_property_manager_;

  delete selection_manager_;

  if (ogre_root_)
  {
    ogre_root_->destroySceneManager( scene_manager_ );
  }
}

void VisualizationManager::initialize(const StatusCallback& cb)
{
  if (cb)
  {
    cb("Initializing TF");
  }

  setFixedFrame("/map");
  setTargetFrame(FIXED_FRAME_STRING);

  render_panel_->getCamera()->setPosition(0, 10, 15);
  render_panel_->getCamera()->setNearClipDistance(0.01f);
  render_panel_->getCamera()->lookAt(0, 0, 0);

  addViewController(OrbitViewController::getClassNameStatic(), "Orbit");
  addViewController(FPSViewController::getClassNameStatic(), "FPS");
  addViewController(FixedOrientationOrthoViewController::getClassNameStatic(), "TopDownOrtho");
  setCurrentViewControllerType(OrbitViewController::getClassNameStatic());

  MoveTool* move_tool = createTool< MoveTool >( "Move Camera", 'm' );
  setCurrentTool( move_tool );
  setDefaultTool( move_tool );

  createTool< SelectionTool >( "Select", 's' );
  createTool< GoalTool >( "2D Nav Goal", 'g' );
  createTool< InitialPoseTool >( "2D Pose Estimate", 'p' );

  selection_manager_->initialize();

  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();
}

void VisualizationManager::startUpdate()
{
  update_timer_ = new wxTimer( this );
  update_timer_->Start( 33 );
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );

  wxTheApp->Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(VisualizationManager::onIdle), NULL, this);
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
  if (target_frame_is_fixed_frame_)
  {
    return FIXED_FRAME_STRING;
  }

  return target_frame_;
}

void VisualizationManager::queueRender()
{
  if (!render_requested_)
  {
    wxWakeUpIdle();
  }

  render_requested_ = 1;
}

void VisualizationManager::onUpdate( wxTimerEvent& event )
{
  if (disable_update_)
  {
    return;
  }

  disable_update_ = true;

  ros::WallTime update_start = ros::WallTime::now();

  ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
  ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
  float wall_dt = wall_diff.toSec();
  float ros_dt = ros_diff.toSec();

  if (ros_dt < 0.0)
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

    if ( display && display->isEnabled() )
    {
      display->update( wall_dt, ros_dt );
    }
  }

  view_controller_->update(wall_dt, ros_dt);

  time_update_timer_ += wall_dt;

  if ( time_update_timer_ > 0.1f )
  {
    time_update_timer_ = 0.0f;

    updateTime();
  }

  frame_update_timer_ += wall_dt;

  if (frame_update_timer_ > 1.0f)
  {
    frame_update_timer_ = 0.0f;

    updateFrames();
  }

  selection_manager_->update();

  if (frame_count_ % 6 == 0)
  {
    property_manager_->update();
    tool_property_manager_->update();
  }

  current_tool_->update(wall_dt, ros_dt);

  disable_update_ = false;

  ++frame_count_;

  wxWakeUpIdle();
}

void VisualizationManager::onIdle(wxIdleEvent& evt)
{
  ros::WallTime cur = ros::WallTime::now();
  double dt = (cur - last_render_).toSec();

  if (dt > 0.1f)
  {
    render_requested_ = 1;
  }

  // Cap at 60fps
  if (render_requested_ && dt > 0.016f)
  {
    render_requested_ = 0;
    last_render_ = cur;

    boost::mutex::scoped_lock lock(render_mutex_);

    //ros::WallTime start = ros::WallTime::now();
    ogre_root_->renderOneFrame();
    //ros::WallTime end = ros::WallTime::now();
    //ros::WallDuration d = end - start;
    //ROS_INFO("Render took [%f] msec", d.toSec() * 1000.0f);
  }

  evt.Skip();
}

void VisualizationManager::updateTime()
{
  if ( ros_time_begin_.isZero() )
  {
    ros_time_begin_ = ros::Time::now();
  }

  ros_time_elapsed_ = ros::Time::now() - ros_time_begin_;

  if ( wall_clock_begin_.isZero() )
  {
    wall_clock_begin_ = ros::WallTime::now();
  }

  wall_clock_elapsed_ = ros::WallTime::now() - wall_clock_begin_;

  time_changed_();
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

  if (frames != available_frames_)
  {
    fixed_prop->clear();

    V_string::iterator it = frames.begin();
    V_string::iterator end = frames.end();
    for (; it != end; ++it)
    {
      const std::string& frame = *it;

      if (frame.empty())
      {
        continue;
      }

      fixed_prop->addOption(frame);
    }

    available_frames_ = frames;

    frames_changed_(frames);
  }

  // Check the fixed frame to see if it's ok
  std::string error;
  if (frame_manager_->frameHasProblems(fixed_frame_, ros::Time(), error))
  {
    if (frames.empty())
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

  if (frame_manager_->transformHasProblems(target_frame_, ros::Time(), error))
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

  display->setTargetFrame( target_frame_ );
  display->setFixedFrame( fixed_frame_ );
}

bool VisualizationManager::addDisplay(DisplayWrapper* wrapper, bool enabled)
{
  if (getDisplayWrapper(wrapper->getName()))
  {
    ROS_ERROR("Display of name [%s] already exists", wrapper->getName().c_str());
    return false;
  }

  display_adding_(wrapper);
  displays_.push_back(wrapper);

  wrapper->getDisplayCreatedSignal().connect(boost::bind(&VisualizationManager::onDisplayCreated, this, _1));
  wrapper->setPropertyManager( property_manager_, CategoryPropertyWPtr() );
  wrapper->createDisplay();

  display_added_(wrapper);

  wrapper->setEnabled(enabled);

  return true;
}

void VisualizationManager::removeDisplay( DisplayWrapper* display )
{
  V_DisplayWrapper::iterator it = std::find(displays_.begin(), displays_.end(), display);
  ROS_ASSERT( it != displays_.end() );

  display_removing_(display);

  displays_.erase( it );

  display_removed_(display);

  delete display;

  queueRender();
}

void VisualizationManager::removeAllDisplays()
{
  displays_removing_(displays_);

  while (!displays_.empty())
  {
    removeDisplay(displays_.back());
  }

  displays_removed_(V_DisplayWrapper());
}

void VisualizationManager::removeDisplay( const std::string& name )
{
  DisplayWrapper* display = getDisplayWrapper( name );

  if ( !display )
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

    if (display)
    {
      display->reset();
    }
  }
}

void VisualizationManager::addTool( Tool* tool )
{
  tools_.push_back( tool );

  tool_added_(tool);
}

void VisualizationManager::setCurrentTool( Tool* tool )
{
  if ( current_tool_ )
  {
    current_tool_->deactivate();
  }

  current_tool_ = tool;
  current_tool_->activate();

  tool_changed_(tool);
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
    if ( wrapper->getName() == name )
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
    if ( wrapper->getDisplay() == display )
    {
      return wrapper;
    }
  }

  return 0;
}

#define CAMERA_TYPE wxT("Camera Type")
#define CAMERA_CONFIG wxT("Camera Config")

void VisualizationManager::loadGeneralConfig( const boost::shared_ptr<wxConfigBase>& config, const StatusCallback& cb )
{
  // Legacy... read camera config from the general config (camera config is now saved in the display config).
  /// \todo Remove this once some time has passed
  wxString camera_type;
  if (config->Read(CAMERA_TYPE, &camera_type))
  {
    if (setCurrentViewControllerType((const char*)camera_type.char_str()))
    {
      wxString camera_config;
      if (config->Read(CAMERA_CONFIG, &camera_config))
      {
        view_controller_->fromString((const char*)camera_config.char_str());
      }
    }
  }

  if (cb)
  {
    cb("Loading plugins");
  }

  plugin_manager_->loadConfig(config);

  general_config_loaded_(config);
}

void VisualizationManager::saveGeneralConfig( const boost::shared_ptr<wxConfigBase>& config )
{
  plugin_manager_->saveConfig(config);
  general_config_saving_(config);
}

void VisualizationManager::loadDisplayConfig( const boost::shared_ptr<wxConfigBase>& config, const StatusCallback& cb )
{
  disable_update_ = true;

  if (cb)
  {
    cb("Creating displays");
  }

  int i = 0;
  while (1)
  {
    wxString name, package, class_name, type;
    name.Printf( wxT("Display%d/Name"), i );
    package.Printf( wxT("Display%d/Package"), i );
    class_name.Printf( wxT("Display%d/ClassName"), i );
    type.Printf(wxT("Display%d/Type"), i);

    wxString vis_name, vis_package, vis_class, vis_type;
    if (!config->Read(name, &vis_name))
    {
      break;
    }

    if (!config->Read(type, &vis_type))
    {
      if (!config->Read(package, &vis_package))
      {
        break;
      }

      if (!config->Read(class_name, &vis_class))
      {
        break;
      }
    }

    // Legacy support, for loading old config files
    if (!vis_type.IsEmpty())
    {
      std::string type = (const char*)vis_type.char_str();
      PluginPtr plugin = plugin_manager_->getPluginByDisplayName(type);
      if (plugin)
      {
        const DisplayTypeInfoPtr& info = plugin->getDisplayTypeInfoByDisplayName(type);
        createDisplay(plugin->getPackageName(), info->class_name, (const char*)vis_name.char_str(), false);
      }
      else
      {
        ROS_WARN("Display type [%s] no longer exists for display [%s]", type.c_str(), (const char*)vis_name.char_str());
      }
    }
    else
    {
      createDisplay((const char*)vis_package.char_str(), (const char*)vis_class.char_str(), (const char*)vis_name.char_str(), false);
    }

    ++i;
  }

  property_manager_->load( config, cb );
  tool_property_manager_->load( config, cb );

  wxString camera_type;
  if (config->Read(CAMERA_TYPE, &camera_type))
  {
    if (setCurrentViewControllerType((const char*)camera_type.char_str()))
    {
      wxString camera_config;
      if (config->Read(CAMERA_CONFIG, &camera_config))
      {
        view_controller_->fromString((const char*)camera_config.char_str());
      }
    }
  }

  displays_config_loaded_(config);

  disable_update_ = false;
}

void VisualizationManager::saveDisplayConfig( const boost::shared_ptr<wxConfigBase>& config )
{
  int i = 0;
  V_DisplayWrapper::iterator vis_it = displays_.begin();
  V_DisplayWrapper::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it, ++i )
  {
    DisplayWrapper* wrapper = *vis_it;

    wxString name, package, class_name;
    name.Printf( wxT("Display%d/Name"), i );
    package.Printf( wxT("Display%d/Package"), i );
    class_name.Printf( wxT("Display%d/ClassName"), i );
    config->Write( name, wxString::FromAscii( wrapper->getName().c_str() ) );
    config->Write( package, wxString::FromAscii( wrapper->getPackage().c_str() ) );
    config->Write( class_name, wxString::FromAscii( wrapper->getClassName().c_str() ) );
  }

  property_manager_->save( config );
  tool_property_manager_->save( config );

  if (view_controller_)
  {
    config->Write(CAMERA_TYPE, wxString::FromAscii(view_controller_->getClassName().c_str()));
    config->Write(CAMERA_CONFIG, wxString::FromAscii(view_controller_->toString().c_str()));
  }

  displays_config_saving_(config);
}

DisplayWrapper* VisualizationManager::createDisplay( const std::string& package, const std::string& class_name, const std::string& name, bool enabled )
{
  PluginPtr plugin = plugin_manager_->getPluginByPackage(package);
  if (!plugin)
  {
    ROS_ERROR("Package [%s] does not have any plugins loaded available, for display of type [%s], and name [%s]", package.c_str(), class_name.c_str(), name.c_str());
  }

  DisplayWrapper* wrapper = new DisplayWrapper(package, class_name, plugin, name, this);
  if (addDisplay(wrapper, enabled))
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
  if (frame == FIXED_FRAME_STRING)
  {
    frame = fixed_frame_;
    target_frame_is_fixed_frame_ = true;
  }

  std::string remapped_name = frame_manager_->getTFClient()->resolve(frame);

  if (target_frame_ == remapped_name)
  {
    return;
  }

  target_frame_ = remapped_name;

  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    Display* display = (*it)->getDisplay();

    if (display)
    {
      display->setTargetFrame(target_frame_);
    }
  }

  propertyChanged(target_frame_property_);

  if (view_controller_)
  {
    view_controller_->setReferenceFrame(target_frame_);
  }
}

void VisualizationManager::setFixedFrame( const std::string& frame )
{
  std::string remapped_name = frame_manager_->getTFClient()->resolve(frame);

  if (fixed_frame_ == remapped_name)
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

    if (display)
    {
      display->setFixedFrame(fixed_frame_);
    }
  }

  propertyChanged(fixed_frame_property_);

  if (target_frame_is_fixed_frame_)
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
    if (display == (*it))
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

  render_panel_->getViewport()->setBackgroundColour(Ogre::ColourValue(c.r_, c.g_, c.b_, 1.0f));

  propertyChanged(background_color_property_);

  queueRender();
}

const Color& VisualizationManager::getBackgroundColor()
{
  return background_color_;
}

void VisualizationManager::handleChar( wxKeyEvent& event )
{
  if ( event.GetKeyCode() == WXK_ESCAPE )
  {
    setCurrentTool( getDefaultTool() );

    return;
  }

  char key = event.GetKeyCode();
  V_Tool::iterator it = tools_.begin();
  V_Tool::iterator end = tools_.end();
  for ( ; it != end; ++it )
  {
    Tool* tool = *it;
    if ( tool->getShortcutKey() == key && tool != getCurrentTool() )
    {
      setCurrentTool( tool );
      return;
    }
  }

  getCurrentTool()->processKeyEvent(event);
}

void VisualizationManager::addViewController(const std::string& class_name, const std::string& name)
{
  view_controller_type_added_(class_name, name);
}

bool VisualizationManager::setCurrentViewControllerType(const std::string& type)
{
  if (view_controller_ && (view_controller_->getClassName() == type || view_controller_->getName() == type))
  {
    return true;
  }

  bool found = true;
  // hack hack hack hack until this becomes truly plugin based
  if (type == "rviz::OrbitViewController" || type == "Orbit")
  {
    view_controller_ = new OrbitViewController(this, "Orbit");
  }
  else if (type == "rviz::FPSViewController" || type == "FPS")
  {
    view_controller_ = new FPSViewController(this, "FPS");
  }
  else if (type == "rviz::FixedOrientationOrthoViewController" || type == "TopDownOrtho" || type == "Top-down Orthographic")
  {
    FixedOrientationOrthoViewController* controller = new FixedOrientationOrthoViewController(this, "TopDownOrtho");
    Ogre::Quaternion orient;
    orient.FromAngleAxis(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
    controller->setOrientation(orient);
    view_controller_ = controller;
  }
  else if (!view_controller_)
  {
    view_controller_ = new OrbitViewController(this, "Orbit");
  }
  else
  {
    found = false;
  }

  if (found)
  {
    render_panel_->setViewController(view_controller_);
    view_controller_type_changed_(view_controller_);
  }

  return found;
}

std::string VisualizationManager::getCurrentViewControllerType()
{
  return view_controller_->getClassName();
}

void VisualizationManager::handleMouseEvent(ViewportMouseEvent& vme)
{
  int flags = getCurrentTool()->processMouseEvent(vme);

  if ( flags & Tool::Render )
  {
    queueRender();
  }

  if ( flags & Tool::Finished )
  {
    setCurrentTool( getDefaultTool() );
  }
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

} // namespace rviz
