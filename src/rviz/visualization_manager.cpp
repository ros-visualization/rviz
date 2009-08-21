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

#include "display.h"
#include "display_wrapper.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "common.h"
#include "new_display_dialog.h"

#include "tools/tool.h"
#include "tools/move_tool.h"
#include "tools/pose_tool.h"
#include "tools/selection_tool.h"

#include <ogre_tools/wx_ogre_render_window.h>
#include <ogre_tools/camera_base.h>
#include "ogre_tools/fps_camera.h"
#include "ogre_tools/orbit_camera.h"
#include "ogre_tools/ortho_camera.h"

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

namespace camera_types
{
enum CameraType
{
  Orbit,
  FPS,
  TopDownOrtho,

  Count
};
}
typedef camera_types::CameraType CameraType;

static const char* g_camera_type_names[camera_types::Count] =
{
  "Orbit",
  "FPS",
  "Top-down Orthographic"
};

VisualizationManager::VisualizationManager( RenderPanel* render_panel, WindowManagerInterface* wm )
: ogre_root_( Ogre::Root::getSingletonPtr() )
, shutting_down_(false)
, current_tool_( NULL )
, render_panel_( render_panel )
, time_update_timer_(0.0f)
, frame_update_timer_(0.0f)
, current_camera_(NULL)
, current_camera_type_(0)
, fps_camera_(NULL)
, orbit_camera_(NULL)
, top_down_ortho_(NULL)
, render_requested_(1)
, render_timer_(0.0f)
, skip_render_(0)
, window_manager_(wm)
{
  initializeCommon();

  render_panel->setAutoRender(false);

  update_nh_.setCallbackQueue(&update_queue_);
  threaded_nh_.setCallbackQueue(&threaded_queue_);
  tf_ = new tf::TransformListener(update_nh_, ros::Duration(10 * 60), false);
  threaded_tf_ = new tf::TransformListener(threaded_nh_, ros::Duration(10 * 60), false);

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  target_relative_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateRelativeNode();

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  update_timer_ = new wxTimer( this );
  update_timer_->Start( 33 );
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );

  property_manager_ = new PropertyManager();

  CategoryPropertyWPtr options_category = property_manager_->createCategory( ".Global Options", "" );
  target_frame_property_ = property_manager_->createProperty<EditEnumProperty>( "Target Frame", "", boost::bind( &VisualizationManager::getTargetFrame, this ),
                                                                              boost::bind( &VisualizationManager::setTargetFrame, this, _1 ), options_category );
  fixed_frame_property_ = property_manager_->createProperty<EditEnumProperty>( "Fixed Frame", "", boost::bind( &VisualizationManager::getFixedFrame, this ),
                                                                             boost::bind( &VisualizationManager::setFixedFrame, this, _1 ), options_category );
  background_color_property_ = property_manager_->createProperty<ColorProperty>( "Background Color", "", boost::bind( &VisualizationManager::getBackgroundColor, this ),
                                                                             boost::bind( &VisualizationManager::setBackgroundColor, this, _1 ), options_category );
  CategoryPropertyPtr cat_prop = options_category.lock();
  cat_prop->collapse();

  setTargetFrame( "base_link" );
  setFixedFrame( "/map" );
  setBackgroundColor(Color(0.0f, 0.0f, 0.0f));

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);

  for (uint32_t i = 0; i < boost::thread::hardware_concurrency(); ++i)
  {
    threaded_queue_threads_.create_thread(boost::bind(&VisualizationManager::threadedQueueThreadFunc, this));
  }

  plugin_manager_ = new PluginManager;
  std::string rviz_path = ros::package::getPath("rviz");
  plugin_manager_->loadDescription(rviz_path + "/lib/default_plugin.yaml");
  plugin_manager_->loadDescriptions();
}

VisualizationManager::~VisualizationManager()
{
  Disconnect( wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );
  update_timer_->Stop();
  delete update_timer_;

  shutting_down_ = true;
  threaded_queue_threads_.join_all();

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

  delete tf_;
  delete threaded_tf_;

  delete property_manager_;

  delete fps_camera_;
  delete orbit_camera_;
  delete top_down_ortho_;

  delete selection_manager_;
  delete plugin_manager_;

  ogre_root_->destroySceneManager( scene_manager_ );
}

void VisualizationManager::initialize()
{
  orbit_camera_ = new ogre_tools::OrbitCamera( scene_manager_ );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.01f );
  orbit_camera_->setPosition( 0, 0, 15 );
  orbit_camera_->setRelativeNode( getTargetRelativeNode() );
  addCamera(orbit_camera_, g_camera_type_names[camera_types::Orbit]);

  fps_camera_ = new ogre_tools::FPSCamera( scene_manager_ );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.01f );
  fps_camera_->setPosition( 0, 0, 15 );
  fps_camera_->setRelativeNode( getTargetRelativeNode() );
  addCamera(fps_camera_, g_camera_type_names[camera_types::FPS]);

  top_down_ortho_ = new ogre_tools::OrthoCamera( render_panel_, scene_manager_ );
  top_down_ortho_->setPosition( 0, 30, 0 );
  top_down_ortho_->pitch( -Ogre::Math::HALF_PI );
  top_down_ortho_->setRelativeNode( getTargetRelativeNode() );
  addCamera(top_down_ortho_, g_camera_type_names[camera_types::TopDownOrtho]);

  current_camera_ = orbit_camera_;
  current_camera_type_ = camera_types::Orbit;

  render_panel_->getViewport()->setCamera( current_camera_->getOgreCamera() );

  MoveTool* move_tool = createTool< MoveTool >( "Move Camera", 'm' );
  setCurrentTool( move_tool );
  setDefaultTool( move_tool );

  createTool< SelectionTool >( "Select", 's' );

  PoseTool* goal_tool = createTool< PoseTool >( "Set Goal", 'g' );
  goal_tool->setIsGoal( true );

  createTool< PoseTool >( "Set Pose", 'p' );

  selection_manager_->initialize();

  last_update_ros_time_ = ros::Time::now();
  last_update_wall_time_ = ros::WallTime::now();
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

void VisualizationManager::queueRender()
{
  render_requested_ = 1;
}

void VisualizationManager::onUpdate( wxTimerEvent& event )
{
  ros::WallDuration wall_diff = ros::WallTime::now() - last_update_wall_time_;
  ros::Duration ros_diff = ros::Time::now() - last_update_ros_time_;
  float wall_dt = wall_diff.toSec();
  float ros_dt = ros_diff.toSec();

  if (ros_dt < 0.0)
  {
    resetTime();
  }

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

  update_queue_.callAvailable(ros::WallDuration());

  updateRelativeNode();

  getCurrentCamera()->update();

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
  property_manager_->update();

  current_tool_->update(wall_dt, ros_dt);

  render_timer_ += wall_dt;
  if (render_timer_ > 0.1f)
  {
    render_requested_ = 1;
  }

  if (!skip_render_)
  {
    // Cap at 60fps
    if (render_requested_ && render_timer_ > 0.016f)
    {
      render_requested_ = 0;
      render_timer_ = 0.0f;

      boost::mutex::scoped_lock lock(render_mutex_);

      ros::WallTime start = ros::WallTime::now();
      ogre_root_->renderOneFrame();
      ros::WallTime end = ros::WallTime::now();
      ros::WallDuration d = end - start;
      //ROS_INFO("Render took [%f] msec", d.toSec() * 1000.0f);
      if (d.toSec() > 0.033f)
      {
        skip_render_ = floor(d.toSec() / 0.033f);
      }
    }
  }
  else
  {
    --skip_render_;
  }
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
  tf_->getFrameStrings( frames );
  std::sort(frames.begin(), frames.end());

  EditEnumPropertyPtr target_prop = target_frame_property_.lock();
  EditEnumPropertyPtr fixed_prop = fixed_frame_property_.lock();
  ROS_ASSERT(target_prop);
  ROS_ASSERT(fixed_prop);

  if (frames != available_frames_)
  {
    bool target = property_manager_->getPropertyGrid()->GetSelectedProperty() != target_prop->getPGProperty();
    bool fixed = property_manager_->getPropertyGrid()->GetSelectedProperty() != fixed_prop->getPGProperty();

    if (target)
    {
      target_prop->clear();
    }

    if (fixed)
    {
      fixed_prop->clear();
    }

    V_string::iterator it = frames.begin();
    V_string::iterator end = frames.end();
    for (; it != end; ++it)
    {
      const std::string& frame = *it;

      if (frame.empty())
      {
        continue;
      }

      if (target)
      {
        target_prop->addOption(frame);
      }

      if (fixed)
      {
        fixed_prop->addOption(frame);
      }
    }

    available_frames_ = frames;

    frames_changed_(frames);
  }
}

void VisualizationManager::resetTime()
{
  skip_render_ = 0;
  resetDisplays();
  tf_->clear();
  threaded_tf_->clear();

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

  if (wrapper->getDisplay())
  {
    wrapper->getDisplay()->setEnabled(enabled);
  }

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

void VisualizationManager::loadGeneralConfig( const boost::shared_ptr<wxConfigBase>& config )
{
  // Legacy... read camera config from the general config (camera config is now saved in the display config).
  /// \todo Remove this once some time has passed
  wxString camera_type;
  if (config->Read(CAMERA_TYPE, &camera_type))
  {
    if (setCurrentCamera((const char*)camera_type.fn_str()))
    {
      wxString camera_config;
      if (config->Read(CAMERA_CONFIG, &camera_config))
      {
        getCurrentCamera()->fromString((const char*)camera_config.fn_str());
      }
    }
  }

  plugin_manager_->loadConfig(config);

  general_config_loaded_(config);
}

void VisualizationManager::saveGeneralConfig( const boost::shared_ptr<wxConfigBase>& config )
{
  plugin_manager_->saveConfig(config);
  general_config_saving_(config);
}

void VisualizationManager::loadDisplayConfig( const boost::shared_ptr<wxConfigBase>& config )
{
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
    }
    else
    {
      createDisplay((const char*)vis_package.char_str(), (const char*)vis_class.char_str(), (const char*)vis_name.char_str(), false);
    }

    ++i;
  }

  property_manager_->load( config );

  wxString camera_type;
  if (config->Read(CAMERA_TYPE, &camera_type))
  {
    if (setCurrentCamera((const char*)camera_type.fn_str()))
    {
      wxString camera_config;
      if (config->Read(CAMERA_CONFIG, &camera_config))
      {
        getCurrentCamera()->fromString((const char*)camera_config.fn_str());
      }
    }
  }

  displays_config_loaded_(config);
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

  if (getCurrentCamera())
  {
    config->Write(CAMERA_TYPE, wxString::FromAscii(getCurrentCameraType()));
    config->Write(CAMERA_CONFIG, wxString::FromAscii(getCurrentCamera()->toString().c_str()));
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

void VisualizationManager::setTargetFrame( const std::string& frame )
{
  std::string remapped_name = tf::remap(tf_->getTFPrefix(), frame);

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

  updateRelativeNode();

  if ( getCurrentCamera() )
  {
    getCurrentCamera()->lookAt( target_relative_node_->getPosition() );
  }
}

void VisualizationManager::setFixedFrame( const std::string& frame )
{
  std::string remapped_name = tf::remap(tf_->getTFPrefix(), frame);

  if (fixed_frame_ == remapped_name)
  {
    return;
  }

  fixed_frame_ = remapped_name;

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

void VisualizationManager::moveDisplayUp(DisplayWrapper* display)
{
  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if (display == *it)
    {
      if (it != displays_.begin())
      {
        uint32_t index = it - displays_.begin();
        std::swap(displays_[index], displays_[index - 1]);
      }

      break;
    }
  }
}

void VisualizationManager::moveDisplayDown(DisplayWrapper* display)
{
  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if (display == *it)
    {
      if (it != (displays_.end() - 1))
      {
        uint32_t index = it - displays_.begin();
        std::swap(displays_[index], displays_[index + 1]);
      }

      break;
    }
  }
}

void VisualizationManager::updateRelativeNode()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                              ros::Time(), target_frame_ );

  typedef std::vector<std::string> V_string;
  V_string frames;
  tf_->getFrameStrings( frames );

  bool has_fixed_frame = std::find( frames.begin(), frames.end(), fixed_frame_ ) != frames.end();
  bool has_target_frame = std::find( frames.begin(), frames.end(), target_frame_ ) != frames.end();

  if (has_fixed_frame && has_target_frame && tf_->canTransform(fixed_frame_, target_frame_, ros::Time()))
  {
    try
    {
      tf_->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s': %s", target_frame_.c_str(), fixed_frame_.c_str(), e.what() );
    }

    Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
    robotToOgre( position );

    btQuaternion quat;
    pose.getBasis().getRotation( quat );
    Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
    ogreToRobot( orientation );
    orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
    robotToOgre( orientation );

    target_relative_node_->setPosition( position );
    target_relative_node_->setOrientation( orientation );
  }
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

void VisualizationManager::addCamera(ogre_tools::CameraBase* camera, const std::string& name)
{
  camera_type_added_(camera, name);
}

void VisualizationManager::setCurrentCamera(int camera_type)
{
  if (camera_type == current_camera_type_)
  {
    return;
  }

  ogre_tools::CameraBase* prev_camera = current_camera_;

  bool set_from_old = false;

  switch ( camera_type )
  {
  case camera_types::FPS:
    {
      if ( current_camera_ == orbit_camera_ )
      {
        set_from_old = true;
      }

      current_camera_ = fps_camera_;
    }
    break;

  case camera_types::Orbit:
    {
      if ( current_camera_ == fps_camera_ )
      {
        set_from_old = true;
      }

      current_camera_ = orbit_camera_;
    }
    break;

  case camera_types::TopDownOrtho:
    {
      current_camera_ = top_down_ortho_;
    }
    break;
  }

  if ( set_from_old )
  {
    current_camera_->setFrom( prev_camera );
  }

  current_camera_type_ = camera_type;
  render_panel_->setCamera( current_camera_->getOgreCamera() );

  camera_type_changed_(current_camera_);
}

bool VisualizationManager::setCurrentCamera(const std::string& camera_type)
{
  for (int i = 0; i < camera_types::Count; ++i)
  {
    if (g_camera_type_names[i] == camera_type)
    {
      setCurrentCamera(i);
      return true;
    }
  }

  return false;
}

const char* VisualizationManager::getCurrentCameraType()
{
  return g_camera_type_names[current_camera_type_];
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

} // namespace rviz
