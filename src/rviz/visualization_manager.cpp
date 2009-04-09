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
#include "render_panel.h"
#include "displays_panel.h"
#include "viewport_mouse_event.h"

#include "display.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "common.h"
#include "factory.h"
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

#include <ros/common.h>
#include <ros/node.h>
#include <tf/transform_listener.h>

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

VisualizationManager::VisualizationManager( RenderPanel* render_panel )
: ogre_root_( Ogre::Root::getSingletonPtr() )
, current_tool_( NULL )
, render_panel_( render_panel )
, needs_reset_( false )
, new_ros_time_( false )
, time_update_timer_(0.0f)
, frame_update_timer_(0.0f)
, current_camera_(NULL)
, current_camera_type_(0)
, fps_camera_(NULL)
, orbit_camera_(NULL)
, top_down_ortho_(NULL)
{
  initializeCommon();
  registerFactories( this );

  ros_node_ = ros::Node::instance();

  /// @todo This should go away once creation of the ros::Node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::Node( "visualization_manager", ros::Node::DONT_HANDLE_SIGINT | ros::Node::ANONYMOUS_NAME );
  }
  ROS_ASSERT( ros_node_ );

  tf_ = new tf::TransformListener( *ros_node_, true, ros::Duration(10));

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  target_relative_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  updateRelativeNode();

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  update_timer_ = new wxTimer( this );
  update_timer_->Start( 33 );
  update_stopwatch_.Start();
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

  ros_node_->subscribe( "/time", time_message_, &VisualizationManager::incomingROSTime, this, 1 );

  createColorMaterials();

  selection_manager_ = new SelectionManager(this);
}

VisualizationManager::~VisualizationManager()
{
  ros_node_->unsubscribe( "/time", &VisualizationManager::incomingROSTime, this );

  Disconnect( wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler( VisualizationManager::onUpdate ), NULL, this );
  delete update_timer_;

  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it);
    display->disable(false);
    delete display;
  }
  displays_.clear();

  M_FactoryInfo::iterator factory_it = factories_.begin();
  M_FactoryInfo::iterator factory_end = factories_.end();
  for ( ; factory_it != factory_end; ++factory_it )
  {
    delete factory_it->second.factory_;
  }
  factories_.clear();

  V_Tool::iterator tool_it = tools_.begin();
  V_Tool::iterator tool_end = tools_.end();
  for ( ; tool_it != tool_end; ++tool_it )
  {
    delete *tool_it;
  }
  tools_.clear();

  delete tf_;

  delete property_manager_;

  delete fps_camera_;
  delete orbit_camera_;
  delete top_down_ortho_;

  delete selection_manager_;

  ogre_root_->destroySceneManager( scene_manager_ );
}

void VisualizationManager::initialize()
{
  orbit_camera_ = new ogre_tools::OrbitCamera( scene_manager_ );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  orbit_camera_->setPosition( 0, 0, 15 );
  orbit_camera_->setRelativeNode( getTargetRelativeNode() );
  addCamera(orbit_camera_, g_camera_type_names[camera_types::Orbit]);

  fps_camera_ = new ogre_tools::FPSCamera( scene_manager_ );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
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
  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it);

    displays.insert(display->getName());
  }
}

void VisualizationManager::onUpdate( wxTimerEvent& event )
{
  long millis = update_stopwatch_.Time();
  float dt = millis / 1000.0f;

  update_stopwatch_.Start();

  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it);

    if ( display->isEnabled() )
    {
      display->update( dt );
    }
  }

  updateRelativeNode();

  getCurrentCamera()->update();

  if ( needs_reset_ )
  {
    needs_reset_ = false;
    resetTime();
  }

  time_update_timer_ += dt;

  if ( time_update_timer_ > 0.1f )
  {
    time_update_timer_ = 0.0f;

    updateTime();
  }

  frame_update_timer_ += dt;

  if (frame_update_timer_ > 1.0f)
  {
    frame_update_timer_ = 0.0f;

    updateFrames();
  }

  selection_manager_->update();
  property_manager_->update();

  current_tool_->update(dt);
}

void VisualizationManager::updateTime()
{
  if ( new_ros_time_ )
  {
    time_message_.lock();

    if ( ros_time_begin_.is_zero() )
    {
      ros_time_begin_ = time_message_.rostime;
      wall_clock_begin_ = ros::WallTime::now();
    }

    ros_time_elapsed_ = time_message_.rostime - ros_time_begin_;

    time_message_.unlock();
  }

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
  resetDisplays();
  tf_->clear();

  ros_time_begin_ = ros::Time();
  wall_clock_begin_ = ros::WallTime();

  render_panel_->queueRender();
}

void VisualizationManager::addDisplay( Display* display, bool enabled )
{
  display_adding_(display);
  displays_.push_back( display );

  display->setRenderCallback( boost::bind( &RenderPanel::queueRender, render_panel_ ) );
  display->setLockRenderCallback( boost::bind( &RenderPanel::lockRender, render_panel_ ) );
  display->setUnlockRenderCallback( boost::bind( &RenderPanel::unlockRender, render_panel_ ) );

  display->setTargetFrame( target_frame_ );
  display->setFixedFrame( fixed_frame_ );
  display->setPropertyManager( property_manager_, CategoryPropertyWPtr() );

  display_added_(display);

  setDisplayEnabled( display, enabled );
}

void VisualizationManager::removeDisplay( Display* display )
{
  V_Display::iterator it = std::find(displays_.begin(), displays_.end(), display);
  ROS_ASSERT( it != displays_.end() );

  display_removing_(display);

  displays_.erase( it );
  display->disable(false);

  display_removed_(display);

  delete display;

  render_panel_->queueRender();
}

void VisualizationManager::removeAllDisplays()
{
  displays_removing_(displays_);

  while (!displays_.empty())
  {
    removeDisplay(displays_.back());
  }

  displays_removed_(V_Display());
}

void VisualizationManager::removeDisplay( const std::string& name )
{
  Display* display = getDisplay( name );

  if ( !display )
  {
    return;
  }

  removeDisplay( display );
}

void VisualizationManager::resetDisplays()
{
  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it);

    display->reset();
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

Display* VisualizationManager::getDisplay( const std::string& name )
{
  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    Display* display = (*vis_it);

    if ( display->getName() == name )
    {
      return display;
    }
  }

  return NULL;
}

void VisualizationManager::setDisplayEnabled( Display* display, bool enabled )
{
  if ( enabled )
  {
    display->enable();
  }
  else
  {
    display->disable();
  }

  display_state_( display );

  render_panel_->queueRender();
}

#define CAMERA_TYPE wxT("Camera Type")
#define CAMERA_CONFIG wxT("Camera Config")

void VisualizationManager::loadGeneralConfig( wxConfigBase* config )
{
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

  general_config_loaded_(config);
}

void VisualizationManager::saveGeneralConfig( wxConfigBase* config )
{
  general_config_saving_(config);

  config->Write(CAMERA_TYPE, wxString::FromAscii(getCurrentCameraType()));
  config->Write(CAMERA_CONFIG, wxString::FromAscii(getCurrentCamera()->toString().c_str()));
}

void VisualizationManager::loadDisplayConfig( wxConfigBase* config )
{
  int i = 0;
  while (1)
  {
    wxString type, name;
    type.Printf( wxT("Display%d/Type"), i );
    name.Printf( wxT("Display%d/Name"), i );

    wxString vis_type, vis_name;
    if ( !config->Read( type, &vis_type ) )
    {
      break;
    }

    if ( !config->Read( name, &vis_name ) )
    {
      break;
    }

    createDisplay( (const char*)vis_type.mb_str(), (const char*)vis_name.mb_str(), false );

    ++i;
  }

  property_manager_->load( config );

  displays_config_loaded_(config);
}

void VisualizationManager::saveDisplayConfig( wxConfigBase* config )
{
  int i = 0;
  V_Display::iterator vis_it = displays_.begin();
  V_Display::iterator vis_end = displays_.end();
  for ( ; vis_it != vis_end; ++vis_it, ++i )
  {
    Display* display = (*vis_it);

    wxString type, name;
    type.Printf( wxT("Display%d/Type"), i );
    name.Printf( wxT("Display%d/Name"), i );
    config->Write( type, wxString::FromAscii( display->getType() ) );
    config->Write( name, wxString::FromAscii( display->getName().c_str() ) );
  }

  property_manager_->save( config );

  displays_config_saving_(config);
}

bool VisualizationManager::registerFactory( const std::string& type, const std::string& description, DisplayFactory* factory )
{
  M_FactoryInfo::iterator it = factories_.find( type );
  if ( it != factories_.end() )
  {
    return false;
  }

  factories_.insert( std::make_pair( type, FactoryInfo( type, description, factory ) ) );

  return true;
}

Display* VisualizationManager::createDisplay( const std::string& type, const std::string& name, bool enabled )
{
  M_FactoryInfo::iterator it = factories_.find( type );
  if ( it == factories_.end() )
  {
    return NULL;
  }

  Display* current_vis = getDisplay( name );
  if ( current_vis )
  {
    return NULL;
  }

  DisplayFactory* factory = it->second.factory_;
  Display* display = factory->create( name, this );

  addDisplay( display, enabled );

  return display;
}

void VisualizationManager::setTargetFrame( const std::string& frame )
{
  if (target_frame_ == frame)
  {
    return;
  }

  target_frame_ = frame;

  V_Display::iterator it = displays_.begin();
  V_Display::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    Display* display = (*it);

    display->setTargetFrame(frame);
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
  if (fixed_frame_ == frame)
  {
    return;
  }

  fixed_frame_ = frame;

  V_Display::iterator it = displays_.begin();
  V_Display::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    Display* display = (*it);

    display->setFixedFrame(frame);
  }

  propertyChanged(fixed_frame_property_);
}

bool VisualizationManager::isValidDisplay( Display* display )
{
  V_Display::iterator it = displays_.begin();
  V_Display::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if (display == *it)
    {
      return true;
    }
  }

  return false;
}

void VisualizationManager::moveDisplayUp(Display* display)
{
  V_Display::iterator it = displays_.begin();
  V_Display::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if (display == *it)
    {
      if (it != displays_.begin())
      {
        uint32_t index = it - displays_.begin();
        displays_[index] = *(it - 1);
        displays_[index - 1] = display;
      }

      break;
    }
  }
}

void VisualizationManager::moveDisplayDown(Display* display)
{
  V_Display::iterator it = displays_.begin();
  V_Display::iterator end = displays_.end();
  for ( ; it != end; ++it )
  {
    if (display == *it)
    {
      if (it != (displays_.end() - 1))
      {
        uint32_t index = it - displays_.begin();
        displays_[index] = *(it + 1);
        displays_[index + 1] = display;
      }

      break;
    }
  }
}

void VisualizationManager::getRegisteredTypes( std::vector<std::string>& types, std::vector<std::string>& descriptions )
{
  M_FactoryInfo::iterator it = factories_.begin();
  M_FactoryInfo::iterator end = factories_.end();
  for ( ; it != end; ++it )
  {
    types.push_back( it->first );
    descriptions.push_back( it->second.description_ );
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

void VisualizationManager::incomingROSTime()
{
  static ros::Time last_time = ros::Time();

  if ( time_message_.rostime < last_time )
  {
    needs_reset_ = true;
  }

  last_time = time_message_.rostime;

  new_ros_time_ = true;
}

void VisualizationManager::setBackgroundColor(const Color& c)
{
  background_color_ = c;

  render_panel_->getViewport()->setBackgroundColour(Ogre::ColourValue(c.r_, c.g_, c.b_, 1.0f));

  propertyChanged(background_color_property_);

  render_panel_->queueRender();
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
    render_panel_->queueRender();
  }

  if ( flags & Tool::Finished )
  {
    setCurrentTool( getDefaultTool() );
  }
}

} // namespace rviz
