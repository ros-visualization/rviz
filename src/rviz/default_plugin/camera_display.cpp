/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "camera_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/window_manager_interface.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/display_wrapper.h"
#include "rviz/uniform_string_stream.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <rviz/ogre_helpers/axes.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>

namespace rviz
{

static const std::string IMAGE_POS_BACKGROUND = "background";
static const std::string IMAGE_POS_OVERLAY = "overlay";
static const std::string IMAGE_POS_BOTH = "background & overlay";

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.D);
  valid = valid && validateFloats(msg.K);
  valid = valid && validateFloats(msg.R);
  valid = valid && validateFloats(msg.P);
  return valid;
}

CameraDisplay::CameraDisplay()
  : Display()
  , zoom_(1)
  , transport_("raw")
  , image_position_(IMAGE_POS_BOTH)
  , caminfo_tf_filter_( 0 )
  , new_caminfo_(false)
  , texture_(update_nh_)
  , render_panel_( 0 )
  , force_render_(false)
  , panel_container_( 0 )
{
}

CameraDisplay::~CameraDisplay()
{
  unsubscribe();
  caminfo_tf_filter_->clear();

  if( render_panel_ )
  {
    if( panel_container_ )
    {
      delete panel_container_;
    }
    else
    {
      delete render_panel_;
    }
  }

  delete bg_screen_rect_;
  delete fg_screen_rect_;

  bg_scene_node_->getParentSceneNode()->removeAndDestroyChild(bg_scene_node_->getName());
  fg_scene_node_->getParentSceneNode()->removeAndDestroyChild(fg_scene_node_->getName());

  delete caminfo_tf_filter_;
}

void CameraDisplay::onInitialize()
{
  caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>(*vis_manager_->getTFClient(), "", 2, update_nh_);

  bg_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  fg_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    UniformStringStream ss;
    ss << "CameraDisplayObject" << count++;

    //background rectangle
    bg_screen_rect_ = new Ogre::Rectangle2D(true);
    bg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    bg_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    bg_material_->setDepthWriteEnabled(false);

    bg_material_->setReceiveShadows(false);
    bg_material_->setDepthCheckEnabled(false);

    bg_material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );
    tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );

    bg_material_->setCullingMode(Ogre::CULL_NONE);
    bg_material_->setSceneBlending( Ogre::SBT_REPLACE );

    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();

    bg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
    bg_screen_rect_->setBoundingBox(aabInf);
    bg_screen_rect_->setMaterial(bg_material_->getName());

    bg_scene_node_->attachObject(bg_screen_rect_);
    bg_scene_node_->setVisible(false);

    //overlay rectangle
    fg_screen_rect_ = new Ogre::Rectangle2D(true);
    fg_screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    fg_material_ = bg_material_->clone( ss.str()+"fg" );
    fg_screen_rect_->setBoundingBox(aabInf);
    fg_screen_rect_->setMaterial(fg_material_->getName());

    fg_material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    fg_screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    fg_scene_node_->attachObject(fg_screen_rect_);
    fg_scene_node_->setVisible(false);
  }

  setAlpha( 0.5f );

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->addListener( this );
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );
  render_panel_->resize( 640, 480 );
  render_panel_->initialize(vis_manager_->getSceneManager(), vis_manager_);

  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  if( wm )
  {
    panel_container_ = wm->addPane(name_, render_panel_);
  }
  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );

  caminfo_tf_filter_->connectInput(caminfo_sub_);
  caminfo_tf_filter_->registerCallback(boost::bind(&CameraDisplay::caminfoCallback, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

  if( panel_container_ )
  {
    // TODO: wouldn't it be better to connect this straight to the wrapper?
    connect( panel_container_, SIGNAL( visibilityChanged( bool ) ), this, SLOT( setWrapperEnabled( bool )));
  }
}

void CameraDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  bg_scene_node_->setVisible( image_position_ == IMAGE_POS_BACKGROUND || image_position_ == IMAGE_POS_BOTH );
  fg_scene_node_->setVisible( image_position_ == IMAGE_POS_OVERLAY || image_position_ == IMAGE_POS_BOTH );
}

void CameraDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  bg_scene_node_->setVisible(false);
  fg_scene_node_->setVisible(false);
}

void CameraDisplay::setWrapperEnabled( bool enabled )
{
  // Have to use the DisplayWrapper disable function so the checkbox
  // gets checked or unchecked, since it owns the "enabled" property.
  DisplayWrapper* wrapper = vis_manager_->getDisplayWrapper( this );
  if( wrapper != NULL )
  {
    wrapper->setEnabled( enabled );
  }
}

void CameraDisplay::onEnable()
{
  subscribe();
  if( render_panel_->parentWidget() == 0 )
  {
    render_panel_->show();
  }
  else
  {
    panel_container_->show();
  }

  render_panel_->getRenderWindow()->setActive(true);
}

void CameraDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);

  if( render_panel_->parentWidget() == 0 )
  {
    if( render_panel_->isVisible() )
    {
      render_panel_->hide();
    }
  }
  else
  {
    if( panel_container_->isVisible() )
    {
      panel_container_->hide();
    }
  }

  unsubscribe();

  clear();
}

void CameraDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    texture_.setTopic(topic_);
    setStatus( status_levels::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what() );
  }

  // parse out the namespace from the topic so we can subscribe to the caminfo
  std::string caminfo_topic = "camera_info";
  size_t pos = topic_.rfind('/');
  if (pos != std::string::npos)
  {
    std::string ns = topic_;
    ns.erase(pos);

    caminfo_topic = ns + "/" + caminfo_topic;
  }

  try
  {
    caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
    setStatus( status_levels::Ok, "Camera Info Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( status_levels::Error, "Camera Info Topic", std::string("Error subscribing: ") + e.what() );
  }
}

void CameraDisplay::unsubscribe()
{
  texture_.setTopic("");
  caminfo_sub_.unsubscribe();
}

void CameraDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  Ogre::Pass* pass = fg_material_->getTechnique(0)->getPass(0);
  if (pass->getNumTextureUnitStates() > 0)
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_ );
  }
  else
  {
    fg_material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
    fg_material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
  }

  propertyChanged(alpha_property_);
  force_render_ = true;
  causeRender();
}

void CameraDisplay::setZoom( float zoom )
{
  if (fabs(zoom) < .00001 || fabs(zoom) > 100000)
  {
    return;
  }
  zoom_ = zoom;

  propertyChanged(zoom_property_);

  force_render_ = true;
  causeRender();
}

void CameraDisplay::setQueueSize( int size )
{
  if( size != (int) caminfo_tf_filter_->getQueueSize() )
  {
    texture_.setQueueSize( (uint32_t) size );
    caminfo_tf_filter_->setQueueSize( (uint32_t) size );
    propertyChanged( queue_size_property_ );
  }
}

int CameraDisplay::getQueueSize()
{
  return (int) caminfo_tf_filter_->getQueueSize();
}

void CameraDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;
  clear();

  subscribe();

  propertyChanged(topic_property_);
}

void CameraDisplay::setTransport(const std::string& transport)
{
  transport_ = transport;

  texture_.setTransportType(transport);

  propertyChanged(transport_property_);
}

void CameraDisplay::setImagePosition(const std::string& image_position)
{
  image_position_ = image_position;

  propertyChanged(image_position_property_);

  force_render_ = true;
  causeRender();
}

void CameraDisplay::clear()
{
  texture_.clear();
  force_render_ = true;
  causeRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  setStatus(status_levels::Warn, "CameraInfo", "No CameraInfo received on [" + caminfo_sub_.getTopic() + "].  Topic may not exist.");
  setStatus(status_levels::Warn, "Image", "No Image received");

  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void CameraDisplay::updateStatus()
{
  if (texture_.getImageCount() == 0)
  {
    setStatus(status_levels::Warn, "Image", "No image received");
  }
  else
  {
    std::stringstream ss;
    ss << texture_.getImageCount() << " images received";
    setStatus(status_levels::Ok, "Image", ss.str());
  }
}

void CameraDisplay::update(float wall_dt, float ros_dt)
{
  updateStatus();

  try
  {
    if (texture_.update() || force_render_)
    {
      float old_alpha = alpha_;
      if (texture_.getImageCount() == 0)
      {
        alpha_ = 1.0f;
      }

      updateCamera();
      render_panel_->getRenderWindow()->update();
      alpha_ = old_alpha;

      force_render_ = false;
    }
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(status_levels::Error, "Image", e.what());
  }
}

void CameraDisplay::updateCamera()
{
  sensor_msgs::CameraInfo::ConstPtr info;
  sensor_msgs::Image::ConstPtr image;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);

    info = current_caminfo_;
    image = texture_.getImage();
  }

  if (!info || !image)
  {
    return;
  }

  if (!validateFloats(*info))
  {
    setStatus(status_levels::Error, "CameraInfo", "Contains invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  //uses the latest TF info to make sure 3D rendered view is up to date with rendered robot pose
  vis_manager_->getFrameManager()->getTransform(image->header.frame_id, ros::Time(0), position, orientation);

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], width = 0", getName().c_str());

    img_width = texture_.getWidth();
  }

  if (img_height == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], height = 0", getName().c_str());

    img_height = texture_.getHeight();
  }

  if (img_height == 0.0 || img_width == 0.0)
  {
    setStatus(status_levels::Error, "CameraInfo", "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return;
  }

  double fx = info->P[0];
  double fy = info->P[5];

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();
  float zoom_x = zoom_;
  float zoom_y = zoom_;

  //preserve aspect ratio
  if ( win_width != 0 && win_height != 0 )
  {
    float img_aspect = (img_width/fx) / (img_height/fy);
    float win_aspect = win_width / win_height;

    if ( img_aspect > win_aspect )
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->P[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!validateFloats(position))
  {
    setStatus(status_levels::Error, "CameraInfo", "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return;
  }

  render_panel_->getCamera()->setPosition(position);
  render_panel_->getCamera()->setOrientation(orientation);

  // calculate the projection matrix
  double cx = info->P[2];
  double cy = info->P[6];

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;
 
  proj_matrix[0][0]= 2.0 * fx/img_width * zoom_x;
  proj_matrix[1][1]= 2.0 * fy/img_height * zoom_y;

  proj_matrix[0][2]= 2.0 * (0.5 - cx/img_width) * zoom_x;
  proj_matrix[1][2]= 2.0 * (cy/img_height - 0.5) * zoom_y;

  proj_matrix[2][2]= -(far_plane+near_plane) / (far_plane-near_plane);
  proj_matrix[2][3]= -2.0*far_plane*near_plane / (far_plane-near_plane);

  proj_matrix[3][2]= -1;

  render_panel_->getCamera()->setCustomProjectionMatrix( true, proj_matrix );

  setStatus(status_levels::Ok, "CameraInfo", "OK");

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif

  //adjust the image rectangles to fit the zoom & aspect ratio
  bg_screen_rect_->setCorners(-1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y);
  fg_screen_rect_->setCorners(-1.0f*zoom_x, 1.0f*zoom_y, 1.0f*zoom_x, -1.0f*zoom_y);

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  bg_screen_rect_->setBoundingBox(aabInf);
  fg_screen_rect_->setBoundingBox(aabInf);
}

void CameraDisplay::caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraDisplay::onTransportEnumOptions(V_string& choices)
{
  texture_.getAvailableTransportTypes(choices);
}

void CameraDisplay::onImagePositionEnumOptions(V_string& choices)
{
  choices.clear();
  choices.push_back(IMAGE_POS_BACKGROUND);
  choices.push_back(IMAGE_POS_OVERLAY);
  choices.push_back(IMAGE_POS_BOTH);
}

void CameraDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Image Topic", property_prefix_, boost::bind( &CameraDisplay::getTopic, this ),
                                                                         boost::bind( &CameraDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::Image topic to subscribe to.  The topic must be a well-formed <strong>camera</strong> topic, and in order to work properly must have a matching <strong>camera_info<strong> topic.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

  transport_property_ = property_manager_->createProperty<EditEnumProperty>("Transport Hint", property_prefix_, boost::bind(&CameraDisplay::getTransport, this),
                                                                            boost::bind(&CameraDisplay::setTransport, this, _1), parent_category_, this);
  EditEnumPropertyPtr transport_prop = transport_property_.lock();
  transport_prop->setOptionCallback(boost::bind(&CameraDisplay::onTransportEnumOptions, this, _1));

  image_position_property_ = property_manager_->createProperty<EditEnumProperty>("Image Rendering", property_prefix_, boost::bind(&CameraDisplay::getImagePosition, this),
                                                                            boost::bind(&CameraDisplay::setImagePosition, this, _1), parent_category_, this);
  setPropertyHelpText(image_position_property_, "Render the image behind all other geometry or overlay it on top.");
  EditEnumPropertyPtr ip_prop = image_position_property_.lock();
  ip_prop->setOptionCallback(boost::bind(&CameraDisplay::onImagePositionEnumOptions, this, _1));

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Overlay Alpha", property_prefix_, boost::bind( &CameraDisplay::getAlpha, this ),
                                                                      boost::bind( &CameraDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "The amount of transparency to apply to the camera image when rendered as overlay.");

  zoom_property_ = property_manager_->createProperty<FloatProperty>("Zoom Factor", property_prefix_, boost::bind(&CameraDisplay::getZoom, this),
                                                                      boost::bind( &CameraDisplay::setZoom, this, _1), parent_category_, this);
  setPropertyHelpText(image_position_property_, "Set a zoom factor below 1 to see a larger part of the world, a factor above 1 to magnify the image.");

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &CameraDisplay::getQueueSize, this ),
                                                                         boost::bind( &CameraDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your camera data, but it can greatly increase memory usage if the messages are big." );
}

void CameraDisplay::fixedFrameChanged()
{
  caminfo_tf_filter_->setTargetFrame(fixed_frame_);
  texture_.setFrame(fixed_frame_, vis_manager_->getTFClient());
}

void CameraDisplay::reset()
{
  Display::reset();

  clear();
}

} // namespace rviz
