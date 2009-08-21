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
#include "rviz/common.h"
#include "rviz/window_manager_interface.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <ogre_tools/axes.h>

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

#include <wx/frame.h>

namespace rviz
{

CameraDisplay::RenderListener::RenderListener(CameraDisplay* display)
: display_(display)
{

}

void CameraDisplay::RenderListener::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  Ogre::Pass* pass = display_->material_->getTechnique(0)->getPass(0);

  if (pass->getNumTextureUnitStates() > 0)
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, display_->alpha_ );
  }
  else
  {
    display_->material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, display_->alpha_));
    display_->material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, display_->alpha_));
  }

  display_->updateCamera();
}

void CameraDisplay::RenderListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  Ogre::Pass* pass = display_->material_->getTechnique(0)->getPass(0);

  if (pass->getNumTextureUnitStates() > 0)
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0f );
  }
  else
  {
    display_->material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, 0.0f));
    display_->material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, 0.0f));
  }
}

CameraDisplay::CameraDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, caminfo_tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
, new_caminfo_(false)
, texture_(update_nh_)
, frame_(0)
, render_listener_(this)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "CameraDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    //material_->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    material_->getTechnique(0)->getPass(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);


#if 1
    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );
    tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );
#else
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f));
#endif

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    screen_rect_->setMaterial(material_->getName());
    scene_node_->attachObject(screen_rect_);

  }

  setAlpha( 0.5f );

  wxWindow* parent = 0;

  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  if (wm)
  {
    parent = wm->getParentWindow();
  }
  else
  {
    frame_ = new wxFrame(0, wxID_ANY, wxString::FromAscii(name.c_str()), wxDefaultPosition, wxDefaultSize, wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxRESIZE_BORDER | wxCAPTION | wxCLIP_CHILDREN);
    parent = frame_;
  }

  render_panel_ = new RenderPanel(parent, false);
  render_panel_->SetSize(wxSize(640, 480));
  if (wm)
  {
    wm->addPane(name, render_panel_);
  }

  render_panel_->createRenderWindow();

  render_panel_->setAutoRender(false);
  render_panel_->getRenderWindow()->addListener(&render_listener_);
  render_panel_->getViewport()->setOverlaysEnabled(false);
  render_panel_->getViewport()->setClearEveryFrame(true);
  render_panel_->getRenderWindow()->setActive(false);

  {
    std::stringstream ss;
    static uint32_t count = 0;
    ss << "CameraDisplayCamera" << count++;
    camera_ = scene_manager_->createCamera( ss.str() );
    render_panel_->setCamera(camera_);

    camera_->setPosition(Ogre::Vector3(-5, 5, 5));
    camera_->lookAt(Ogre::Vector3(0, 0, 0));
    camera_->setNearClipDistance( 0.1f );
  }

  caminfo_tf_filter_.connectInput(caminfo_sub_);
  caminfo_tf_filter_.registerCallback(boost::bind(&CameraDisplay::caminfoCallback, this, _1));
}

CameraDisplay::~CameraDisplay()
{
  unsubscribe();
  caminfo_tf_filter_.clear();

  if (frame_)
  {
    frame_->Destroy();
  }
  else
  {
    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    wm->removePane(render_panel_);
    render_panel_->Destroy();
  }

  scene_manager_->destroyCamera(camera_);
  delete screen_rect_;

  scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
}

void CameraDisplay::onEnable()
{
  subscribe();

  if (frame_)
  {
    frame_->Show(true);
  }
  else
  {
    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    wm->showPane(render_panel_);
  }

  render_panel_->getRenderWindow()->setActive(true);
}

void CameraDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);

  if (frame_)
  {
    frame_->Show(false);
  }
  else
  {
    WindowManagerInterface* wm = vis_manager_->getWindowManager();
    wm->closePane(render_panel_);
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

  texture_.setTopic(topic_);

  // parse out the namespace from the topic so we can subscribe to the caminfo
  std::string caminfo_topic = "cam_info";
  size_t pos = topic_.rfind('/');
  if (pos != std::string::npos)
  {
    std::string ns = topic_;
    ns.erase(pos);

    caminfo_topic = ns + "/" + caminfo_topic;
  }

  caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
}

void CameraDisplay::unsubscribe()
{
  texture_.setTopic("");
  caminfo_sub_.unsubscribe();
}

void CameraDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  causeRender();
}

void CameraDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;
  clear();

  subscribe();

  propertyChanged(topic_property_);
}

void CameraDisplay::clear()
{
  texture_.clear();

  new_caminfo_ = false;
  current_caminfo_.reset();
}


void CameraDisplay::update(float wall_dt, float ros_dt)
{
  texture_.update();
}

void CameraDisplay::updateCamera()
{
  sensor_msgs::CameraInfo::ConstPtr info;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);

    info = current_caminfo_;
  }

  if (!info)
  {
    return;
  }

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                              ros::Time(), info->header.frame_id );
  try
  {
    vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException& e)
  {

  }

  Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre(position);

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  Ogre::Quaternion orientation(quat.w(), quat.x(), quat.y(), quat.z());
  robotToOgre(orientation);

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float width = info->width;
  float height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (info->width == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], width = 0", getName().c_str());

    width = texture_.getWidth();
  }

  if (info->height == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], height = 0", getName().c_str());

    height = texture_.getHeight();
  }

  if (height == 0.0 || width == 0.0)
  {
    ROS_ERROR("Could not determine width/height of image, due to malformed CameraInfo");
    return;
  }

  double fx = info->P[0];
  double fy = info->P[5];
  double fovy = 2*atan(height / (2 * fy));
  double aspect_ratio = width / height;
  camera_->setFOVy(Ogre::Radian(fovy));
  camera_->setAspectRatio(aspect_ratio);

  // Add the camera's translation relative to the left camera (from P[3]);
  // Tx = -1*(P[3] / P[0])
  double tx = -1 * (info->P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  camera_->setPosition(position);
  camera_->setOrientation(orientation);

  double cx = info->P[2];
  double cy = info->P[6];
  double normalized_cx = cx / width;
  double normalized_cy = cy / height;
  double dx = 2*(0.5 - normalized_cx);
  double dy = 2*(normalized_cy - 0.5);
  screen_rect_->setCorners(-1.0f + dx, 1.0f + dy, 1.0f + dx, -1.0f + dy);

#if 0
  static ogre_tools::Axes* debug_axes = new ogre_tools::Axes(scene_manager_, 0, 0.2, 0.02);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif
}

void CameraDisplay::caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Image Topic", property_prefix_, boost::bind( &CameraDisplay::getTopic, this ),
                                                                         boost::bind( &CameraDisplay::setTopic, this, _1 ), parent_category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(sensor_msgs::Image::__s_getDataType());

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &CameraDisplay::getAlpha, this ),
                                                                      boost::bind( &CameraDisplay::setAlpha, this, _1 ), parent_category_, this );

}

void CameraDisplay::fixedFrameChanged()
{
  caminfo_tf_filter_.setTargetFrame(fixed_frame_);
}

void CameraDisplay::targetFrameChanged()
{

}

void CameraDisplay::reset()
{
  clear();
}

} // namespace rviz
