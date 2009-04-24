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
#include "visualization_manager.h"
#include "render_panel.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"
#include "window_manager_interface.h"

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

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
  display_->screen_rect_->setVisible(true);
  display_->updateCamera();
}

void CameraDisplay::RenderListener::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  display_->screen_rect_->setVisible(false);

#if 0
  Ogre::ManualObject::ManualObjectSection* section = display_->screen_rect_->getSection(0);
  Ogre::Matrix4 world_mat;
  section->getWorldTransforms(&world_mat);
  Ogre::Matrix4 proj_mat;
  Ogre::Root::getSingleton().getRenderSystem()->_convertProjectionMatrix(Ogre::Matrix4::IDENTITY, proj_mat);
  display_->scene_manager_->manualRender(section->getRenderOperation(),
                                         section->getTechnique()->getPass(0),
                                         display_->render_panel_->getViewport(),
                                         world_mat,
                                         Ogre::Matrix4::IDENTITY,
                                         proj_mat,
                                         false);
#endif
}

CameraDisplay::CameraDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, new_image_(false)
, new_caminfo_(false)
, frame_(0)
, render_listener_(this)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "CameraDisplayObject" << count++;

    screen_rect_ = scene_manager_->createManualObject(ss.str());
    screen_rect_->setUseIdentityProjection(true);
    screen_rect_->setUseIdentityView(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    //material_->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    material_->getTechnique(0)->getPass(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);

    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);


#if 1
    material_->getTechnique(0)->setLightingEnabled(false);
#if 1
    const static uint32_t texture_data[4] = { 0x00ffff80, 0x00ffff80, 0x00ffff80, 0x00ffff80 };
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 16 ));
    ss << "Texture";
    texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 2, 2, Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);
#else
    texture_ = Ogre::TextureManager::getSingleton().load("flare.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
#endif


    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );
#else
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, 1.0f));
#endif

    {
      screen_rect_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
      screen_rect_->position(-1.0f, 1.0f, 0.0f);
      screen_rect_->textureCoord(0.0f, 0.0f);

      screen_rect_->position(1.0f, 1.0f, 0.0f);
      screen_rect_->textureCoord(1.0f, 0.0f);

      screen_rect_->position(1.0f, -1.0f, 0.0f);
      screen_rect_->textureCoord(1.0f, 1.0f);

      screen_rect_->position(-1.0f, -1.0f, 0.0f);
      screen_rect_->textureCoord(0.0f, 1.0f);

      screen_rect_->index(0);
      screen_rect_->index(1);
      screen_rect_->index(2);
      screen_rect_->index(0);
      screen_rect_->index(2);
      screen_rect_->index(3);
      screen_rect_->end();
    }
    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    scene_node_->attachObject(screen_rect_);
    screen_rect_->setVisible(false);
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

  image_notifier_ = new tf::MessageNotifier<image_msgs::Image>(tf_, ros_node_, boost::bind(&CameraDisplay::imageCallback, this, _1), "", "", 10);
  caminfo_notifier_ = new tf::MessageNotifier<image_msgs::CamInfo>(tf_, ros_node_, boost::bind(&CameraDisplay::caminfoCallback, this, _1), "", "", 10);
}

CameraDisplay::~CameraDisplay()
{
  unsubscribe();

  delete image_notifier_;
  delete caminfo_notifier_;

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
  scene_manager_->destroyManualObject(screen_rect_);

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

  image_notifier_->setTopic(topic_);

  // parse out the namespace from the topic so we can subscribe to the caminfo
  std::string caminfo_topic = "cam_info";
  size_t pos = topic_.rfind('/');
  if (pos != std::string::npos)
  {
    std::string ns = topic_;
    ns.erase(pos);

    caminfo_topic = ns + "/" + caminfo_topic;
  }

  caminfo_notifier_->setTopic(caminfo_topic);
}

void CameraDisplay::unsubscribe()
{
  image_notifier_->setTopic("");
  caminfo_notifier_->setTopic("");
}

void CameraDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);

  if (pass->getNumTextureUnitStates() > 0)
  {
    Ogre::TextureUnitState* tex_unit = pass->getTextureUnitState(0);
    tex_unit->setAlphaOperation( Ogre::LBX_MODULATE, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_ );
  }
  else
  {
    material_->setAmbient(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
    material_->setDiffuse(Ogre::ColourValue(0.0f, 1.0f, 1.0f, alpha_));
  }

  propertyChanged(alpha_property_);

  causeRender();
}

void CameraDisplay::setTopic( const std::string& topic )
{
  topic_ = topic;
  clear();

  subscribe();

  propertyChanged(topic_property_);
}

void CameraDisplay::clear()
{
  const static uint32_t texture_data[4] = { 0x00ffff80, 0x00ffff80, 0x00ffff80, 0x00ffff80 };
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 16 ));

  texture_->unload();
  texture_->loadRawData(pixel_stream, 2, 2, Ogre::PF_R8G8B8A8);

  new_image_ = false;
  new_caminfo_ = false;
  current_image_.reset();
  current_caminfo_.reset();
}


void CameraDisplay::update( float dt )
{
  if (new_image_)
  {
    new_image_ = false;
    updateImage();
  }
}

void CameraDisplay::updateCamera()
{
  CamInfoConstPtr info;
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
    tf_->transformPose(fixed_frame_, pose, pose);
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

  double fy = info->P[5];
  double fovy = 2*atan(info->height / (2 * fy));
  double aspect_ratio = (float)info->width / (float)info->height;
  camera_->setFOVy(Ogre::Radian(fovy));
  camera_->setAspectRatio(aspect_ratio);

  // Add the camera's translation relative to the left camera (from P[3]);
  // Tx = -1*(P[3] / P[0])
  double fx = info->P[0];
  double tx = -1 * (info->P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  camera_->setPosition(position);
  camera_->setOrientation(orientation);

#if 0
  static ogre_tools::Axes* debug_axes = new ogre_tools::Axes(scene_manager_, 0, 0.2, 0.02);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif
}

void CameraDisplay::updateImage()
{
  ImageConstPtr image;
  {
    boost::mutex::scoped_lock lock(image_mutex_);

    image = current_image_;
  }

  if (!image)
  {
    return;
  }

  if (image->depth != "uint8")
  {
    ROS_ERROR("Unsupported image depth [%s]", image->depth.c_str());
    return;
  }

  if (image->uint8_data.layout.dim.size() != 3)
  {
    ROS_ERROR("Unsupported # of dimensions [%d]", image->uint8_data.layout.dim.size());
    return;
  }

  if (image->uint8_data.layout.dim[0].label != "height"
   || image->uint8_data.layout.dim[1].label != "width"
   || image->uint8_data.layout.dim[2].label != "channel")
  {
    ROS_ERROR("Unsupported image layout.  Currently only 0=height/1=width/2=channel is supported");
    return;
  }

  Ogre::PixelFormat format = Ogre::PF_R8G8B8;

  if (image->encoding == "rgb")
  {
    format = Ogre::PF_R8G8B8;
  }
  else if (image->encoding == "bgr")
  {
    format = Ogre::PF_B8G8R8;
  }
  else if (image->encoding == "rgba")
  {
    format = Ogre::PF_R8G8B8A8;
  }
  else if (image->encoding == "bgra")
  {
    format = Ogre::PF_B8G8R8A8;
  }
  else if (image->encoding == "mono")
  {
    format = Ogre::PF_L8;
  }
  else
  {
    ROS_ERROR("Unsupported image encoding [%s]", image->encoding.c_str());
    return;
  }

  uint32_t size = image->uint8_data.layout.dim[0].stride;
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream((void*)(&image->uint8_data.data[0] + image->uint8_data.layout.data_offset), size));
  texture_->unload();
  texture_->loadRawData(pixel_stream, image->uint8_data.layout.dim[1].size, image->uint8_data.layout.dim[0].size, format);
}

void CameraDisplay::imageCallback(const ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(image_mutex_);
  current_image_ = msg;
  new_image_ = true;
}

void CameraDisplay::caminfoCallback(const CamInfoConstPtr& msg)
{
  boost::mutex::scoped_lock lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Image Topic", property_prefix_, boost::bind( &CameraDisplay::getTopic, this ),
                                                                         boost::bind( &CameraDisplay::setTopic, this, _1 ), category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(image_msgs::Image::__s_getDataType());

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &CameraDisplay::getAlpha, this ),
                                                                      boost::bind( &CameraDisplay::setAlpha, this, _1 ), category_, this );

}

void CameraDisplay::fixedFrameChanged()
{
  image_notifier_->setTargetFrame(fixed_frame_);
  caminfo_notifier_->setTargetFrame(fixed_frame_);
}

void CameraDisplay::targetFrameChanged()
{

}

void CameraDisplay::reset()
{
  clear();
}

const char* CameraDisplay::getDescription()
{
  return "Displays an alphad-out image from a camera, with the visualized world rendered behind it.";
}

} // namespace rviz
