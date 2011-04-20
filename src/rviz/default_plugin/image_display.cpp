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

#include "image_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/window_manager_interface.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

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

ImageDisplay::ImageDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, transport_("raw")
, texture_(update_nh_)
, frame_(0)
{
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "ImageDisplay" << count++;
    scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "ImageDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering( Ogre::TFO_NONE );

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    screen_rect_->setMaterial(material_->getName());
    scene_node_->attachObject(screen_rect_);

  }

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

  render_panel_->initialize(scene_manager_, vis_manager_);

  render_panel_->setAutoRender(false);
  render_panel_->getViewport()->setOverlaysEnabled(false);
  render_panel_->getViewport()->setClearEveryFrame(true);
  render_panel_->getRenderWindow()->setActive(false);
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );
}

ImageDisplay::~ImageDisplay()
{
  unsubscribe();

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

  delete screen_rect_;

  scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
}

void ImageDisplay::onEnable()
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

void ImageDisplay::onDisable()
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

void ImageDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  texture_.setTopic(topic_);
}

void ImageDisplay::unsubscribe()
{
  texture_.setTopic("");
}

void ImageDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;
  clear();

  subscribe();

  propertyChanged(topic_property_);
}

void ImageDisplay::setTransport(const std::string& transport)
{
  transport_ = transport;

  texture_.setTransportType(transport);

  propertyChanged(transport_property_);
}

void ImageDisplay::clear()
{
  texture_.clear();

  setStatus(status_levels::Warn, "Image", "No Image received");

  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void ImageDisplay::updateStatus()
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

void ImageDisplay::update(float wall_dt, float ros_dt)
{
  updateStatus();

  try
  {
    texture_.update();
    render_panel_->getRenderWindow()->update();
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(status_levels::Error, "Image", e.what());
  }
}

void ImageDisplay::onTransportEnumOptions(V_string& choices)
{
  texture_.getAvailableTransportTypes(choices);
}

void ImageDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Image Topic", property_prefix_, boost::bind( &ImageDisplay::getTopic, this ),
                                                                         boost::bind( &ImageDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "sensor_msgs::Image topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

  transport_property_ = property_manager_->createProperty<EditEnumProperty>("Transport Hint", property_prefix_, boost::bind(&ImageDisplay::getTransport, this),
                                                                            boost::bind(&ImageDisplay::setTransport, this, _1), parent_category_, this);
  EditEnumPropertyPtr ee_prop = transport_property_.lock();
  ee_prop->setOptionCallback(boost::bind(&ImageDisplay::onTransportEnumOptions, this, _1));
}

void ImageDisplay::fixedFrameChanged()
{
}

void ImageDisplay::targetFrameChanged()
{

}

void ImageDisplay::reset()
{
  Display::reset();

  clear();
}

} // namespace rviz
