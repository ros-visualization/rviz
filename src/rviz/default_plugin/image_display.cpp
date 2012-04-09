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
#include "rviz/window_manager_interface.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/display_wrapper.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

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

ImageDisplay::ImageDisplay()
  : Display()
  , transport_("raw")
  , texture_(update_nh_)
  , panel_container_( 0 )
{
}

void ImageDisplay::onInitialize()
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

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive( false );

  render_panel_->resize( 640, 480 );
  render_panel_->initialize(scene_manager_, vis_manager_);

  WindowManagerInterface* wm = vis_manager_->getWindowManager();
  if (wm)
  {
    panel_container_ = wm->addPane(name_, render_panel_);
  }
  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance( 0.01f );

  if( panel_container_ )
  {
    connect( panel_container_, SIGNAL( visibilityChanged( bool ) ), this, SLOT( setWrapperEnabled( bool )));
  }
}

ImageDisplay::~ImageDisplay()
{
  unsubscribe();

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

  delete screen_rect_;

  scene_node_->getParentSceneNode()->removeAndDestroyChild(scene_node_->getName());
}

void ImageDisplay::setWrapperEnabled( bool enabled )
{
  // Have to use the DisplayWrapper disable function so the checkbox
  // gets checked or unchecked, since it owns the "enabled" property.
  DisplayWrapper* wrapper = vis_manager_->getDisplayWrapper( this );
  if( wrapper != NULL )
  {
    wrapper->setEnabled( enabled );
  }
}

void ImageDisplay::onEnable()
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

void ImageDisplay::onDisable()
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
      panel_container_->close();
    }
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

  try
  {
    texture_.setTopic(topic_);
    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void ImageDisplay::unsubscribe()
{
  texture_.setTopic("");
}

void ImageDisplay::setQueueSize( int size )
{
  if( size != texture_.getQueueSize() )
  {
    texture_.setQueueSize( size );
    propertyChanged( queue_size_property_ );
  }
}

int ImageDisplay::getQueueSize()
{
  return texture_.getQueueSize();
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

  if( render_panel_->getCamera() )
  {
    render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
  }
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

    //make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if ( img_width != 0 && img_height != 0 && win_width !=0 && win_height != 0 )
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if ( img_aspect > win_aspect )
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect/img_aspect, 1.0f, -1.0f * win_aspect/img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect/win_aspect, 1.0f, 1.0f * img_aspect/win_aspect, -1.0f, false);
      }
    }

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

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &ImageDisplay::getQueueSize, this ),
                                                                         boost::bind( &ImageDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming Image message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your Image data, but it can greatly increase memory usage if the messages are big." );
}

void ImageDisplay::reset()
{
  Display::reset();

  clear();
}

} // namespace rviz
