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

#include "map_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <rviz/ogre_helpers/grid.h>

#include <ros/ros.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz
{

MapDisplay::MapDisplay()
  : Display()
  , scene_node_( 0 )
  , manual_object_( NULL )
  , material_( 0 )
  , loaded_( false )
  , resolution_( 0.0f )
  , width_( 0 )
  , height_( 0 )
  , position_(Ogre::Vector3::ZERO)
  , orientation_(Ogre::Quaternion::IDENTITY)
  , draw_under_(false)
  , hidden_(false)
{
}

MapDisplay::~MapDisplay()
{
  unsubscribe();

  clear();
}

void MapDisplay::onInitialize()
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "MapObjectMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);

  setAlpha( 0.7f );
}

void MapDisplay::onEnable()
{
  subscribe();
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MapDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible( enabled_ && !hidden_ );
  clear();
}

void MapDisplay::hideVisible()
{
  hidden_ = true;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MapDisplay::restoreVisible()
{
  hidden_ = false;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if (!topic_.empty())
  {
    try
    {
      map_sub_ = update_nh_.subscribe(topic_, 1, &MapDisplay::incomingMap, this);
      setStatus(status_levels::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
    }
  }
}

void MapDisplay::unsubscribe()
{
  map_sub_.shutdown();
}

void MapDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, alpha_ );

  if ( alpha_ < 0.9998 )
  {
    material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->setDepthWriteEnabled(false);
  }
  else
  {
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled(!draw_under_);
  }

  propertyChanged(alpha_property_);
}

void MapDisplay::setDrawUnder(bool under)
{
  draw_under_ = under;
  if (alpha_ >= 0.9998)
  {
    material_->setDepthWriteEnabled(!draw_under_);
  }

  if (manual_object_)
  {
    if (draw_under_)
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
    }
    else
    {
      manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
    }
  }

  propertyChanged(draw_under_property_);
}

void MapDisplay::setTopic(const std::string& topic)
{
  unsubscribe();
  // something of a hack.  try to provide backwards compatibility with the service-version
  if (topic == "static_map" || topic == "dynamic_map")
  {
    topic_ = "map";
  }
  else
  {
    topic_ = topic;
  }
  subscribe();

  clear();

  propertyChanged(topic_property_);
}

void MapDisplay::clear()
{
  setStatus(status_levels::Warn, "Message", "No map received");

  if ( !loaded_ )
  {
    return;
  }

  scene_manager_->destroyManualObject( manual_object_ );
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload( tex_name );

  loaded_ = false;
}

bool validateFloats(const nav_msgs::OccupancyGrid& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.info.resolution);
  valid = valid && validateFloats(msg.info.origin);
  return valid;
}

void MapDisplay::load(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  if (!validateFloats(*msg))
  {
    setStatus(status_levels::Error, "Map", "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (msg->info.width * msg->info.height == 0)
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << msg->info.width << "x" << msg->info.height << ")";
    setStatus(status_levels::Error, "Map", ss.str());
    return;
  }

  clear();

  setStatus(status_levels::Ok, "Message", "Map received");

  ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
             msg->info.width,
             msg->info.height,
             msg->info.resolution);

  resolution_ = msg->info.resolution;

  // Pad dimensions to power of 2
  width_ = msg->info.width;//(int)pow(2,ceil(log2(msg->info.width)));
  height_ = msg->info.height;//(int)pow(2,ceil(log2(msg->info.height)));

  //printf("Padded dimensions to %d X %d\n", width_, height_);

  map_ = msg;
  position_.x = msg->info.origin.position.x;
  position_.y = msg->info.origin.position.y;
  position_.z = msg->info.origin.position.z;
  orientation_.w = msg->info.origin.orientation.w;
  orientation_.x = msg->info.origin.orientation.x;
  orientation_.y = msg->info.origin.orientation.y;
  orientation_.z = msg->info.origin.orientation.z;
  frame_ = msg->header.frame_id;
  if (frame_.empty())
  {
    frame_ = "/map";
  }

  // Expand it to be RGB data
  unsigned int pixels_size = width_ * height_;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  bool map_status_set = false;
  unsigned int num_pixels_to_copy = pixels_size;
  if( pixels_size != msg->data.size() )
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width_
       << ", height = " << height_ << ", data size = " << msg->data.size();
    setStatus(status_levels::Error, "Map", ss.str());
    map_status_set = true;

    // Keep going, but don't read past the end of the data.
    if( msg->data.size() < pixels_size )
    {
      num_pixels_to_copy = msg->data.size();
    }
  }

  for( unsigned int pixel_index = 0; pixel_index < num_pixels_to_copy; pixel_index++ )
  {
    unsigned char val;
    if(msg->data[ pixel_index ] == 100)
      val = 0;
    else if(msg->data[ pixel_index ] == 0)
      val = 255;
    else
      val = 127;

    pixels[ pixel_index ] = val;
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( pixels, pixels_size ));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  try
  {
    texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
								 pixel_stream, width_, height_, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
								 0);

    if( !map_status_set )
    {
      setStatus(status_levels::Ok, "Map", "Map OK");
    }
  }
  catch(Ogre::RenderingAPIException&)
  {
    Ogre::Image image;
    pixel_stream->seek(0);
    float width = width_;
    float height = height_;
    if (width_ > height_)
    {
      float aspect = height / width;
      width = 2048;
      height = width * aspect;
    }
    else
    {
      float aspect = width / height;
      height = 2048;
      width = height * aspect;
    }

    {
      std::stringstream ss;
      ss << "Map is larger than your graphics card supports.  Downsampled from [" << width_ << "x" << height_ << "] to [" << width << "x" << height << "]";
      setStatus(status_levels::Ok, "Map", ss.str());
    }

    ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)width, (int)height);
    //ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width_, height_, width_ * height_);
    image.loadRawData(pixel_stream, width_, height_, Ogre::PF_L8);
    image.resize(width, height, Ogre::Image::FILTER_NEAREST);
    ss << "Downsampled";
    texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
  }

  delete [] pixels;

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "MapObject" << map_count++;
  manual_object_ = scene_manager_->createManualObject( ss2.str() );
  scene_node_->attachObject( manual_object_ );

  manual_object_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( resolution_*width_, resolution_*height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top left
      manual_object_->position( 0.0f, resolution_*height_, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Bottom left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( resolution_*width_, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( resolution_*width_, resolution_*height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  if (draw_under_)
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  propertyChanged(resolution_property_);
  propertyChanged(width_property_);
  propertyChanged(width_property_);
  propertyChanged(position_property_);
  propertyChanged(orientation_property_);

  transformMap();

  loaded_ = true;

  causeRender();
}

void MapDisplay::transformMap()
{
  if (!map_)
  {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!vis_manager_->getFrameManager()->transform(frame_, ros::Time(), map_->info.origin, position, orientation))
  {
    ROS_DEBUG("Error transforming map '%s' from frame '%s' to frame '%s'", name_.c_str(), frame_.c_str(), fixed_frame_.c_str());

    std::stringstream ss;
    ss << "No transform from [" << frame_ << "] to [" << fixed_frame_ << "]";
    setStatus(status_levels::Error, "Transform", ss.str());
  }
  else
  {
    setStatus(status_levels::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
}

void MapDisplay::update(float wall_dt, float ros_dt)
{
}

void MapDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &MapDisplay::getTopic, this ),
                                                                         boost::bind( &MapDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "nav_msgs::OccupancyGrid topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<nav_msgs::OccupancyGrid>());
  topic_prop->addLegacyName("Service"); // something of a hack, but should provide reasonable backwards compatibility

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &MapDisplay::getAlpha, this ),
                                                                      boost::bind( &MapDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the map.");
  draw_under_property_ = property_manager_->createProperty<BoolProperty>( "Draw Behind", property_prefix_, boost::bind( &MapDisplay::getDrawUnder, this ),
                                                                        boost::bind( &MapDisplay::setDrawUnder, this, _1 ), parent_category_, this );
  setPropertyHelpText(draw_under_property_, "Rendering option, controls whether or not the map is always drawn behind everything else.");

  resolution_property_ = property_manager_->createProperty<FloatProperty>( "Resolution", property_prefix_, boost::bind( &MapDisplay::getResolution, this ),
                                                                            FloatProperty::Setter(), parent_category_, this );
  setPropertyHelpText(resolution_property_, "Resolution of the map. (not editable)");
  width_property_ = property_manager_->createProperty<IntProperty>( "Width", property_prefix_, boost::bind( &MapDisplay::getWidth, this ),
                                                                    IntProperty::Setter(), parent_category_, this );
  setPropertyHelpText(width_property_, "Width of the map, in meters. (not editable)");
  height_property_ = property_manager_->createProperty<IntProperty>( "Height", property_prefix_, boost::bind( &MapDisplay::getHeight, this ),
                                                                     IntProperty::Setter(), parent_category_, this );
  setPropertyHelpText(height_property_, "Height of the map, in meters. (not editable)");
  position_property_ = property_manager_->createProperty<Vector3Property>( "Position", property_prefix_, boost::bind( &MapDisplay::getPosition, this ),
                                                                           Vector3Property::Setter(), parent_category_, this );
  setPropertyHelpText(position_property_, "Position of the bottom left corner of the map, in meters. (not editable)");
  orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", property_prefix_, boost::bind( &MapDisplay::getOrientation, this ),
                                                                                 QuaternionProperty::Setter(), parent_category_, this );
  setPropertyHelpText(orientation_property_, "Orientation of the map. (not editable)");
}

void MapDisplay::fixedFrameChanged()
{
  transformMap();
}

void MapDisplay::reset()
{
  Display::reset();

  clear();
  // Force resubscription so that the map will be re-sent
  setTopic(topic_);
}

void MapDisplay::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  load(msg);
}

} // namespace rviz
