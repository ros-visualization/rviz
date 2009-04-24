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
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include <tf/transform_listener.h>
#include <robot_srvs/StaticMap.h>

#include <ogre_tools/grid.h>

#include <ros/service.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz
{

MapDisplay::MapDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, manual_object_( NULL )
, loaded_( false )
, service_( "static_map" )
, resolution_( 0.0f )
, width_( 0.0f )
, height_( 0.0f )
, load_timer_( 2.0f )
, map_request_time_(0.0f)
, map_request_timer_(0.0f)
, new_metadata_( false )
, last_loaded_map_time_( ros::Time() )
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

  setAlpha( 0.7f );
}

MapDisplay::~MapDisplay()
{
  unsubscribe();

  clear();
}

void MapDisplay::onEnable()
{
  subscribe();

  scene_node_->setVisible( true );
}

void MapDisplay::onDisable()
{
  unsubscribe();

  scene_node_->setVisible( false );
  clear();
}

void MapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  ros_node_->subscribe( "map_metadata", metadata_message_, &MapDisplay::incomingMetaData, this, 1 );
}

void MapDisplay::unsubscribe()
{
  ros_node_->unsubscribe( "map_metadata", &MapDisplay::incomingMetaData, this );
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
    material_->setDepthWriteEnabled(true);
  }

  propertyChanged(alpha_property_);
}

void MapDisplay::setMapRequestTime(float time)
{
  map_request_time_ = time;

  propertyChanged(map_request_time_property_);
}

void MapDisplay::setService( const std::string& service )
{
  service_ = service;
  clear();

  propertyChanged(service_property_);
}

void MapDisplay::clear()
{
  if ( !loaded_ )
  {
    return;
  }

  scene_manager_->destroyManualObject( manual_object_ );
  manual_object_ = NULL;

  std::string tex_name = texture_->getName();
  texture_.setNull();
  Ogre::TextureManager::getSingleton().unload( tex_name );

  load_timer_ = 2.0f;
  loaded_ = false;
}

void MapDisplay::load()
{
  robot_srvs::StaticMap::Request  req;
  robot_srvs::StaticMap::Response resp;
  ROS_DEBUG("Requesting the map...");
  if( !ros::service::call(service_, req, resp) )
  {
    ROS_DEBUG("request failed");

    return;
  }

  clear();

  ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
             resp.map.width,
             resp.map.height,
             resp.map.resolution);

  resolution_ = resp.map.resolution;

  // Pad dimensions to power of 2
  width_ = resp.map.width;//(int)pow(2,ceil(log2(resp.map.width)));
  height_ = resp.map.height;//(int)pow(2,ceil(log2(resp.map.height)));

  //printf("Padded dimensions to %d X %d\n", width_, height_);

  // Expand it to be RGB data
  int pixels_size = width_ * height_ * 3;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  for(unsigned int j=0;j<resp.map.height;j++)
  {
    for(unsigned int i=0;i<resp.map.width;i++)
    {
      unsigned char val;
      if(resp.map.data[j*resp.map.width+i] == 100)
        val = 0;
      else if(resp.map.data[j*resp.map.width+i] == 0)
        val = 255;
      else
        val = 127;

      int pidx = 3*(j*width_ + i);
      pixels[pidx+0] = val;
      pixels[pidx+1] = val;
      pixels[pidx+2] = val;
      //pixels[pidx+3] = 1.0f;stamp
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( pixels, pixels_size ));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "NavViewMapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                   pixel_stream, width_, height_, Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D,
                                                                   0);

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
      // Top left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( resolution_*width_, resolution_*height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom left
      manual_object_->position( 0.0f, resolution_*height_, 0.0f );
      manual_object_->textureCoord(0.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Top left
      manual_object_->position( 0.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(0.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( resolution_*width_, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      manual_object_->position( resolution_*width_, resolution_*height_, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  propertyChanged(resolution_property_);
  propertyChanged(width_property_);
  propertyChanged(width_property_);

  transformMap();

  loaded_ = true;

  causeRender();
}

void MapDisplay::transformMap()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), "map" );

  if ( tf_->canTransform(fixed_frame_, "map", ros::Time()))
  {
    try
    {
      tf_->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming map '%s' to frame '%s'\n", name_.c_str(), fixed_frame_.c_str() );
    }
  }

  Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );

  Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( yaw, pitch, roll ) );
  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
}

void MapDisplay::update( float dt )
{
  if ( new_metadata_ )
  {
    /// @todo implement ros::Time::operator!=
    if ( !(metadata_message_.map_load_time == last_loaded_map_time_) )
    {
      last_loaded_map_time_ = metadata_message_.map_load_time;

      clear();
    }
  }

  if (map_request_time_ > 0.01f)
  {
    map_request_timer_ += dt;
    if (map_request_timer_ >= map_request_time_)
    {
      load();

      map_request_timer_ = 0.0f;
    }
  }

  if ( !loaded_ )
  {
    load_timer_ += dt;

    if ( load_timer_ > 2.0f )
    {
      load();
      load_timer_ = 0.0f;
    }
  }
}

void MapDisplay::createProperties()
{
  service_property_ = property_manager_->createProperty<StringProperty>( "Service", property_prefix_, boost::bind( &MapDisplay::getService, this ),
                                                                         boost::bind( &MapDisplay::setService, this, _1 ), category_, this );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &MapDisplay::getAlpha, this ),
                                                                      boost::bind( &MapDisplay::setAlpha, this, _1 ), category_, this );

  map_request_time_property_ = property_manager_->createProperty<FloatProperty>( "Request Frequency", property_prefix_, boost::bind( &MapDisplay::getMapRequestTime, this ),
                                                                      boost::bind( &MapDisplay::setMapRequestTime, this, _1 ), category_, this );

  resolution_property_ = property_manager_->createProperty<FloatProperty>( "Resolution", property_prefix_, boost::bind( &MapDisplay::getResolution, this ),
                                                                            FloatProperty::Setter(), category_, this );
  width_property_ = property_manager_->createProperty<FloatProperty>( "Width", property_prefix_, boost::bind( &MapDisplay::getWidth, this ),
                                                                       FloatProperty::Setter(), category_, this );
  height_property_ = property_manager_->createProperty<FloatProperty>( "Height", property_prefix_, boost::bind( &MapDisplay::getHeight, this ),
                                                                        FloatProperty::Setter(), category_, this );
}

void MapDisplay::fixedFrameChanged()
{
  transformMap();
}

void MapDisplay::reset()
{
  clear();
}

void MapDisplay::incomingMetaData()
{
  new_metadata_ = true;
}

const char* MapDisplay::getDescription()
{
  return "Displays an image of a map gotten through a robot_srvs::StaticMap service.";
}

} // namespace rviz
