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
#include "rviz/common.h"

#include <tf/transform_listener.h>

#include <ogre_tools/grid.h>

#include <ros/ros.h>

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
, map_request_time_(0.0f)
, map_request_timer_(0.0f)
, last_loaded_map_time_( ros::Time() )
, new_map_(false)
, reload_(false)
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

  request_thread_ = boost::thread(&MapDisplay::requestThreadFunc, this);
}

void MapDisplay::onDisable()
{
  unsubscribe();

  scene_node_->setVisible( false );
  clear();

  request_thread_.join();
}

void MapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  metadata_sub_ = update_nh_.subscribe("map_metadata", 1, &MapDisplay::incomingMetaData, this);
}

void MapDisplay::unsubscribe()
{
  metadata_sub_.shutdown();
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

  loaded_ = false;
}

void MapDisplay::requestThreadFunc()
{
  while (isEnabled())
  {
    if (!new_map_ && (!loaded_ || reload_))
    {
      nav_msgs::GetMap srv;
      ROS_DEBUG("Requesting the map...");
      ros::ServiceClient client = update_nh_.serviceClient<nav_msgs::GetMap>(service_);
      if(client.call(srv) )
      {
        {
          boost::mutex::scoped_lock lock(map_mutex_);
          map_srv_ = srv;
          new_map_ = true;
        }

        reload_ = false;
      }
      else
      {
        ROS_DEBUG("request failed");
      }
    }

    ros::WallDuration(0.1).sleep();
  }
}

void MapDisplay::load()
{
  boost::mutex::scoped_lock lock(map_mutex_);

  if (map_srv_.response.map.info.width * map_srv_.response.map.info.height == 0)
  {
    return;
  }

  clear();

  ROS_DEBUG("Received a %d X %d map @ %.3f m/pix\n",
             map_srv_.response.map.info.width,
             map_srv_.response.map.info.height,
             map_srv_.response.map.info.resolution);

  resolution_ = map_srv_.response.map.info.resolution;

  // Pad dimensions to power of 2
  width_ = map_srv_.response.map.info.width;//(int)pow(2,ceil(log2(map_srv_.response.map.info.width)));
  height_ = map_srv_.response.map.info.height;//(int)pow(2,ceil(log2(map_srv_.response.map.info.height)));

  //printf("Padded dimensions to %d X %d\n", width_, height_);

  position_.x = map_srv_.response.map.info.origin.position.x;
  position_.y = map_srv_.response.map.info.origin.position.y;
  position_.z = map_srv_.response.map.info.origin.position.z;
  orientation_.w = map_srv_.response.map.info.origin.orientation.w;
  orientation_.x = map_srv_.response.map.info.origin.orientation.x;
  orientation_.y = map_srv_.response.map.info.origin.orientation.y;
  orientation_.z = map_srv_.response.map.info.origin.orientation.z;

  // Expand it to be RGB data
  int pixels_size = width_ * height_;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  for(unsigned int j=0;j<map_srv_.response.map.info.height;j++)
  {
    for(unsigned int i=0;i<map_srv_.response.map.info.width;i++)
    {
      unsigned char val;
      if(map_srv_.response.map.data[j*map_srv_.response.map.info.width+i] == 100)
        val = 0;
      else if(map_srv_.response.map.data[j*map_srv_.response.map.info.width+i] == 0)
        val = 255;
      else
        val = 127;

      int pidx = (j*width_ + i);
      pixels[pidx] = val;
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( pixels, pixels_size ));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                   pixel_stream, width_, height_, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
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
  propertyChanged(position_property_);
  propertyChanged(orientation_property_);

  transformMap();

  loaded_ = true;

  causeRender();
}

void MapDisplay::transformMap()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( orientation_.x, orientation_.y, orientation_.z, orientation_.w ),
                                           btVector3(position_.x, position_.y, position_.z) ), ros::Time(), "/map" );

  if ( vis_manager_->getTFClient()->canTransform(fixed_frame_, "/map", ros::Time()))
  {
    try
    {
      vis_manager_->getTFClient()->transformPose( fixed_frame_, pose, pose );
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

void MapDisplay::update(float wall_dt, float ros_dt)
{
  if (map_request_time_ > 0.01f && !reload_)
  {
    map_request_timer_ += wall_dt;
    if (map_request_timer_ >= map_request_time_)
    {
      reload_ = true;

      map_request_timer_ = 0.0f;
    }
  }

  if (new_map_)
  {
    load();
    new_map_ = false;
  }
}

void MapDisplay::createProperties()
{
  service_property_ = property_manager_->createProperty<StringProperty>( "Service", property_prefix_, boost::bind( &MapDisplay::getService, this ),
                                                                         boost::bind( &MapDisplay::setService, this, _1 ), parent_category_, this );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &MapDisplay::getAlpha, this ),
                                                                      boost::bind( &MapDisplay::setAlpha, this, _1 ), parent_category_, this );

  map_request_time_property_ = property_manager_->createProperty<FloatProperty>( "Request Frequency", property_prefix_, boost::bind( &MapDisplay::getMapRequestTime, this ),
                                                                      boost::bind( &MapDisplay::setMapRequestTime, this, _1 ), parent_category_, this );

  resolution_property_ = property_manager_->createProperty<FloatProperty>( "Resolution", property_prefix_, boost::bind( &MapDisplay::getResolution, this ),
                                                                            FloatProperty::Setter(), parent_category_, this );
  width_property_ = property_manager_->createProperty<FloatProperty>( "Width", property_prefix_, boost::bind( &MapDisplay::getWidth, this ),
                                                                       FloatProperty::Setter(), parent_category_, this );
  height_property_ = property_manager_->createProperty<FloatProperty>( "Height", property_prefix_, boost::bind( &MapDisplay::getHeight, this ),
                                                                        FloatProperty::Setter(), parent_category_, this );
  position_property_ = property_manager_->createProperty<Vector3Property>( "Position", property_prefix_, boost::bind( &MapDisplay::getPosition, this ),
                                                                           Vector3Property::Setter(), parent_category_, this );
  orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", property_prefix_, boost::bind( &MapDisplay::getOrientation, this ),
                                                                                 QuaternionProperty::Setter(), parent_category_, this );
}

void MapDisplay::fixedFrameChanged()
{
  transformMap();
}

void MapDisplay::reset()
{
  clear();
}

void MapDisplay::incomingMetaData(const nav_msgs::MapMetaData::ConstPtr& msg)
{
  if ( !(msg->map_load_time == last_loaded_map_time_) )
  {
    last_loaded_map_time_ = msg->map_load_time;

    clear();
  }
}

} // namespace rviz
