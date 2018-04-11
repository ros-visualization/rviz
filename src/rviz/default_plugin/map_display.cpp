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

#include <boost/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/custom_parameter_indices.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/validate_quaternions.h"
#include "rviz/display_context.h"

#include "map_display.h"

namespace rviz
{

// helper class to set alpha parameter on all renderables.
class AlphaSetter: public Ogre::Renderable::Visitor
{
public:
  AlphaSetter( float alpha )
  : alpha_vec_( alpha, alpha, alpha, alpha )
  {}

  void visit( Ogre::Renderable *rend, ushort lodIndex, bool isDebug, Ogre::Any *pAny=0)
  {
    rend->setCustomParameter( ALPHA_PARAMETER, alpha_vec_ );
  }
private:
  Ogre::Vector4 alpha_vec_;
};


Swatch::Swatch(MapDisplay* parent, unsigned int x, unsigned int y, unsigned int width, unsigned int height, float resolution)
  : parent_(parent), manual_object_( NULL ), x_(x), y_(y), width_(width), height_(height)
{
  // Set up map material
  static int material_count = 0;
  std::stringstream ss;
  ss << "MapMaterial" << material_count++;
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone( ss.str() );

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias( -16.0f, 0.0f );
  material_->setCullingMode( Ogre::CULL_NONE );
  material_->setDepthWriteEnabled(false);

  static int map_count = 0;
  std::stringstream ss2;
  ss2 << "MapObject" << map_count++;
  manual_object_ = parent_->scene_manager_->createManualObject( ss2.str() );

  static int node_count = 0;
  std::stringstream ss3;
  ss3 << "NodeObject" << node_count++;

  scene_node_ = parent_->scene_node_->createChildSceneNode(ss3.str());
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
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top left
      manual_object_->position( 0.0f, 1.0f, 0.0f );
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
      manual_object_->position( 1.0f, 0.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 0.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      manual_object_->position( 1.0f, 1.0f, 0.0f );
      manual_object_->textureCoord(1.0f, 1.0f);
      manual_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  manual_object_->end();

  scene_node_->setPosition( x*resolution, y*resolution, 0 );
  scene_node_->setScale( width*resolution, height*resolution, 1.0 );

  if( parent_->draw_under_property_->getValue().toBool() )
  {
    manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
  }

  // don't show map until the plugin is actually enabled
  manual_object_->setVisible( false );


}

Swatch::~Swatch()
{
  parent_->scene_manager_->destroyManualObject(manual_object_);
}

void Swatch::updateAlpha(const Ogre::SceneBlendType sceneBlending, bool depthWrite, AlphaSetter* alpha_setter)
{
  Ogre::Pass* pass = material_->getTechnique( 0 )->getPass( 0 );
  Ogre::TextureUnitState* tex_unit = NULL;

  material_->setSceneBlending( sceneBlending );
  material_->setDepthWriteEnabled( depthWrite );
  if( manual_object_ )
  {
    manual_object_->visitRenderables( alpha_setter );
  }
}

void Swatch::updateData()
{
  unsigned int pixels_size = width_ * height_;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);
  unsigned char* ptr = pixels;
  int N = parent_->current_map_.data.size();
  unsigned int fw = parent_->current_map_.info.width;

  for(unsigned int yy=y_; yy<y_+height_;yy++){
    int index = yy * fw + x_;
    int pixels_to_copy = std::min((int)width_, N-index);
    memcpy(ptr, &parent_->current_map_.data[ index ], pixels_to_copy);
    ptr+=pixels_to_copy;
    if(index+pixels_to_copy>=N) break;
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind( new Ogre::MemoryDataStream( pixels, pixels_size ));

  if( !texture_.isNull() )
  {
    Ogre::TextureManager::getSingleton().remove( texture_->getName() );
    texture_.setNull();
  }

  static int tex_count = 0;
  std::stringstream ss;
  ss << "MapTexture" << tex_count++;
  texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                 pixel_stream, width_, height_, Ogre::PF_L8, Ogre::TEX_TYPE_2D,
                                                                 0);

  delete[] pixels;
}


MapDisplay::MapDisplay()
  : Display()
  , loaded_( false )
  , resolution_( 0.0f )
  , width_( 0 )
  , height_( 0 )
{
  connect(this, SIGNAL( mapUpdated() ), this, SLOT( showMap() ));
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::OccupancyGrid>() ),
                                          "nav_msgs::OccupancyGrid topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  alpha_property_ = new FloatProperty( "Alpha", 0.7,
                                       "Amount of transparency to apply to the map.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  color_scheme_property_ = new EnumProperty( "Color Scheme", "map", "How to color the occupancy values.",
                                             this, SLOT( updatePalette() ));
  // Option values here must correspond to indices in palette_textures_ array in onInitialize() below.
  color_scheme_property_->addOption( "map", 0 );
  color_scheme_property_->addOption( "costmap", 1 );
  color_scheme_property_->addOption( "raw", 2 );

  draw_under_property_ = new Property( "Draw Behind", false,
                                       "Rendering option, controls whether or not the map is always"
                                       " drawn behind everything else.",
                                       this, SLOT( updateDrawUnder() ));

  resolution_property_ = new FloatProperty( "Resolution", 0,
                                            "Resolution of the map. (not editable)", this );
  resolution_property_->setReadOnly( true );

  width_property_ = new IntProperty( "Width", 0,
                                     "Width of the map, in meters. (not editable)", this );
  width_property_->setReadOnly( true );

  height_property_ = new IntProperty( "Height", 0,
                                      "Height of the map, in meters. (not editable)", this );
  height_property_->setReadOnly( true );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of the bottom left corner of the map, in meters. (not editable)",
                                           this );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of the map. (not editable)",
                                                  this );
  orientation_property_->setReadOnly( true );

  unreliable_property_ = new BoolProperty( "Unreliable", false,
                                           "Prefer UDP topic transport",
                                           this,
                                           SLOT( updateTopic() ));

  transform_timestamp_property_ = new BoolProperty( "Use Timestamp", false,
                                                    "Use map header timestamp when transforming",
                                                    this,
                                                    SLOT( transformMap() ));
}

MapDisplay::~MapDisplay()
{
  unsubscribe();
  clear();
  for (unsigned i=0; i < swatches.size(); i++){
      delete swatches[i];
  }
  swatches.clear();
}

unsigned char* makeMapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  // Standard gray map palette values
  for( int i = 0; i <= 100; i++ )
  {
    unsigned char v = 255 - (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = v; // green
    *palette_ptr++ = v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeCostmapPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;

  // zero values have alpha=0
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 0; // blue
  *palette_ptr++ = 0; // alpha

  // Blue to red spectrum for most normal cost values
  for( int i = 1; i <= 98; i++ )
  {
    unsigned char v = (255 * i) / 100;
    *palette_ptr++ = v; // red
    *palette_ptr++ = 0; // green
    *palette_ptr++ = 255 - v; // blue
    *palette_ptr++ = 255; // alpha
  }
  // inscribed obstacle values (99) in cyan
  *palette_ptr++ = 0; // red
  *palette_ptr++ = 255; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha
  // lethal obstacle values (100) in purple
  *palette_ptr++ = 255; // red
  *palette_ptr++ = 0; // green
  *palette_ptr++ = 255; // blue
  *palette_ptr++ = 255; // alpha
  // illegal positive values in green
  for( int i = 101; i <= 127; i++ )
  {
    *palette_ptr++ = 0; // red
    *palette_ptr++ = 255; // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // illegal negative (char) values in shades of red/yellow
  for( int i = 128; i <= 254; i++ )
  {
    *palette_ptr++ = 255; // red
    *palette_ptr++ = (255*(i-128))/(254-128); // green
    *palette_ptr++ = 0; // blue
    *palette_ptr++ = 255; // alpha
  }
  // legal -1 value is tasteful blueish greenish grayish color
  *palette_ptr++ = 0x70; // red
  *palette_ptr++ = 0x89; // green
  *palette_ptr++ = 0x86; // blue
  *palette_ptr++ = 255; // alpha

  return palette;
}

unsigned char* makeRawPalette()
{
  unsigned char* palette = new unsigned char[256*4];
  unsigned char* palette_ptr = palette;
  // Standard gray map palette values
  for( int i = 0; i < 256; i++ )
  {
    *palette_ptr++ = i; // red
    *palette_ptr++ = i; // green
    *palette_ptr++ = i; // blue
    *palette_ptr++ = 255; // alpha
  }

  return palette;
}

Ogre::TexturePtr makePaletteTexture( unsigned char *palette_bytes )
{
  Ogre::DataStreamPtr palette_stream;
  palette_stream.bind( new Ogre::MemoryDataStream( palette_bytes, 256*4 ));

  static int palette_tex_count = 0;
  std::stringstream ss;
  ss << "MapPaletteTexture" << palette_tex_count++;
  return Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                           palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0 );
}

void MapDisplay::onInitialize()
{
  // Order of palette textures here must match option indices for color_scheme_property_ above.
  palette_textures_.push_back( makePaletteTexture( makeMapPalette() ));
  color_scheme_transparency_.push_back( false );
  palette_textures_.push_back( makePaletteTexture( makeCostmapPalette() ));
  color_scheme_transparency_.push_back( true );
  palette_textures_.push_back( makePaletteTexture( makeRawPalette() ));
  color_scheme_transparency_.push_back( true );
}

void MapDisplay::onEnable()
{
  subscribe();
}

void MapDisplay::onDisable()
{
  unsubscribe();
  clear();
}

void MapDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if( !topic_property_->getTopic().isEmpty() )
  {
    try
    {
      if(unreliable_property_->getBool())
      {
        map_sub_ = update_nh_.subscribe( topic_property_->getTopicStd(), 1, &MapDisplay::incomingMap, this,  ros::TransportHints().unreliable());
      }else{
        map_sub_ = update_nh_.subscribe( topic_property_->getTopicStd(), 1, &MapDisplay::incomingMap, this, ros::TransportHints().reliable() );
      }
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
    }

    try
    {
      update_sub_ = update_nh_.subscribe( topic_property_->getTopicStd() + "_updates", 1, &MapDisplay::incomingUpdate, this );
      setStatus( StatusProperty::Ok, "Update Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Update Topic", QString( "Error subscribing: " ) + e.what() );
    }
  }
}

void MapDisplay::unsubscribe()
{
  map_sub_.shutdown();
  update_sub_.shutdown();
}

void MapDisplay::updateAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::SceneBlendType sceneBlending;
  bool depthWrite;

  if( alpha < 0.9998 || color_scheme_transparency_[ color_scheme_property_->getOptionInt() ])
  {
    sceneBlending = Ogre::SBT_TRANSPARENT_ALPHA;
    depthWrite = false;
  }
  else
  {
    sceneBlending = Ogre::SBT_REPLACE;
    depthWrite = !draw_under_property_->getValue().toBool();
  }

  AlphaSetter alpha_setter( alpha );

  for (unsigned i=0; i < swatches.size(); i++) {
    swatches[i]->updateAlpha(sceneBlending, depthWrite, &alpha_setter);
  }
}

void MapDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getValue().toBool();

  if( alpha_property_->getFloat() >= 0.9998 )
  {
    for (unsigned i=0; i < swatches.size(); i++)
      swatches[i]->material_->setDepthWriteEnabled( !draw_under );
  }

  int group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (unsigned i=0; i < swatches.size(); i++){
    if( swatches[i]->manual_object_ )
      swatches[i]->manual_object_->setRenderQueueGroup( group );
  }
}

void MapDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
  clear();
}

void MapDisplay::clear()
{
  setStatus( StatusProperty::Warn, "Message", "No map received" );

  if( !loaded_ )
  {
    return;
  }

  for (unsigned i=0; i < swatches.size(); i++){
    if( swatches[i]->manual_object_ )
      swatches[i]->manual_object_->setVisible( false );

    if( !swatches[i]->texture_.isNull() )
    {
      Ogre::TextureManager::getSingleton().remove( swatches[i]->texture_->getName() );
      swatches[i]->texture_.setNull();
    }
  }

  loaded_ = false;
}

bool validateFloats(const nav_msgs::OccupancyGrid& msg)
{
  bool valid = true;
  valid = valid && validateFloats( msg.info.resolution );
  valid = valid && validateFloats( msg.info.origin );
  return valid;
}

void MapDisplay::incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  current_map_ = *msg;
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
  loaded_ = true;
}


void MapDisplay::incomingUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& update)
{
  // Only update the map if we have gotten a full one first.
  if( !loaded_ )
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if( update->x < 0 ||
      update->y < 0 ||
      current_map_.info.width < update->x + update->width ||
      current_map_.info.height < update->y + update->height )
  {
    setStatus( StatusProperty::Error, "Update", "Update area outside of original map area." );
    return;
  }

  // Copy the incoming data into current_map_'s data.
  for( size_t y = 0; y < update->height; y++ )
  {
    memcpy( &current_map_.data[ (update->y + y) * current_map_.info.width + update->x ],
            &update->data[ y * update->width ],
            update->width );
  }
  // updated via signal in case ros spinner is in a different thread
  Q_EMIT mapUpdated();
}

void MapDisplay::createSwatches()
{
  int width = current_map_.info.width;
  int height = current_map_.info.height;
  float resolution = current_map_.info.resolution;

  int sw = width;
  int sh = height;
  int n_swatches = 1;

  for(int i=0;i<4;i++){
    ROS_INFO("Creating %d swatches", n_swatches);
    for (unsigned i=0; i < swatches.size(); i++){
        delete swatches[i];
    }
    swatches.clear();
    try
    {
      int x = 0;
      int y = 0;
      for(int i=0;i<n_swatches;i++){
        int tw, th;
        if(width - x - sw >= sw)
          tw = sw;
        else
          tw = width - x;

        if(height - y - sh >= sh)
          th = sh;
        else
          th = height - y;

        swatches.push_back(new Swatch(this, x, y, tw, th, resolution));
        swatches[i]->updateData();

        x += tw;
        if(x>=width){
            x = 0;
            y += sh;
        }
      }
      updateAlpha();
      return;
    }catch(Ogre::RenderingAPIException&)
    {
        ROS_WARN("Failed to create %d swatches", n_swatches);
        if(sw>sh) sw/=2;
        else      sh/=2;
        n_swatches *= 2;
    }
  }
}

void MapDisplay::showMap()
{
  if (current_map_.data.empty())
  {
    return;
  }

  if( !validateFloats( current_map_ ))
  {
    setStatus( StatusProperty::Error, "Map", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  if( !validateQuaternions( current_map_.info.origin ))
  {
    ROS_WARN_ONCE_NAMED( "quaternions", "Map received on topic '%s' contains unnormalized quaternions. "
                         "This warning will only be output once but may be true for others; "
                         "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                         topic_property_->getTopicStd().c_str() );
    ROS_DEBUG_NAMED( "quaternions", "Map received on topic '%s' contains unnormalized quaternions.", 
                     topic_property_->getTopicStd().c_str() );
  }

  if( current_map_.info.width * current_map_.info.height == 0 )
  {
    std::stringstream ss;
    ss << "Map is zero-sized (" << current_map_.info.width << "x" << current_map_.info.height << ")";
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    return;
  }

  setStatus( StatusProperty::Ok, "Message", "Map received" );

  ROS_DEBUG( "Received a %d X %d map @ %.3f m/pix\n",
             current_map_.info.width,
             current_map_.info.height,
             current_map_.info.resolution );

  float resolution = current_map_.info.resolution;

  int width = current_map_.info.width;
  int height = current_map_.info.height;

  if(width != width_ || height != height_ || resolution_ != resolution){
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }

  Ogre::Vector3 position( current_map_.info.origin.position.x,
                          current_map_.info.origin.position.y,
                          current_map_.info.origin.position.z );
  Ogre::Quaternion orientation;
  normalizeQuaternion( current_map_.info.origin.orientation, orientation );

  frame_ = current_map_.header.frame_id;
  if (frame_.empty())
  {
    frame_ = "/map";
  }

  bool map_status_set = false;
  if( width * height != current_map_.data.size() )
  {
    std::stringstream ss;
    ss << "Data size doesn't match width*height: width = " << width
       << ", height = " << height << ", data size = " << current_map_.data.size();
    setStatus( StatusProperty::Error, "Map", QString::fromStdString( ss.str() ));
    map_status_set = true;
  }

  for(int i=0;i<swatches.size();i++){
    swatches[i]->updateData();

    Ogre::Pass* pass = swatches[i]->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = NULL;
    if (pass->getNumTextureUnitStates() > 0)
    {
      tex_unit = pass->getTextureUnitState(0);
    }
    else
    {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatches[i]->texture_->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );
    swatches[i]->manual_object_->setVisible( true );
  }


  if( !map_status_set )
  {
    setStatus( StatusProperty::Ok, "Map", "Map OK" );
  }
  updatePalette();

  resolution_property_->setValue( resolution );
  width_property_->setValue( width );
  height_property_->setValue( height );
  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );

  transformMap();

  context_->queueRender();
}

void MapDisplay::updatePalette()
{
  int palette_index = color_scheme_property_->getOptionInt();

  for (unsigned i=0; i < swatches.size(); i++){
    Ogre::Pass* pass = swatches[i]->material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* palette_tex_unit = NULL;
    if( pass->getNumTextureUnitStates() > 1 )
    {
      palette_tex_unit = pass->getTextureUnitState( 1 );
    }
    else
    {
      palette_tex_unit = pass->createTextureUnitState();
    }
    palette_tex_unit->setTextureName( palette_textures_[ palette_index ]->getName() );
    palette_tex_unit->setTextureFiltering( Ogre::TFO_NONE );
  }

  updateAlpha();
}

void MapDisplay::transformMap()
{
  if (!loaded_)
  {
    return;
  }

  ros::Time transform_time;

  if (transform_timestamp_property_->getBool())
  {
    transform_time = current_map_.header.stamp;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(frame_, transform_time, current_map_.info.origin, position, orientation) &&
      !context_->getFrameManager()->transform(frame_, ros::Time(0), current_map_.info.origin, position, orientation))
  {
    ROS_DEBUG( "Error transforming map '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), frame_.c_str(), qPrintable( fixed_frame_ ));

    setStatus( StatusProperty::Error, "Transform",
               "No transform from [" + QString::fromStdString( frame_ ) + "] to [" + fixed_frame_ + "]" );
  }
  else
  {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );
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
  updateTopic();
}

void MapDisplay::setTopic( const QString &topic, const QString &datatype )
{
  topic_property_->setString( topic );
}

void MapDisplay::update( float wall_dt, float ros_dt ) {
  transformMap();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::MapDisplay, rviz::Display )
