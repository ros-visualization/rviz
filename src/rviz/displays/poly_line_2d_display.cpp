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

#include "poly_line_2d_display.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

namespace rviz
{

PolyLine2DDisplay::PolyLine2DDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, render_operation_( poly_line_render_ops::Lines )
, loop_( false )
, override_color_( false )
, new_message_( false )
, new_metadata_( false )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;
  manual_object_ = scene_manager_->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  cloud_ = new ogre_tools::PointCloud( scene_manager_, scene_node_ );
  cloud_->setBillboardType( Ogre::BBT_PERPENDICULAR_COMMON );
  cloud_->setCommonDirection( Ogre::Vector3::UNIT_Y );
  cloud_->setCommonUpVector( Ogre::Vector3::NEGATIVE_UNIT_Z );
  setAlpha( 1.0f );
  setPointSize( 0.05f );
  setZPosition( 0.0f );

  notifier_ = new tf::MessageNotifier<visualization_msgs::Polyline>(tf_, ros_node_, boost::bind(&PolyLine2DDisplay::incomingMessage, this, _1), "", "", 10);
}

PolyLine2DDisplay::~PolyLine2DDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject( manual_object_ );

  delete cloud_;
  delete notifier_;
}

void PolyLine2DDisplay::clear()
{
  manual_object_->clear();
  cloud_->clear();
}

void PolyLine2DDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PolyLine2DDisplay::setColor( const Color& color )
{
  color_ = color;

  propertyChanged(color_property_);

  processMessage();
  causeRender();
}

void PolyLine2DDisplay::setOverrideColor( bool override )
{
  override_color_ = override;

  propertyChanged(override_color_property_);

  processMessage();
  causeRender();
}

void PolyLine2DDisplay::setRenderOperation( int op )
{
  render_operation_ = op;

  propertyChanged(render_operation_property_);

  processMessage();
  causeRender();
}

void PolyLine2DDisplay::setLoop( bool loop )
{
  loop_ = loop;

  propertyChanged(loop_property_);

  processMessage();
  causeRender();
}

void PolyLine2DDisplay::setPointSize( float size )
{
  point_size_ = size;

  propertyChanged(point_size_property_);

  cloud_->setBillboardDimensions( size, size );
  causeRender();
}

void PolyLine2DDisplay::setZPosition( float z )
{
  z_position_ = z;

  propertyChanged(z_position_property_);

  scene_node_->setPosition( 0.0f, z, 0.0f );
  causeRender();
}

void PolyLine2DDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  cloud_->setAlpha( alpha );

  propertyChanged(alpha_property_);

  processMessage();
  causeRender();
}

void PolyLine2DDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !topic_.empty() )
  {
    notifier_->setTopic(topic_);
  }

  ros_node_->subscribe( "map_metadata", metadata_message_, &PolyLine2DDisplay::incomingMetadataMessage, this, 1 );
}

void PolyLine2DDisplay::unsubscribe()
{
  notifier_->setTopic("");

  ros_node_->unsubscribe( "map_metadata", &PolyLine2DDisplay::incomingMetadataMessage, this );
}

void PolyLine2DDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void PolyLine2DDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PolyLine2DDisplay::fixedFrameChanged()
{
  clear();

  notifier_->setTargetFrame( fixed_frame_ );
}

void PolyLine2DDisplay::update( float dt )
{
  if ( new_message_ )
  {
    new_message_ = false;

    processMessage();

    causeRender();
  }

  if ( new_metadata_ )
  {
    setPointSize( metadata_message_.resolution );
  }
}

void PolyLine2DDisplay::processMessage()
{
  boost::shared_ptr<visualization_msgs::Polyline> msg;
  {
    boost::mutex::scoped_lock lock(message_mutex_);

    msg = message_;
  }

  clear();

  if (!msg)
  {
    return;
  }

  std::string frame_id = msg->header.frame_id;
  if (frame_id.empty())
  {
    frame_id = "/map";
  }

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, z_position_ ) ),
                                ros::Time(), frame_id);

  if (tf_->canTransform(fixed_frame_, frame_id, ros::Time()))
  {
    try
    {
      tf_->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame 'map' to frame '%s'", fixed_frame_.c_str() );
    }
  }

  Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
  ogreToRobot( orientation );
  orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
  robotToOgre( orientation );

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  Ogre::ColourValue color;
  if ( override_color_ )
  {
    color = Ogre::ColourValue( color_.r_, color_.g_, color_.b_, alpha_ );
  }
  else
  {
    color = Ogre::ColourValue( msg->color.r, msg->color.g, msg->color.b, alpha_ );
  }

  uint32_t num_points = msg->get_points_size();
  if ( render_operation_ == poly_line_render_ops::Points )
  {
    typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
    V_Point points;
    points.resize( num_points );
    for(uint32_t i = 0; i < num_points; i++)
    {
      ogre_tools::PointCloud::Point& current_point = points[ i ];

      current_point.x_ = -msg->points[i].y;
      current_point.y_ = -msg->points[i].z; ///\todo Josh please check this orientation --Tully
      current_point.z_ = -msg->points[i].x;
      current_point.r_ = color.r;
      current_point.g_ = color.g;
      current_point.b_ = color.b;
    }

    cloud_->clear();

    if ( !points.empty() )
    {
      cloud_->addPoints( &points.front(), points.size() );
    }
  }
  else
  {
    manual_object_->estimateVertexCount( num_points );
    manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
    for( uint32_t i=0; i < num_points; ++i)
    {
      manual_object_->position(-msg->points[i].y, 0.0f, -msg->points[i].x);
      manual_object_->colour( color );
    }

    if ( loop_ && num_points > 0 )
    {
      manual_object_->position(-msg->points[num_points - 1].y, 0.0f, -msg->points[num_points - 1].x);
      manual_object_->colour( color );
      manual_object_->position(-msg->points[0].y, 0.0f, -msg->points[0].x);
      manual_object_->colour( color );
    }

    manual_object_->end();
  }
}

void PolyLine2DDisplay::incomingMessage(const boost::shared_ptr<visualization_msgs::Polyline>& msg)
{
  boost::mutex::scoped_lock lock(message_mutex_);
  new_message_ = true;
  message_ = msg;
}

void PolyLine2DDisplay::incomingMetadataMessage()
{
  new_metadata_ = true;
}


void PolyLine2DDisplay::reset()
{
  clear();
}

void PolyLine2DDisplay::createProperties()
{
  override_color_property_ = property_manager_->createProperty<BoolProperty>( "Override Color", property_prefix_, boost::bind( &PolyLine2DDisplay::getOverrideColor, this ),
                                                                              boost::bind( &PolyLine2DDisplay::setOverrideColor, this, _1 ), category_, this );
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PolyLine2DDisplay::getColor, this ),
                                                                      boost::bind( &PolyLine2DDisplay::setColor, this, _1 ), category_, this );

  loop_property_ = property_manager_->createProperty<BoolProperty>( "Loop", property_prefix_, boost::bind( &PolyLine2DDisplay::getLoop, this ),
                                                                    boost::bind( &PolyLine2DDisplay::setLoop, this, _1 ), category_, this );

  render_operation_property_ = property_manager_->createProperty<EnumProperty>( "Render Operation", property_prefix_, boost::bind( &PolyLine2DDisplay::getRenderOperation, this ),
                                                                                boost::bind( &PolyLine2DDisplay::setRenderOperation, this, _1 ), category_, this );
  EnumPropertyPtr enum_prop = render_operation_property_.lock();
  enum_prop->addOption( "Lines", poly_line_render_ops::Lines );
  enum_prop->addOption( "Points", poly_line_render_ops::Points );

  /*point_size_property_ = property_manager_->createProperty<FloatProperty>( "Point Size", property_prefix_, boost::bind( &PolyLine2DDisplay::getPointSize, this ),
                                                                      boost::bind( &PolyLine2DDisplay::setPointSize, this, _1 ), category_, this );*/
  z_position_property_ = property_manager_->createProperty<FloatProperty>( "Z Position", property_prefix_, boost::bind( &PolyLine2DDisplay::getZPosition, this ),
                                                                        boost::bind( &PolyLine2DDisplay::setZPosition, this, _1 ), category_, this );
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &PolyLine2DDisplay::getAlpha, this ),
                                                                       boost::bind( &PolyLine2DDisplay::setAlpha, this, _1 ), category_, this );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PolyLine2DDisplay::getTopic, this ),
                                                                                boost::bind( &PolyLine2DDisplay::setTopic, this, _1 ), category_, this );
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(visualization_msgs::Polyline::__s_getDataType());
}

const char* PolyLine2DDisplay::getDescription()
{
  return "Displays data from a visualization_msgs::Polyline message as either points or lines.";
}

} // namespace rviz

