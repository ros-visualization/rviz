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

#include "planning_display.h"
#include "common.h"
#include "helpers/robot.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include <ogre_tools/axes.h>

#include <urdf/URDF.h>
#include <tf/transform_listener.h>
#include <planning_models/kinematic.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz
{

PlanningDisplay::PlanningDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, kinematic_model_( NULL )
, new_kinematic_path_( false )
, animating_path_( false )
, state_display_time_( 0.05f )
, visual_enabled_property_( NULL )
, collision_enabled_property_( NULL )
, state_display_time_property_( NULL )
, robot_description_property_( NULL )
, topic_property_( NULL )
{
  robot_ = new Robot( scene_manager_ );

  setVisualVisible( false );
  setCollisionVisible( true );
  robot_->setUserData( Ogre::Any( (void*)this ) );
}

PlanningDisplay::~PlanningDisplay()
{
  unsubscribe();

  delete robot_;
}

void PlanningDisplay::initialize( const std::string& description_param, const std::string& kinematic_path_topic )
{
  setRobotDescription( description_param );
  setTopic( kinematic_path_topic );
}

void PlanningDisplay::setRobotDescription( const std::string& description_param )
{
  description_param_ = description_param;

  if ( robot_description_property_ )
  {
    robot_description_property_->changed();
  }

  if ( isEnabled() )
  {
    load();
    causeRender();
  }
}

void PlanningDisplay::setTopic( const std::string& topic )
{
  unsubscribe();
  kinematic_path_topic_ = topic;
  subscribe();

  if ( topic_property_ )
  {
    topic_property_->changed();
  }
}

void PlanningDisplay::setStateDisplayTime( float time )
{
  state_display_time_ = time;

  if ( state_display_time_property_ )
  {
    state_display_time_property_->changed();
  }

  causeRender();
}

void PlanningDisplay::setVisualVisible( bool visible )
{
  robot_->setVisualVisible( visible );

  if ( visual_enabled_property_ )
  {
    visual_enabled_property_->changed();
  }

  causeRender();
}

void PlanningDisplay::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible );

  if ( collision_enabled_property_ )
  {
    collision_enabled_property_->changed();
  }

  causeRender();
}

bool PlanningDisplay::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool PlanningDisplay::isCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void PlanningDisplay::load()
{
  std::string content;
  ros_node_->getParam(description_param_, content);
  robot_desc::URDF file;
  file.loadString(content.c_str());

  TiXmlDocument doc;
  doc.Parse(content.c_str());
  if (!doc.RootElement())
    return;

  mechanism::Robot descr;
  descr.initXml(doc.RootElement());
  robot_->load( descr );

  delete kinematic_model_;
  kinematic_model_ = new planning_models::KinematicModel();
  kinematic_model_->setVerbose( false );
  kinematic_model_->build( file );
  kinematic_model_->reduceToRobotFrame();

  robot_->update( kinematic_model_, target_frame_ );
}

void PlanningDisplay::onEnable()
{
  subscribe();

  load();
  robot_->setVisible( true );
}

void PlanningDisplay::onDisable()
{
  unsubscribe();
  robot_->setVisible( false );
}

void PlanningDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !kinematic_path_topic_.empty() )
  {
    ros_node_->subscribe( kinematic_path_topic_, incoming_kinematic_path_message_, &PlanningDisplay::incomingKinematicPath, this, 0 );
  }

}

void PlanningDisplay::unsubscribe()
{
  if ( !kinematic_path_topic_.empty() )
  {
    ros_node_->unsubscribe( kinematic_path_topic_, &PlanningDisplay::incomingKinematicPath, this );
  }
}

void PlanningDisplay::update( float dt )
{
  incoming_kinematic_path_message_.lock();

  if ( !animating_path_ && new_kinematic_path_ )
  {
    displaying_kinematic_path_message_ = incoming_kinematic_path_message_;

    animating_path_ = true;
    new_kinematic_path_ = false;
    current_state_ = -1;
    current_state_time_ = state_display_time_ + 1.0f;

    if (displaying_kinematic_path_message_.start_state.vals.size() > 0)
	kinematic_model_->computeTransforms(&displaying_kinematic_path_message_.start_state.vals[0]);
    robot_->update( kinematic_model_, target_frame_ );
  }

  incoming_kinematic_path_message_.unlock();

  if ( animating_path_ )
  {
    if ( current_state_time_ > state_display_time_ )
    {
      ++current_state_;

      calculateRobotPosition();

      if ( (size_t)current_state_ < displaying_kinematic_path_message_.path.get_states_size() )
      {
        int group_id = kinematic_model_->getGroupID( displaying_kinematic_path_message_.model_name );
	if (displaying_kinematic_path_message_.path.states[ current_state_ ].vals.size() > 0)
	    kinematic_model_->computeTransformsGroup(&displaying_kinematic_path_message_.path.states[ current_state_ ].vals[0], group_id);
        robot_->update( kinematic_model_, target_frame_ );

        causeRender();
      }
      else
      {
        animating_path_ = false;
      }

      current_state_time_ = 0.0f;
    }

    current_state_time_ += dt;
  }
}

void PlanningDisplay::calculateRobotPosition()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), displaying_kinematic_path_message_.frame_id );

  if (tf_->canTransform(target_frame_, displaying_kinematic_path_message_.frame_id, ros::Time()))
  {
    try
    {
      tf_->transformPose( target_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming from frame '%s' to frame '%s'\n", pose.frame_id_.c_str(), target_frame_.c_str() );
    }
  }

  Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );
  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ( Ogre::Radian( yaw ), Ogre::Radian( pitch ), Ogre::Radian( roll ) );

  robot_->setPosition( position );
  robot_->setOrientation( orientation );
}

void PlanningDisplay::incomingKinematicPath()
{
  new_kinematic_path_ = true;
}

void PlanningDisplay::targetFrameChanged()
{
  calculateRobotPosition();
}

void PlanningDisplay::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Visual Enabled", property_prefix_, boost::bind( &PlanningDisplay::isVisualVisible, this ),
                                                                               boost::bind( &PlanningDisplay::setVisualVisible, this, _1 ), parent_category_, this );
  collision_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Collision Enabled", property_prefix_, boost::bind( &PlanningDisplay::isCollisionVisible, this ),
                                                                                 boost::bind( &PlanningDisplay::setCollisionVisible, this, _1 ), parent_category_, this );
  state_display_time_property_ = property_manager_->createProperty<FloatProperty>( "State Display Time", property_prefix_, boost::bind( &PlanningDisplay::getStateDisplayTime, this ),
                                                                                  boost::bind( &PlanningDisplay::setStateDisplayTime, this, _1 ), parent_category_, this );
  state_display_time_property_->setMin( 0.0001 );
  robot_description_property_ = property_manager_->createProperty<StringProperty>( "Robot Description", property_prefix_, boost::bind( &PlanningDisplay::getRobotDescription, this ),
                                                                                   boost::bind( &PlanningDisplay::setRobotDescription, this, _1 ), parent_category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PlanningDisplay::getTopic, this ),
                                                                               boost::bind( &PlanningDisplay::setTopic, this, _1 ), parent_category_, this );
  topic_property_->setMessageType(robot_msgs::DisplayKinematicPath::__s_getDataType());
}

const char* PlanningDisplay::getDescription()
{
  return "Displays a planned path given by a robot_msgs::DisplayKinematicPath message.";
}

} // namespace rviz


