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

#include "robot.h"
#include "robot_link.h"
#include "common.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "visualization_manager.h"

#include "ogre_tools/object.h"
#include "ogre_tools/shape.h"
#include "ogre_tools/axes.h"

#include <tf/transform_listener.h>
#include <planning_models/kinematic.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRibbonTrail.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>

#include <ros/console.h>

namespace rviz
{

Robot::Robot( VisualizationManager* manager, const std::string& name )
: ogre_tools::Object( manager->getSceneManager() )
, visual_visible_( true )
, collision_visible_( false )
, vis_manager_(manager)
, property_manager_(NULL)
, user_data_( NULL )
, name_( name )
{
  root_visual_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_collision_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_other_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
  setAlpha(1.0f);
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode( root_visual_node_->getName() );
  scene_manager_->destroySceneNode( root_collision_node_->getName() );
  scene_manager_->destroySceneNode( root_other_node_->getName() );
}

void Robot::setVisible( bool visible )
{
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
    root_collision_node_->setVisible( collision_visible_ );
  }
  else
  {
    root_visual_node_->setVisible( false );
    root_collision_node_->setVisible( false );
  }
}

void Robot::setVisualVisible( bool visible )
{
  visual_visible_ = visible;
  root_visual_node_->setVisible( visible );
}

void Robot::setCollisionVisible( bool visible )
{
  collision_visible_ = visible;
  root_collision_node_->setVisible( visible );
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}

bool Robot::isCollisionVisible()
{
  return collision_visible_;
}

void Robot::setAlpha(float a)
{
  alpha_ = a;

  M_NameToLink::iterator it = links_.begin();
  M_NameToLink::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    RobotLink* info = it->second;

    info->setAlpha(alpha_);
  }
}

void Robot::setUserData( const Ogre::Any& user_data )
{
  user_data_ = user_data;
}

void Robot::clear()
{
  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* info = link_it->second;
    delete info;
  }

  if ( property_manager_ )
  {
    property_manager_->deleteByUserData( this );
  }

  links_category_.reset();
  links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
}

void Robot::setPropertyManager( PropertyManager* property_manager, const CategoryPropertyWPtr& parent )
{
  ROS_ASSERT( property_manager );
  ROS_ASSERT( parent.lock() );

  property_manager_ = property_manager;
  parent_property_ = parent;

  if ( !links_.empty() )
  {
    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
      RobotLink* info = link_it->second;

      info->createProperties();
    }
  }
}

void Robot::load( mechanism::Robot &descr, bool visual, bool collision )
{
  clear();

  if ( property_manager_ )
  {
    ROS_ASSERT(!links_category_.lock());
    links_category_ = property_manager_->createCategory( "Links", name_, parent_property_, this );
  }

  for (size_t i = 0; i < descr.links_.size(); ++i)
  {
    mechanism::Link &link = *descr.links_[i];

    RobotLink* link_info = new RobotLink(this, vis_manager_);
    link_info->load(descr, link, visual, collision);

    bool inserted = links_.insert( std::make_pair( link_info->getName(), link_info ) ).second;
    ROS_ASSERT( inserted );

    joint_to_link_[ link_info->getJointName() ] = link_info->getName();

    link_info->setAlpha(alpha_);
  }

  CategoryPropertyPtr cat_prop = links_category_.lock();
  cat_prop->collapse();

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());
}

RobotLink* Robot::getLink( const std::string& name )
{
  M_NameToLink::iterator it = links_.find( name );
  if ( it == links_.end() )
  {
    ROS_WARN( "Link [%s] does not exist", name.c_str() );
    return NULL;
  }

  return it->second;
}

void Robot::update( tf::TransformListener* tf, const std::string& target_frame )
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  tf->getFrameStrings( frames );

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const std::string& name = link_it->first;
    RobotLink* info = link_it->second;

    if ( std::find( frames.begin(), frames.end(), name ) == frames.end() )
    {
      ROS_ERROR( "Frame '%s' does not exist in the TF frame list", name.c_str() );

      info->setToErrorMaterial();
      continue;
    }

    info->setToNormalMaterial();

    tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), name );

    if (tf->canTransform(target_frame, name, ros::Time()))
    {
      try
      {
        tf->transformPose( target_frame, pose, pose );
      }
      catch(tf::TransformException& e)
      {
        ROS_ERROR( "Error transforming from frame '%s' to frame '%s'\n", name.c_str(), target_frame.c_str() );
      }
    }

    //printf( "Link %s:\npose: %6f %6f %6f,\t%6f %6f %6f\n", name.c_str(), pose.data_.getOrigin().x(), pose.data_.getOrigin().y(), pose.data_.getOrigin().z(), pose.data_.getOrigin().y()aw, pose.pitch, pose.roll );

    Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
    robotToOgre( position );

    btScalar yaw, pitch, roll;
    pose.getBasis().getEulerZYX( yaw, pitch, roll );

    Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( yaw, pitch, roll ) );

    // Collision/visual transforms are the same in this case
    info->setTransforms( position, orientation, position, orientation, true );
  }
}

void Robot::update( planning_models::KinematicModel* kinematic_model, const std::string& target_frame )
{
  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const std::string& name = link_it->first;
    RobotLink* info = link_it->second;

    planning_models::KinematicModel::Link* link = kinematic_model->getLink( name );

    if ( !link )
    {
      continue;
    }

    btVector3 robot_visual_position = link->globalTransFwd.getOrigin();
    btQuaternion robot_visual_orientation = link->globalTransFwd.getRotation();
    Ogre::Vector3 visual_position( robot_visual_position.getX(), robot_visual_position.getY(), robot_visual_position.getZ() );
    Ogre::Quaternion visual_orientation( robot_visual_orientation.getW(), robot_visual_orientation.getX(), robot_visual_orientation.getY(), robot_visual_orientation.getZ() );
    robotToOgre( visual_position );
    robotToOgre( visual_orientation );

    btVector3 robot_collision_position = link->globalTrans.getOrigin();
    btQuaternion robot_collision_orientation = link->globalTrans.getRotation();
    Ogre::Vector3 collision_position( robot_collision_position.getX(), robot_collision_position.getY(), robot_collision_position.getZ() );
    Ogre::Quaternion collision_orientation( robot_collision_orientation.getW(), robot_collision_orientation.getX(), robot_collision_orientation.getY(), robot_collision_orientation.getZ() );
    robotToOgre( collision_position );
    robotToOgre( collision_orientation );

    info->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation, false );
  }
}

void Robot::update( const robot_msgs::MechanismState& state )
{
  std::vector<robot_msgs::JointState>::const_iterator it = state.joint_states.begin();
  std::vector<robot_msgs::JointState>::const_iterator end = state.joint_states.end();
  for ( ; it != end; ++it )
  {
    const robot_msgs::JointState& joint_state = *it;
    M_string::iterator str_it = joint_to_link_.find( joint_state.name );
    if ( str_it != joint_to_link_.end() )
    {
      RobotLink* info = getLink( str_it->second );

      info->setJointState(joint_state);
    }
  }
}

void Robot::setPosition( const Ogre::Vector3& position )
{
  root_visual_node_->setPosition( position );
  root_collision_node_->setPosition( position );
}

void Robot::setOrientation( const Ogre::Quaternion& orientation )
{
  root_visual_node_->setOrientation( orientation );
  root_collision_node_->setOrientation( orientation );
}

void Robot::setScale( const Ogre::Vector3& scale )
{
  root_visual_node_->setScale( scale );
  root_collision_node_->setScale( scale );
}

const Ogre::Vector3& Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}

void Robot::setColor( float r, float g, float b, float a )
{
  /// @todo Make this work on the meshes as well?

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* info = link_it->second;
    info->setColor(r, g, b, a);
  }
}

} // namespace rviz
