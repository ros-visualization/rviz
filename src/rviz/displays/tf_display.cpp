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

#include "tf_display.h"
#include "common.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>

#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf/transform_listener.h>

namespace rviz
{

FrameInfo::FrameInfo()
: axes_( NULL )
, parent_arrow_( NULL )
, name_text_( NULL )
, distance_to_parent_( 0.0f )
, category_( NULL )
, position_property_( NULL )
, orientation_property_( NULL )
, tree_property_( NULL )
{

}

typedef std::set<FrameInfo*> S_FrameInfo;

TFDisplay::TFDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, update_timer_( 0.0f )
, update_rate_( 1.0f )
, show_names_( true )
, show_arrows_( true )
, show_axes_( true )
, show_names_property_( NULL )
, show_arrows_property_( NULL )
, show_axes_property_( NULL )
, update_rate_property_( NULL )
, frames_category_( NULL )
, tree_category_( NULL )
{
  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  names_node_ = root_node_->createChildSceneNode();
  arrows_node_ = root_node_->createChildSceneNode();
  axes_node_ = root_node_->createChildSceneNode();
}

TFDisplay::~TFDisplay()
{
  clear();

  root_node_->removeAndDestroyAllChildren();
  scene_manager_->destroySceneNode( root_node_->getName() );
}

void TFDisplay::clear()
{
  S_FrameInfo to_delete;
  M_FrameInfo::iterator frame_it = frames_.begin();
  M_FrameInfo::iterator frame_end = frames_.end();
  for ( ; frame_it != frame_end; ++frame_it )
  {
    to_delete.insert( frame_it->second );
  }

  S_FrameInfo::iterator delete_it = to_delete.begin();
  S_FrameInfo::iterator delete_end = to_delete.end();
  for ( ; delete_it != delete_end; ++delete_it )
  {
    deleteFrame( *delete_it );
  }

  frames_.clear();

  property_manager_->deleteChildren( tree_category_ );

  update_timer_ = 0.0f;
}

void TFDisplay::onEnable()
{
  root_node_->setVisible( true );

  names_node_->setVisible( show_names_ );
  arrows_node_->setVisible( show_arrows_ );
  axes_node_->setVisible( show_axes_ );
}

void TFDisplay::onDisable()
{
  root_node_->setVisible( false );
  clear();
}

void TFDisplay::setShowNames( bool show )
{
  show_names_ = show;

  names_node_->setVisible( show );

  if ( show_names_property_ )
  {
    show_names_property_->changed();
  }
}

void TFDisplay::setShowAxes( bool show )
{
  show_axes_ = show;

  axes_node_->setVisible( show );

  if ( show_axes_property_ )
  {
    show_axes_property_->changed();
  }
}

void TFDisplay::setShowArrows( bool show )
{
  show_arrows_ = show;

  arrows_node_->setVisible( show );

  if ( show_arrows_property_ )
  {
    show_arrows_property_->changed();
  }
}

void TFDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;

  if ( update_rate_property_ )
  {
    update_rate_property_->changed();
  }
}

void TFDisplay::update( float dt )
{
  update_timer_ += dt;
  if ( update_timer_ > update_rate_ )
  {
    updateFrames();

    update_timer_ = 0.0f;
  }
}

FrameInfo* TFDisplay::getFrameInfo( const std::string& frame )
{
  M_FrameInfo::iterator it = frames_.find( frame );
  if ( it == frames_.end() )
  {
    return NULL;
  }

  return it->second;
}

void TFDisplay::updateFrames()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  tf_->getFrameStrings( frames );

  S_FrameInfo current_frames;

  {
    V_string::iterator it = frames.begin();
    V_string::iterator end = frames.end();
    for ( ; it != end; ++it )
    {
      const std::string& frame = *it;

      if ( frame.empty() )
      {
        continue;
      }

      FrameInfo* info = getFrameInfo( frame );
      if (!info)
      {
        info = createFrame(frame);
      }
      else
      {
        updateFrame(info);
      }

      current_frames.insert( info );
    }
  }

  {
    S_FrameInfo to_delete;
    M_FrameInfo::iterator frame_it = frames_.begin();
    M_FrameInfo::iterator frame_end = frames_.end();
    for ( ; frame_it != frame_end; ++frame_it )
    {
      if ( current_frames.find( frame_it->second ) == current_frames.end() )
      {
        to_delete.insert( frame_it->second );
      }
    }

    S_FrameInfo::iterator delete_it = to_delete.begin();
    S_FrameInfo::iterator delete_end = to_delete.end();
    for ( ; delete_it != delete_end; ++delete_it )
    {
      deleteFrame( *delete_it );
    }
  }

  causeRender();
}

FrameInfo* TFDisplay::createFrame(const std::string& frame)
{
  FrameInfo* info = new FrameInfo;
  frames_.insert( std::make_pair( frame, info ) );

  info->name_ = frame;
  info->axes_ = new ogre_tools::Axes( scene_manager_, axes_node_, 0.2, 0.02 );
  info->axes_->setUserData( Ogre::Any( (void*)this ) );
  info->axes_->getSceneNode()->setVisible( show_axes_ );

  info->name_text_ = new ogre_tools::MovableText( frame, "Arial", 0.1 );
  info->name_text_->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject( info->name_text_ );
  info->name_node_->setVisible( show_names_ );

  info->parent_arrow_ = new ogre_tools::Arrow( scene_manager_, arrows_node_, 1.0f, 0.01, 1.0f, 0.08 );
  info->parent_arrow_->getSceneNode()->setVisible( false );

  info->category_ = property_manager_->createCategory( info->name_, property_prefix_ + info->name_, frames_category_, this );
  info->parent_property_ = property_manager_->createProperty<StringProperty>( "Parent", property_prefix_ + info->name_, boost::bind( &FrameInfo::getParent, info ),
                                                                              StringProperty::Setter(), info->category_, this );
  info->position_property_ = property_manager_->createProperty<Vector3Property>( "Position", property_prefix_ + info->name_, boost::bind( &FrameInfo::getPositionInRobotSpace, info ),
                                                                                 Vector3Property::Setter(), info->category_, this );
  info->orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", property_prefix_ + info->name_, boost::bind( &FrameInfo::getOrientationInRobotSpace, info ),
                                                                                    QuaternionProperty::Setter(), info->category_, this );
  updateFrame( info );

  return info;
}

void TFDisplay::updateFrame(FrameInfo* frame)
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), frame->name_ );

  if (tf_->canTransform(fixed_frame_, frame->name_, ros::Time()))
  {
    try
    {
      tf_->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming frame '%s' to frame '%s'\n", frame->name_.c_str(), fixed_frame_.c_str() );
    }
  }

  frame->position_ = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  frame->robot_space_position_ = frame->position_;
  robotToOgre( frame->position_ );

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  frame->orientation_ = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() );
  frame->robot_space_orientation_ = frame->orientation_;
  robotToOgre( frame->orientation_ );

  frame->axes_->setPosition( frame->position_ );
  frame->axes_->setOrientation( frame->orientation_ );

  frame->name_node_->setPosition( frame->position_ );

  frame->position_property_->changed();
  frame->orientation_property_->changed();

  std::string old_parent = frame->parent_;
  frame->parent_.clear();
  bool has_parent = tf_->getParent( frame->name_, ros::Time(), frame->parent_ );
  if ( has_parent )
  {
    if ( !frame->tree_property_ || old_parent != frame->parent_ )
    {
      M_FrameInfo::iterator parent_it = frames_.find( frame->parent_ );

      if ( parent_it != frames_.end() )
      {
        FrameInfo* parent = parent_it->second;

        if ( parent->tree_property_ )
        {
          property_manager_->deleteProperty( frame->tree_property_ );
          frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + frame->name_ + "Tree", parent->tree_property_, this );
        }
      }
    }

    if ( show_arrows_ )
    {
      tf::Stamped<tf::Pose> parent_pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), frame->parent_ );

      if (tf_->canTransform(fixed_frame_, frame->parent_, ros::Time()))
      {
        try
        {
          tf_->transformPose( fixed_frame_, parent_pose, parent_pose );
        }
        catch(tf::TransformException& e)
        {
          ROS_ERROR( "Error transforming frame '%s' to frame '%s'\n", frame->parent_.c_str(), fixed_frame_.c_str() );
        }
      }

      Ogre::Vector3 parent_position = Ogre::Vector3( parent_pose.getOrigin().x(), parent_pose.getOrigin().y(), parent_pose.getOrigin().z() );
      robotToOgre( parent_position );

      Ogre::Vector3 direction = parent_position - frame->position_;
      float distance = direction.length();
      direction.normalise();

      Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );

      Ogre::Vector3 old_pos = frame->parent_arrow_->getPosition();

      // The set() operation on the arrow is rather expensive (has to clear/regenerate geometry), so
      // avoid doing it if possible
      bool distance_changed = fabsf(distance - frame->distance_to_parent_) > 0.0001f;
      if ( distance_changed )
      {
        frame->distance_to_parent_ = distance;
        float head_length = ( distance < 0.1 ) ? (0.1*distance) : 0.1;
        float shaft_length = distance - head_length;
        frame->parent_arrow_->set( shaft_length, 0.01, head_length, 0.08 );
        frame->parent_arrow_->setShaftColor( 0.8f, 0.8f, 0.3f, 1.0f );
        frame->parent_arrow_->setHeadColor( 1.0f, 0.1f, 0.6f, 1.0f );
        frame->parent_arrow_->setUserData( Ogre::Any( (void*)this ) );
      }

      if ( distance > 0.001f )
      {
        frame->parent_arrow_->getSceneNode()->setVisible( show_arrows_ );
      }
      else
      {
        frame->parent_arrow_->getSceneNode()->setVisible( false );
      }

      frame->parent_arrow_->setPosition( frame->position_ );
      frame->parent_arrow_->setOrientation( orient );
    }
    else
    {
      frame->parent_arrow_->getSceneNode()->setVisible( false );
    }
  }
  else
  {
    if ( !frame->tree_property_ || old_parent != frame->parent_ )
    {
      property_manager_->deleteProperty( frame->tree_property_ );
      frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + frame->name_ + "Tree", tree_category_, this );
    }

    frame->parent_arrow_->getSceneNode()->setVisible( false );
  }

  frame->parent_property_->changed();
}

void TFDisplay::deleteFrame(FrameInfo* frame)
{
  M_FrameInfo::iterator it = frames_.find( frame->name_ );
  ROS_ASSERT( it != frames_.end() );

  frames_.erase( it );

  delete frame->axes_;
  delete frame->parent_arrow_;
  delete frame->name_text_;
  scene_manager_->destroySceneNode( frame->name_node_->getName() );
  property_manager_->deleteProperty( frame->category_ );
  delete frame;
}

void TFDisplay::createProperties()
{
  show_names_property_ = property_manager_->createProperty<BoolProperty>( "Show Names", property_prefix_, boost::bind( &TFDisplay::getShowNames, this ),
                                                                          boost::bind( &TFDisplay::setShowNames, this, _1 ), parent_category_, this );
  show_axes_property_ = property_manager_->createProperty<BoolProperty>( "Show Axes", property_prefix_, boost::bind( &TFDisplay::getShowAxes, this ),
                                                                          boost::bind( &TFDisplay::setShowAxes, this, _1 ), parent_category_, this );
  show_arrows_property_ = property_manager_->createProperty<BoolProperty>( "Show Arrows", property_prefix_, boost::bind( &TFDisplay::getShowArrows, this ),
                                                                           boost::bind( &TFDisplay::setShowArrows, this, _1 ), parent_category_, this );
  update_rate_property_ = property_manager_->createProperty<FloatProperty>( "Update Rate", property_prefix_, boost::bind( &TFDisplay::getUpdateRate, this ),
                                                                            boost::bind( &TFDisplay::setUpdateRate, this, _1 ), parent_category_, this );
  update_rate_property_->setMin( 0.05 );

  frames_category_ = property_manager_->createCategory( "Frames", property_prefix_, parent_category_, this );
  tree_category_ = property_manager_->createCategory( "Tree", property_prefix_, parent_category_, this );
}

void TFDisplay::targetFrameChanged()
{
  update_timer_ = update_rate_;
}

void TFDisplay::reset()
{
  clear();
}

const char* TFDisplay::getDescription()
{
  return "Displays the TF transform hierarchy.";
}

} // namespace rviz

