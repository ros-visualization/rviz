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
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/selection/forwards.h"
#include "rviz/common.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/axes.h>
#include <ogre_tools/movable_text.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>


namespace rviz
{

struct FrameInfo
{
  FrameInfo();

  const Ogre::Vector3& getPositionInRobotSpace() { return robot_space_position_; }
  const Ogre::Quaternion& getOrientationInRobotSpace() { return robot_space_orientation_; }
  const std::string& getParent() { return parent_; }

  bool isEnabled() { return enabled_; }

  std::string name_;
  std::string parent_;
  ogre_tools::Axes* axes_;
  CollObjectHandle axes_coll_;
  ogre_tools::Arrow* parent_arrow_;
  ogre_tools::MovableText* name_text_;
  Ogre::SceneNode* name_node_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  float distance_to_parent_;
  Ogre::Quaternion arrow_orientation_;

  Ogre::Vector3 robot_space_position_;
  Ogre::Quaternion robot_space_orientation_;

  bool enabled_;

  CategoryPropertyWPtr category_;
  Vector3PropertyWPtr position_property_;
  QuaternionPropertyWPtr orientation_property_;
  StringPropertyWPtr parent_property_;
  BoolPropertyWPtr enabled_property_;

  CategoryPropertyWPtr tree_property_;
};

FrameInfo::FrameInfo()
: axes_( NULL )
, axes_coll_(NULL)
, parent_arrow_( NULL )
, name_text_( NULL )
, position_(Ogre::Vector3::ZERO)
, orientation_(Ogre::Quaternion::IDENTITY)
, distance_to_parent_( 0.0f )
, arrow_orientation_(Ogre::Quaternion::IDENTITY)
, robot_space_position_(Ogre::Vector3::ZERO)
, robot_space_orientation_(Ogre::Quaternion::IDENTITY)
, enabled_(true)
{

}

void TFDisplay::setFrameEnabled(FrameInfo* frame, bool enabled)
{
  frame->enabled_ = enabled;

  propertyChanged(frame->enabled_property_);

  if (frame->name_node_)
  {
    frame->name_node_->setVisible(show_names_ && enabled);
  }

  if (frame->axes_)
  {
    frame->axes_->getSceneNode()->setVisible(show_axes_ && enabled);
  }

  if (frame->parent_arrow_)
  {
    if (frame->distance_to_parent_ > 0.001f)
    {
      frame->parent_arrow_->getSceneNode()->setVisible(show_arrows_ && enabled);
    }
    else
    {
      frame->parent_arrow_->getSceneNode()->setVisible(false);
    }
  }

  if (all_enabled_ && !enabled)
  {
    all_enabled_ = false;

    propertyChanged(all_enabled_property_);
  }

  causeRender();
}

class FrameSelectionHandler : public SelectionHandler
{
public:
  FrameSelectionHandler(FrameInfo* frame, TFDisplay* display);
  virtual ~FrameSelectionHandler();

  virtual void createProperties(const Picked& obj, PropertyManager* property_manager);

private:
  FrameInfo* frame_;
  TFDisplay* display_;
};

FrameSelectionHandler::FrameSelectionHandler(FrameInfo* frame, TFDisplay* display)
: frame_(frame)
, display_(display)
{
}

FrameSelectionHandler::~FrameSelectionHandler()
{
}

void FrameSelectionHandler::createProperties(const Picked& obj, PropertyManager* property_manager)
{
  std::stringstream ss;
  ss << frame_->name_ << " Frame " << frame_->name_;

  CategoryPropertyWPtr cat = property_manager->createCategory( "Frame " + frame_->name_, ss.str(), CategoryPropertyWPtr() );
  properties_.push_back(cat);

  properties_.push_back(property_manager->createProperty<BoolProperty>( "Enabled", ss.str(), boost::bind( &FrameInfo::isEnabled, frame_ ),
                                                                         boost::bind( &TFDisplay::setFrameEnabled, display_, frame_, _1 ), cat, this ));
  properties_.push_back(property_manager->createProperty<StringProperty>( "Parent", ss.str(), boost::bind( &FrameInfo::getParent, frame_ ),
                                                                           StringProperty::Setter(), cat, this ));
  properties_.push_back(property_manager->createProperty<Vector3Property>( "Position", ss.str(), boost::bind( &FrameInfo::getPositionInRobotSpace, frame_ ),
                                                                            Vector3Property::Setter(), cat, this ));
  properties_.push_back(property_manager->createProperty<QuaternionProperty>( "Orientation", ss.str(), boost::bind( &FrameInfo::getOrientationInRobotSpace, frame_ ),
                                                                               QuaternionProperty::Setter(), cat, this ));
}

typedef std::set<FrameInfo*> S_FrameInfo;

TFDisplay::TFDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, update_timer_( 0.0f )
, update_rate_( 1.0f )
, show_names_( true )
, show_arrows_( true )
, show_axes_( true )
, all_enabled_(true)
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

  property_manager_->deleteChildren( tree_category_.lock() );

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

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }

  propertyChanged(show_names_property_);
}

void TFDisplay::setShowAxes( bool show )
{
  show_axes_ = show;

  axes_node_->setVisible( show );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }

  propertyChanged(show_axes_property_);
}

void TFDisplay::setShowArrows( bool show )
{
  show_arrows_ = show;

  arrows_node_->setVisible( show );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }

  propertyChanged(show_arrows_property_);
}

void TFDisplay::setAllEnabled(bool enabled)
{
  all_enabled_ = enabled;

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, enabled);
  }

  propertyChanged(all_enabled_property_);
}

void TFDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;

  propertyChanged(update_rate_property_);
}

void TFDisplay::update(float wall_dt, float ros_dt)
{
  update_timer_ += wall_dt;
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
  vis_manager_->getTFClient()->getFrameStrings( frames );

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
  info->axes_->getSceneNode()->setVisible( show_axes_ );
  info->axes_coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(info->axes_, SelectionHandlerPtr(new FrameSelectionHandler(info, this)));

  info->name_text_ = new ogre_tools::MovableText( frame, "Arial", 0.1 );
  info->name_text_->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject( info->name_text_ );
  info->name_node_->setVisible( show_names_ );

  info->parent_arrow_ = new ogre_tools::Arrow( scene_manager_, arrows_node_, 1.0f, 0.01, 1.0f, 0.08 );
  info->parent_arrow_->getSceneNode()->setVisible( false );

  info->enabled_ = true;

  info->category_ = property_manager_->createCategory( info->name_, property_prefix_ + info->name_, frames_category_, this );
  info->enabled_property_ = property_manager_->createProperty<BoolProperty>( "Enabled", property_prefix_ + info->name_, boost::bind( &FrameInfo::isEnabled, info ),
                                                                             boost::bind( &TFDisplay::setFrameEnabled, this, info, _1 ), info->category_, this );
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
  tf::TransformListener* tf = vis_manager_->getTFClient();

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), frame->name_ );

  if (tf->canTransform(fixed_frame_, frame->name_, ros::Time()))
  {
    try
    {
      tf->transformPose( fixed_frame_, pose, pose );
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming frame '%s' to frame '%s': %s", frame->name_.c_str(), fixed_frame_.c_str(), e.what() );
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
  frame->axes_->getSceneNode()->setVisible(show_axes_ && frame->enabled_);

  frame->name_node_->setPosition( frame->position_ );
  frame->name_node_->setVisible(show_names_ && frame->enabled_);

  propertyChanged(frame->position_property_);
  propertyChanged(frame->orientation_property_);

  std::string old_parent = frame->parent_;
  frame->parent_.clear();
  bool has_parent = tf->getParent( frame->name_, ros::Time(), frame->parent_ );
  if ( has_parent )
  {
    CategoryPropertyPtr cat_prop = frame->tree_property_.lock();
    if ( !cat_prop || old_parent != frame->parent_ )
    {
      M_FrameInfo::iterator parent_it = frames_.find( frame->parent_ );

      if ( parent_it != frames_.end() )
      {
        FrameInfo* parent = parent_it->second;

        if ( parent->tree_property_.lock() )
        {
          property_manager_->deleteProperty( cat_prop );
          frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + frame->name_ + "Tree", parent->tree_property_, this );
        }
      }
    }

    if ( show_arrows_ )
    {
      tf::Stamped<tf::Pose> parent_pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(), frame->parent_ );

      if (tf->canTransform(fixed_frame_, frame->parent_, ros::Time()))
      {
        try
        {
          tf->transformPose( fixed_frame_, parent_pose, parent_pose );
        }
        catch(tf::TransformException& e)
        {
          ROS_ERROR( "Error transforming frame '%s' to frame '%s': %s", frame->parent_.c_str(), fixed_frame_.c_str(), e.what() );
        }
      }

      Ogre::Vector3 parent_position = Ogre::Vector3( parent_pose.getOrigin().x(), parent_pose.getOrigin().y(), parent_pose.getOrigin().z() );
      robotToOgre( parent_position );

      Ogre::Vector3 direction = parent_position - frame->position_;
      float distance = direction.length();
      direction.normalise();

      Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );

      Ogre::Vector3 old_pos = frame->parent_arrow_->getPosition();

      bool distance_changed = fabsf(distance - frame->distance_to_parent_) > 0.0001f;
      if ( distance_changed )
      {
        frame->distance_to_parent_ = distance;
        float head_length = ( distance < 0.1 ) ? (0.1*distance) : 0.1;
        float shaft_length = distance - head_length;
        frame->parent_arrow_->set( shaft_length, 0.02, head_length, 0.08 );
        frame->parent_arrow_->setShaftColor( 0.8f, 0.8f, 0.3f, 1.0f );
        frame->parent_arrow_->setHeadColor( 1.0f, 0.1f, 0.6f, 1.0f );
      }

      if ( distance > 0.001f )
      {
        frame->parent_arrow_->getSceneNode()->setVisible( show_arrows_ && frame->enabled_ );
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
    CategoryPropertyPtr tree_prop = frame->tree_property_.lock();
    if ( !tree_prop || old_parent != frame->parent_ )
    {
      property_manager_->deleteProperty( tree_prop );
      frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + frame->name_ + "Tree", tree_category_, this );
    }

    frame->parent_arrow_->getSceneNode()->setVisible( false );
  }

  propertyChanged(frame->parent_property_);
}

void TFDisplay::deleteFrame(FrameInfo* frame)
{
  M_FrameInfo::iterator it = frames_.find( frame->name_ );
  ROS_ASSERT( it != frames_.end() );

  frames_.erase( it );

  delete frame->axes_;
  vis_manager_->getSelectionManager()->removeObject(frame->axes_coll_);
  delete frame->parent_arrow_;
  delete frame->name_text_;
  scene_manager_->destroySceneNode( frame->name_node_->getName() );
  property_manager_->deleteProperty( frame->category_.lock() );
  property_manager_->deleteProperty( frame->tree_property_.lock() );
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
  FloatPropertyPtr float_prop = update_rate_property_.lock();
  float_prop->setMin( 0.05 );

  frames_category_ = property_manager_->createCategory( "Frames", property_prefix_, parent_category_, this );

  all_enabled_property_ = property_manager_->createProperty<BoolProperty>( "All Enabled", property_prefix_, boost::bind( &TFDisplay::getAllEnabled, this ),
                                                                           boost::bind( &TFDisplay::setAllEnabled, this, _1 ), frames_category_, this );

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

} // namespace rviz

