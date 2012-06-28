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
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/uniform_string_stream.h"

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/movable_text.h>

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
  Axes* axes_;
  CollObjectHandle axes_coll_;
  Arrow* parent_arrow_;
  MovableText* name_text_;
  Ogre::SceneNode* name_node_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  float distance_to_parent_;
  Ogre::Quaternion arrow_orientation_;

  Ogre::Vector3 robot_space_position_;
  Ogre::Quaternion robot_space_orientation_;

  bool enabled_;

  ros::Time last_update_;
  ros::Time last_time_to_fixed_;

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
  UniformStringStream ss;
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

TFDisplay::TFDisplay()
  : Display()
  , update_timer_( 0.0f )
  , update_rate_( 0.0f )
  , show_names_( true )
  , show_arrows_( true )
  , show_axes_( true )
  , frame_timeout_(15.0f)
  , all_enabled_(true)
  , scale_( 1 )
{
}

TFDisplay::~TFDisplay()
{
  clear();

  root_node_->removeAndDestroyAllChildren();
  scene_manager_->destroySceneNode( root_node_->getName() );
}

void TFDisplay::onInitialize()
{
  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  names_node_ = root_node_->createChildSceneNode();
  arrows_node_ = root_node_->createChildSceneNode();
  axes_node_ = root_node_->createChildSceneNode();
}

void TFDisplay::clear()
{
  property_manager_->deleteChildren(tree_category_.lock());
  property_manager_->deleteChildren(frames_category_.lock());
  all_enabled_property_ = property_manager_->createProperty<BoolProperty>( "All Enabled", property_prefix_, boost::bind( &TFDisplay::getAllEnabled, this ),
                                                                           boost::bind( &TFDisplay::setAllEnabled, this, _1 ), frames_category_, this );
  setPropertyHelpText(all_enabled_property_, "Whether all the frames should be enabled or not.");

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
    deleteFrame( *delete_it, false );
  }

  frames_.clear();

  update_timer_ = 0.0f;

  clearStatuses();
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

void TFDisplay::setFrameTimeout(float timeout)
{
  frame_timeout_ = timeout;
  propertyChanged(frame_timeout_property_);
}

void TFDisplay::setScale(float scale) 
{ 
  scale_ = scale; 
  propertyChanged(scale_property_); 
} 

void TFDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;

  propertyChanged(update_rate_property_);
}

void TFDisplay::update(float wall_dt, float ros_dt)
{
  update_timer_ += wall_dt;
  if ( update_rate_ < 0.0001f || update_timer_ > update_rate_ )
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
  std::sort(frames.begin(), frames.end());

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
      deleteFrame( *delete_it, true );
    }
  }

  causeRender();
}

static const Ogre::ColourValue ARROW_HEAD_COLOR(1.0f, 0.1f, 0.6f, 1.0f);
static const Ogre::ColourValue ARROW_SHAFT_COLOR(0.8f, 0.8f, 0.3f, 1.0f);

FrameInfo* TFDisplay::createFrame(const std::string& frame)
{
  FrameInfo* info = new FrameInfo;
  frames_.insert( std::make_pair( frame, info ) );

  info->name_ = frame;
  info->last_update_ = ros::Time::now();
  info->axes_ = new Axes( scene_manager_, axes_node_, 0.2, 0.02 );
  info->axes_->getSceneNode()->setVisible( show_axes_ );
  info->axes_coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(info->axes_, SelectionHandlerPtr(new FrameSelectionHandler(info, this)));

  info->name_text_ = new MovableText( frame, "Arial", 0.1 );
  info->name_text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject( info->name_text_ );
  info->name_node_->setVisible( show_names_ );

  info->parent_arrow_ = new Arrow( scene_manager_, arrows_node_, 1.0f, 0.01, 1.0f, 0.08 );
  info->parent_arrow_->getSceneNode()->setVisible( false );
  info->parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
  info->parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);

  info->enabled_ = true;

  std::string prefix = property_prefix_ + "Frames.";

  info->category_ = property_manager_->createCategory( info->name_, prefix, frames_category_, this );

  prefix += info->name_ + ".";

  info->enabled_property_ = property_manager_->createProperty<BoolProperty>( "Enabled", prefix, boost::bind( &FrameInfo::isEnabled, info ),
                                                                             boost::bind( &TFDisplay::setFrameEnabled, this, info, _1 ), info->category_, this );
  setPropertyHelpText(info->enabled_property_, "Enable or disable this individual frame.");
  info->parent_property_ = property_manager_->createProperty<StringProperty>( "Parent", prefix, boost::bind( &FrameInfo::getParent, info ),
                                                                              StringProperty::Setter(), info->category_, this );
  setPropertyHelpText(info->parent_property_, "Parent of this frame.  (Not editable)");
  info->position_property_ = property_manager_->createProperty<Vector3Property>( "Position", prefix, boost::bind( &FrameInfo::getPositionInRobotSpace, info ),
                                                                                 Vector3Property::Setter(), info->category_, this );
  setPropertyHelpText(info->position_property_, "Position of this frame, in the current Fixed Frame.  (Not editable)");
  info->orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", prefix, boost::bind( &FrameInfo::getOrientationInRobotSpace, info ),
                                                                                    QuaternionProperty::Setter(), info->category_, this );
  setPropertyHelpText(info->orientation_property_, "Orientation of this frame, in the current Fixed Frame.  (Not editable)");
  updateFrame( info );

  return info;
}

Ogre::ColourValue lerpColor(const Ogre::ColourValue& start, const Ogre::ColourValue& end, float t)
{
  return start * t + end * (1 - t);
}

void TFDisplay::updateFrame(FrameInfo* frame)
{
  tf::TransformListener* tf = vis_manager_->getTFClient();

  // Check last received time so we can grey out/fade out frames that have stopped being published
  ros::Time latest_time;
  tf->getLatestCommonTime(fixed_frame_, frame->name_, latest_time, 0);
  if (latest_time != frame->last_time_to_fixed_)
  {
    frame->last_update_ = ros::Time::now();
    frame->last_time_to_fixed_ = latest_time;
  }

  // Fade from color -> grey, then grey -> fully transparent
  ros::Duration age = ros::Time::now() - frame->last_update_;
  float one_third_timeout = frame_timeout_ * 0.3333333f;
  if (age > ros::Duration(frame_timeout_))
  {
    frame->parent_arrow_->getSceneNode()->setVisible(false);
    frame->axes_->getSceneNode()->setVisible(false);
    frame->name_node_->setVisible(false);
    return;
  }
  else if (age > ros::Duration(one_third_timeout))
  {
    Ogre::ColourValue grey(0.7, 0.7, 0.7, 1.0);

    if (age > ros::Duration(one_third_timeout * 2))
    {
      float a = std::max(0.0, (frame_timeout_ - age.toSec())/one_third_timeout);
      Ogre::ColourValue c = Ogre::ColourValue(grey.r, grey.g, grey.b, a);

      frame->axes_->setXColor(c);
      frame->axes_->setYColor(c);
      frame->axes_->setZColor(c);
      frame->name_text_->setColor(c);
      frame->parent_arrow_->setColor(c.r, c.g, c.b, c.a);
    }
    else
    {
      float t = std::max(0.0, (one_third_timeout * 2 - age.toSec())/one_third_timeout);
      frame->axes_->setXColor(lerpColor(frame->axes_->getDefaultXColor(), grey, t));
      frame->axes_->setYColor(lerpColor(frame->axes_->getDefaultYColor(), grey, t));
      frame->axes_->setZColor(lerpColor(frame->axes_->getDefaultZColor(), grey, t));
      frame->name_text_->setColor(lerpColor(Ogre::ColourValue::White, grey, t));
      frame->parent_arrow_->setShaftColor(lerpColor(ARROW_SHAFT_COLOR, grey, t));
      frame->parent_arrow_->setHeadColor(lerpColor(ARROW_HEAD_COLOR, grey, t));
    }
  }
  else
  {
    frame->axes_->setToDefaultColors();
    frame->name_text_->setColor(Ogre::ColourValue::White);
    frame->parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
    frame->parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);
  }

  setStatus(status_levels::Ok, frame->name_, "Transform OK");

  if (!vis_manager_->getFrameManager()->getTransform(frame->name_, ros::Time(), frame->position_, frame->orientation_))
  {
    std::stringstream ss;
    ss << "No transform from [" << frame->name_ << "] to frame [" << fixed_frame_ << "]";
    setStatus(status_levels::Warn, frame->name_, ss.str());
    ROS_DEBUG("Error transforming frame '%s' to frame '%s'", frame->name_.c_str(), fixed_frame_.c_str());
  }

  frame->robot_space_position_ = frame->position_;
  frame->robot_space_orientation_ = frame->orientation_;

  frame->axes_->setPosition( frame->position_ );
  frame->axes_->setOrientation( frame->orientation_ );
  frame->axes_->getSceneNode()->setVisible(show_axes_ && frame->enabled_);
  frame->axes_->setScale( Ogre::Vector3(scale_,scale_,scale_) );

  frame->name_node_->setPosition( frame->position_ );
  frame->name_node_->setVisible(show_names_ && frame->enabled_);
  frame->name_node_->setScale(scale_,scale_,scale_);

  propertyChanged(frame->position_property_);
  propertyChanged(frame->orientation_property_);

  std::string old_parent = frame->parent_;
  frame->parent_.clear();
  bool has_parent = tf->getParent( frame->name_, ros::Time(), frame->parent_ );
  if ( has_parent )
  {
    {
      CategoryPropertyPtr cat_prop = frame->tree_property_.lock();
      if ( !cat_prop || old_parent != frame->parent_ )
      {
        M_FrameInfo::iterator parent_it = frames_.find( frame->parent_ );

        if ( parent_it != frames_.end() )
        {
          FrameInfo* parent = parent_it->second;

          property_manager_->deleteProperty( cat_prop );
          cat_prop.reset(); // Clear the last remaining reference, deleting the old tree property entirely

          if ( parent->tree_property_.lock() )
          {
            frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + "Tree.", parent->tree_property_, this );
          }
        }
      }
    }

    if ( show_arrows_ )
    {
      Ogre::Vector3 parent_position;
      Ogre::Quaternion parent_orientation;
      if (!vis_manager_->getFrameManager()->getTransform(frame->parent_, ros::Time(), parent_position, parent_orientation))
      {
        ROS_DEBUG("Error transforming frame '%s' (parent of '%s') to frame '%s'", frame->parent_.c_str(), frame->name_.c_str(), fixed_frame_.c_str());
      }

      Ogre::Vector3 direction = parent_position - frame->position_;
      float distance = direction.length();
      direction.normalise();

      Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );

      Ogre::Vector3 old_pos = frame->parent_arrow_->getPosition();

      frame->distance_to_parent_ = distance;
      float head_length = ( distance < 0.1*scale_ ) ? (0.1*scale_*distance) : 0.1*scale_;
      float shaft_length = distance - head_length;
      frame->parent_arrow_->set( shaft_length, 0.02*scale_, head_length, 0.08*scale_ );

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
      frame->tree_property_ = property_manager_->createCategory( frame->name_, property_prefix_ + "Tree.", tree_category_, this );
    }

    frame->parent_arrow_->getSceneNode()->setVisible( false );
  }

  propertyChanged(frame->parent_property_);
}

void TFDisplay::deleteFrame(FrameInfo* frame, bool delete_properties)
{
  M_FrameInfo::iterator it = frames_.find( frame->name_ );
  ROS_ASSERT( it != frames_.end() );

  frames_.erase( it );

  delete frame->axes_;
  vis_manager_->getSelectionManager()->removeObject(frame->axes_coll_);
  delete frame->parent_arrow_;
  delete frame->name_text_;
  scene_manager_->destroySceneNode( frame->name_node_->getName() );
  if( delete_properties )
  {
    property_manager_->deleteProperty( frame->category_.lock() );
    property_manager_->deleteProperty( frame->tree_property_.lock() );
  }
  delete frame;
}

void TFDisplay::createProperties()
{
  show_names_property_ = property_manager_->createProperty<BoolProperty>( "Show Names", property_prefix_, boost::bind( &TFDisplay::getShowNames, this ),
                                                                          boost::bind( &TFDisplay::setShowNames, this, _1 ), parent_category_, this );
  setPropertyHelpText(show_names_property_, "Whether or not names should be shown next to the frames.");
  show_axes_property_ = property_manager_->createProperty<BoolProperty>( "Show Axes", property_prefix_, boost::bind( &TFDisplay::getShowAxes, this ),
                                                                          boost::bind( &TFDisplay::setShowAxes, this, _1 ), parent_category_, this );
  setPropertyHelpText(show_axes_property_, "Whether or not the axes of each frame should be shown.");
  show_arrows_property_ = property_manager_->createProperty<BoolProperty>( "Show Arrows", property_prefix_, boost::bind( &TFDisplay::getShowArrows, this ),
                                                                           boost::bind( &TFDisplay::setShowArrows, this, _1 ), parent_category_, this );
  setPropertyHelpText(show_arrows_property_, "Whether or not arrows from child to parent should be shown.");
  scale_property_ = property_manager_->createProperty<FloatProperty>( "Marker Scale", property_prefix_, boost::bind( &TFDisplay::getScale, this ), 
                                                                      boost::bind( &TFDisplay::setScale, this, _1 ), parent_category_, this ); 
  setPropertyHelpText(scale_property_, "Scaling factor for all names, axes and arrows.");
  update_rate_property_ = property_manager_->createProperty<FloatProperty>( "Update Interval", property_prefix_, boost::bind( &TFDisplay::getUpdateRate, this ),
                                                                            boost::bind( &TFDisplay::setUpdateRate, this, _1 ), parent_category_, this );
  setPropertyHelpText(update_rate_property_, "The interval, in seconds, at which to update the frame transforms.  0 means to do so every update cycle.");
  FloatPropertyPtr float_prop = update_rate_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->addLegacyName("Update Rate");

  frame_timeout_property_ = property_manager_->createProperty<FloatProperty>( "Frame Timeout", property_prefix_, boost::bind( &TFDisplay::getFrameTimeout, this ),
                                                                              boost::bind( &TFDisplay::setFrameTimeout, this, _1 ), parent_category_, this );
  setPropertyHelpText(frame_timeout_property_, "The length of time, in seconds, before a frame that has not been updated is considered \"dead\".  For 1/3 of this time"
                                               " the frame will appear correct, for the second 1/3rd it will fade to gray, and then it will fade out completely.");
  float_prop = frame_timeout_property_.lock();
  float_prop->setMin( 1.0 );

  frames_category_ = property_manager_->createCategory( "Frames", property_prefix_, parent_category_, this );
  setPropertyHelpText(frames_category_, "The list of all frames.");
  CategoryPropertyPtr cat_prop = frames_category_.lock();
  cat_prop->collapse();

  all_enabled_property_ = property_manager_->createProperty<BoolProperty>( "All Enabled", property_prefix_, boost::bind( &TFDisplay::getAllEnabled, this ),
                                                                           boost::bind( &TFDisplay::setAllEnabled, this, _1 ), frames_category_, this );
  setPropertyHelpText(all_enabled_property_, "Whether all the frames should be enabled or not.");

  tree_category_ = property_manager_->createCategory( "Tree", property_prefix_, parent_category_, this );
  setPropertyHelpText(tree_category_, "A tree-view of the frames, showing the parent/child relationships.");
  cat_prop = tree_category_.lock();
  cat_prop->collapse();
}

void TFDisplay::fixedFrameChanged()
{
  update_timer_ = update_rate_;
}

void TFDisplay::reset()
{
  Display::reset();
  clear();
}

} // namespace rviz

