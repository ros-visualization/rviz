/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/properties/property.h"
#include "rviz/selection/forwards.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/uniform_string_stream.h"

#include "rviz/default_plugin/tf_display.h"

namespace rviz
{

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

  Property* cat = property_manager->createCategory( "Frame " + frame_->name_, ss.str(), Property*() );
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
  , show_arrows_( true )
  , show_axes_( true )
  , frame_timeout_(15.0f)
  , all_enabled_(true)
  , scale_( 1 )
{
  show_names_property_ = new BoolProperty( "Show Names", true, "Whether or not names should be shown next to the frames.",
                                           this, SLOT( updateShowNames() ));

  show_axes_property_ = new BoolProperty( "Show Axes", true, "Whether or not the axes of each frame should be shown.",
                                          this, SLOT( updateShowAxes() ));

  show_arrows_property_ = new BoolProperty( "Show Arrows", true, "Whether or not arrows from child to parent should be shown."
                                            this, SLOT( updateShowArrows() ));

  scale_property_ = new FloatProperty( "Marker Scale", 1, "Scaling factor for all names, axes and arrows.", this );

  update_rate_property_ = new FloatProperty( "Update Interval", 0,
                                             "The interval, in seconds, at which to update the frame transforms.  0 means to do so every update cycle.",
                                             this );
  update_rate_property_->setMin( 0 );

  frame_timeout_property_ = new FloatProperty( "Frame Timeout", 15,
                                               "The length of time, in seconds, before a frame that has not been updated is considered \"dead\"."
                                               "  For 1/3 of this time the frame will appear correct, for the second 1/3rd it will fade to gray,"
                                               " and then it will fade out completely.",
                                               this );
  frame_timeout_property_->setMin( 1 );

  frames_category_ = new Property( "Frames", QVariant(), "The list of all frames.", this );

  all_enabled_property_ = new BoolProperty( "All Enabled", true,
                                            "Whether all the frames should be enabled or not.",
                                            frames_category_, SLOT( allEnabledChanged() ));

  tree_category_ = new Property( "Tree", QVariant(), "A tree-view of the frames, showing the parent/child relationships.", this );
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
  // Clear the tree.
  tree_category_->removeAllChildren();

  // Clear the frames category, except for the "All enabled" property.
  frames_category_->takeChild( all_enabled_property_ );
  frames_category_->removeAllChildren();
  frames_category_->addChild( all_enabled_property_ );

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

  names_node_->setVisible( show_names_property_->getBool() );
  arrows_node_->setVisible( show_arrows_property_->getBool() );
  axes_node_->setVisible( show_axes_property_->getBool() );
}

void TFDisplay::onDisable()
{
  root_node_->setVisible( false );
  clear();
}

void TFDisplay::updateShowNames()
{
  names_node_->setVisible( show_names_property_->getBool() );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::updateShowAxes()
{
  axes_node_->setVisible( show_axes_property_->getBool() );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::updateShowArrows()
{
  arrows_node_->setVisible( show_arrows_property_->getBool() );

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, frame->enabled_);
  }
}

void TFDisplay::allEnabledChanged()
{
  bool enabled = all_enabled_property_->getBool();

  M_FrameInfo::iterator it = frames_.begin();
  M_FrameInfo::iterator end = frames_.end();
  for (; it != end; ++it)
  {
    FrameInfo* frame = it->second;

    setFrameEnabled(frame, enabled);
  }
}

void TFDisplay::update(float wall_dt, float ros_dt)
{
  update_timer_ += wall_dt;
  if ( update_rate_ < 0.0001f || update_timer_ > update_rate_property_->getFloat() )
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
  context_->getTFClient()->getFrameStrings( frames );
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

  context_->queueRender();
}

static const Ogre::ColourValue ARROW_HEAD_COLOR(1.0f, 0.1f, 0.6f, 1.0f);
static const Ogre::ColourValue ARROW_SHAFT_COLOR(0.8f, 0.8f, 0.3f, 1.0f);

FrameInfo* TFDisplay::createFrame(const std::string& frame)
{
  FrameInfo* info = new FrameInfo( this );
  frames_.insert( std::make_pair( frame, info ) );

  info->name_ = frame;
  info->last_update_ = ros::Time::now();
  info->axes_ = new Axes( scene_manager_, axes_node_, 0.2, 0.02 );
  info->axes_->getSceneNode()->setVisible( show_axes_property_->getBool() );
  info->axes_coll_ =
    context_->getSelectionManager()->createCollisionForObject( info->axes_,
                                                               SelectionHandlerPtr( new FrameSelectionHandler( info, this )));

  info->name_text_ = new MovableText( frame, "Arial", 0.1 );
  info->name_text_->setTextAlignment(MovableText::H_CENTER, MovableText::V_BELOW);
  info->name_node_ = names_node_->createChildSceneNode();
  info->name_node_->attachObject( info->name_text_ );
  info->name_node_->setVisible( show_names_property_->getBool() );

  info->parent_arrow_ = new Arrow( scene_manager_, arrows_node_, 1.0f, 0.01, 1.0f, 0.08 );
  info->parent_arrow_->getSceneNode()->setVisible( false );
  info->parent_arrow_->setHeadColor(ARROW_HEAD_COLOR);
  info->parent_arrow_->setShaftColor(ARROW_SHAFT_COLOR);

  info->enabled_ = true;

  info->category_ = new Property( QString::fromStdString( info->name_ ), QVariant(), "", frames_category_ );

  info->enabled_property_ = new BoolProperty( "Enabled", true, "Enable or disable this individual frame.",
                                              info->category_, SLOT( updateVisibility() ), info );

  info->parent_property_ = new StringProperty( "Parent", "", "Parent of this frame.  (Not editable)",
                                               info->category_ );
  info->parent_property_->setReadOnly( true );

  info->position_property_ = new VectorProperty( "Position", Ogre::Vector::ZERO,
                                                 "Position of this frame, in the current Fixed Frame.  (Not editable)",
                                                 info->category_ );
  info->position_property_->setReadOnly( true );

  info->orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                        "Orientation of this frame, in the current Fixed Frame.  (Not editable)",
                                                        info->category_ );
  info->orientation_property_->setReadOnly( true );

  updateFrame( info );

  return info;
}

Ogre::ColourValue lerpColor(const Ogre::ColourValue& start, const Ogre::ColourValue& end, float t)
{
  return start * t + end * (1 - t);
}

void TFDisplay::updateFrame( FrameInfo* frame )
{
  tf::TransformListener* tf = context_->getTFClient();

  // Check last received time so we can grey out/fade out frames that have stopped being published
  ros::Time latest_time;
  tf->getLatestCommonTime( fixed_frame_.toStdString(), frame->name_, latest_time, 0 );
  if( latest_time != frame->last_time_to_fixed_ )
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

  setStatusStd(StatusProperty::Ok, frame->name_, "Transform OK");

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( frame->name_, ros::Time(), position, orientation ))
  {
    std::stringstream ss;
    ss << "No transform from [" << frame->name_ << "] to frame [" << fixed_frame_.toStdString() << "]";
    setStatusStd(StatusProperty::Warn, frame->name_, ss.str());
    ROS_DEBUG("Error transforming frame '%s' to frame '%s'", frame->name_.c_str(), fixed_frame_.c_str());
  }

//  frame->robot_space_position_ = frame->position_;
//  frame->robot_space_orientation_ = frame->orientation_;

  frame->axes_->setPosition( position );
  frame->axes_->setOrientation( orientation );
  frame->axes_->getSceneNode()->setVisible( show_axes_property_->getBool() && frame->enabled_);
  frame->axes_->setScale( Ogre::Vector3( scale_, scale_, scale_ ));

  frame->name_node_->setPosition( position );
  frame->name_node_->setVisible( show_names_property_->getBool() && frame->enabled_ );
  frame->name_node_->setScale( scale_, scale_, scale_ );

  frame->position_property_->setVector( position );
  frame->orientation_property_->setQuaternion( orientation );

  std::string old_parent = frame->parent_;
  frame->parent_.clear();
  bool has_parent = tf->getParent( frame->name_, ros::Time(), frame->parent_ );
  if( has_parent )
  {
    // If this frame has no tree property or the parent has changed,
    if( !frame->tree_property_ || old_parent != frame->parent_ )
    {
      // Look up the new parent.
      M_FrameInfo::iterator parent_it = frames_.find( frame->parent_ );
      if( parent_it != frames_.end() )
      {
        FrameInfo* parent = parent_it->second;

        // Delete the old tree property.
        delete frame->tree_property_;
        frame->tree_property_ = NULL;

        // If the parent has a tree property, make a new tree property for this frame.
        if( parent->tree_property_ )
        {
          frame->tree_property_ = new Property( QString::fromStdString( frame->name_ ), QVariant(), "", parent->tree_property_ );
        }
      }
    }
    if( show_arrows_property_->getBool() )
    {
      Ogre::Vector3 parent_position;
      Ogre::Quaternion parent_orientation;
      if (!context_->getFrameManager()->getTransform(frame->parent_, ros::Time(), parent_position, parent_orientation))
      {
        ROS_DEBUG("Error transforming frame '%s' (parent of '%s') to frame '%s'", frame->parent_.c_str(), frame->name_.c_str(), fixed_frame_.c_str());
      }

      Ogre::Vector3 direction = parent_position - position;
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
        frame->parent_arrow_->getSceneNode()->setVisible( show_arrows_property_->getBool() && frame->enabled_ );
      }
      else
      {
        frame->parent_arrow_->getSceneNode()->setVisible( false );
      }

      frame->parent_arrow_->setPosition( position );
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
      delete frame->tree_property_;
      frame->tree_property_ = new Property( QString::fromStdString( frame->name_ ), QVariant(), "", tree_category_ );
    }

    frame->parent_arrow_->getSceneNode()->setVisible( false );
  }

  frame->parent_property_->setStdString( frame->parent_ );
}

void TFDisplay::deleteFrame(FrameInfo* frame, bool delete_properties)
{
  M_FrameInfo::iterator it = frames_.find( frame->name_ );
  ROS_ASSERT( it != frames_.end() );

  frames_.erase( it );

  delete frame->axes_;
  context_->getSelectionManager()->removeObject(frame->axes_coll_);
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

void TFDisplay::fixedFrameChanged()
{
  update_timer_ = update_rate_;
}

void TFDisplay::reset()
{
  Display::reset();
  clear();
}

TFDisplay::FrameInfo::FrameInfo( TFDisplay* display )
  : display_( display )
  , axes_( NULL )
  , axes_coll_(NULL)
  , parent_arrow_( NULL )
  , name_text_( NULL )
  , distance_to_parent_( 0.0f )
  , arrow_orientation_(Ogre::Quaternion::IDENTITY)
  , enabled_(true)
  , tree_property_( NULL )
{}

void TFDisplay::FrameInfo::updateVisibility()
{
  bool enabled = enabled_property_->getBool();

  if( name_node_ )
  {
    name_node_->setVisible( display_->show_names_property_->getBool() && enabled );
  }

  if( axes_ )
  {
    axes_->getSceneNode()->setVisible( display_->show_axes_property_->getBool() && enabled );
  }

  if( parent_arrow_ )
  {
    if( distance_to_parent_ > 0.001f )
    {
      parent_arrow_->getSceneNode()->setVisible( display_->show_arrows_property_->getBool() && enabled );
    }
    else
    {
      parent_arrow_->getSceneNode()->setVisible( false );
    }
  }

  if( display_->all_enabled_property_->getBool() && !enabled)
  {
    display_->all_enabled_property_->setBool( false );
  }

  display_->context_->queueRender();
}

} // namespace rviz

