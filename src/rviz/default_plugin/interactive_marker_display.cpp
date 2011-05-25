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

#include "interactive_marker_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "interactive_markers/interactive_marker_tools.h"


#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

//////////////
bool validateFloats(const visualization_msgs::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose);
  valid = valid && validateFloats(msg.scale);
  for ( unsigned c=0; c<msg.controls.size(); c++)
  {
    valid = valid && validateFloats( msg.controls[c].orientation );
    for ( unsigned m=0; m<msg.controls[c].markers.size(); m++ )
    {
      valid = valid && validateFloats(msg.controls[c].markers[m].pose);
      valid = valid && validateFloats(msg.controls[c].markers[m].scale);
      valid = valid && validateFloats(msg.controls[c].markers[m].color);
      valid = valid && validateFloats(msg.controls[c].markers[m].points);
    }
  }
  return valid;
}
/////////////



InteractiveMarkerDisplay::InteractiveMarkerDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, tf_filter_(*manager->getTFClient(), "", 100, update_nh_)
, show_names_(true)
, show_tool_tips_(true)
, show_axes_(false)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  //message flow: incomingMarkerUpdate(..) -> tf_filter_ -> tfMarkerSuccess(..) -> message_queue_ -> update(..)

  tf_filter_.registerCallback(boost::bind(&InteractiveMarkerDisplay::tfMarkerSuccess, this, _1));
  tf_filter_.registerFailureCallback(boost::bind(&InteractiveMarkerDisplay::tfMarkerFail, this, _1, _2));
}

InteractiveMarkerDisplay::~InteractiveMarkerDisplay()
{
  unsubscribe();
  scene_manager_->destroySceneNode( scene_node_ );
}

void InteractiveMarkerDisplay::onEnable()
{
  subscribe();
  scene_node_->setVisible( true );
}

void InteractiveMarkerDisplay::onDisable()
{
  unsubscribe();
  tf_filter_.clear();
  scene_node_->setVisible( false );
}

void InteractiveMarkerDisplay::setMarkerUpdateTopic(const std::string& topic)
{
  if ( marker_update_topic_.empty() && topic.empty() )
  {
    return;
  }
  unsubscribe();
  marker_update_topic_ = topic;
  subscribe();
  propertyChanged(marker_update_topic_property_);
}

void InteractiveMarkerDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  marker_update_sub_.shutdown();
  publisher_contexts_.clear();
  num_publishers_ = 0;

  try
  {
    if ( !marker_update_topic_.empty() )
    {
      ROS_DEBUG( "Subscribing to %s", marker_update_topic_.c_str() );
      marker_update_sub_ = update_nh_.subscribe(marker_update_topic_, 100, &InteractiveMarkerDisplay::processMarkerUpdate, this);
    }

    setStatus(status_levels::Ok, "Topic", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
  }
}

void InteractiveMarkerDisplay::unsubscribe()
{
  interactive_markers_.clear();
  marker_update_sub_.shutdown();
}

void InteractiveMarkerDisplay::processMarkerUpdate(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr& marker_update)
{
  // get caller ID of the sending entity
  std::string callerid = "<unknown caller id>";
  if ( marker_update->__connection_header->find("callerid") != marker_update->__connection_header->end() )
  {
    callerid = marker_update->__connection_header->find("callerid")->second;
  }

  std::map<std::string, PublisherContext>::iterator context_it = publisher_contexts_.find(callerid);

  if ( context_it == publisher_contexts_.end() )
  {
    //create new context
    PublisherContext pc;
    pc.update_time_ok = true;
    context_it = publisher_contexts_.insert( std::make_pair(callerid,pc) ).first;
  }
  else if ( marker_update->seq_num != context_it->second.last_seq_num+1 )
  {
    // we've lost some updates
    ROS_ERROR_STREAM( "Detected lost update or server restart. Resetting. Reason: Received wrong sequence number (expected: " <<
        context_it->second.last_seq_num+1 << ", received: " << marker_update->seq_num << ")" );
    reset();
  }

  context_it->second.last_seq_num = marker_update->seq_num;
  context_it->second.last_update_time = ros::Time::now();

  std::set<std::string> names;
  std::vector<visualization_msgs::InteractiveMarker>::const_iterator it = marker_update->markers.begin();
  std::vector<visualization_msgs::InteractiveMarker>::const_iterator end = marker_update->markers.end();

  // pipe all markers into tf filter
  for (; it != end; ++it)
  {
    if ( !names.insert( it->name ).second )
    {
      setStatus(status_levels::Error, "Marker array", "The name '" + it->name + "' was used multiple times.");
    }

    visualization_msgs::InteractiveMarker::Ptr marker_ptr(new visualization_msgs::InteractiveMarker(*it));

    ROS_DEBUG("Forwarding %s to tf filter.", it->name.c_str());
    tf_filter_.add( marker_ptr );
  }

  // save pose updates
  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    std::vector<visualization_msgs::InteractiveMarkerPose>::const_iterator it = marker_update->poses.begin();
    std::vector<visualization_msgs::InteractiveMarkerPose>::const_iterator end = marker_update->poses.end();

    for (; it != end; ++it)
    {
      visualization_msgs::InteractiveMarkerPose::Ptr pose_ptr(new visualization_msgs::InteractiveMarkerPose(*it));
      pose_queue_.push_back( pose_ptr );
    }
  }
}

void InteractiveMarkerDisplay::tfMarkerSuccess( const visualization_msgs::InteractiveMarker::ConstPtr& marker )
{
  ROS_DEBUG("Queueing %s", marker->name.c_str());
  boost::mutex::scoped_lock lock(queue_mutex_);
  marker_queue_.push_back(marker);
}

void InteractiveMarkerDisplay::tfMarkerFail(const visualization_msgs::InteractiveMarker::ConstPtr& marker, tf::FilterFailureReason reason)
{
  std::string error = FrameManager::instance()->discoverFailureReason(marker->header.frame_id, marker->header.stamp, marker->__connection_header ? (*marker->__connection_header)["callerid"] : "unknown", reason);
  setStatus( status_levels::Error, marker->name, error);
  unsubscribe();
}

void InteractiveMarkerDisplay::update(float wall_dt, float ros_dt)
{
  //detect if all servers have shut down
  if ( !publisher_contexts_.empty() )
  {
    // weak detection of server shutdown
    unsigned num_pub = marker_update_sub_.getNumPublishers();
    if ( num_pub < num_publishers_ )
    {
      reset();
    }
    else
    {
      num_publishers_ = num_pub;
    }

    // detect weak connection
    std::map<std::string, PublisherContext>::iterator it;
    for ( it =publisher_contexts_.begin(); it!=publisher_contexts_.end(); it++ )
    {
      double time_since_last_update = (ros::Time::now() - it->second.last_update_time).toSec();
      if ( time_since_last_update > 1.0 )
      {
        std::stringstream s;
        s << "No update received for " << (int)time_since_last_update << " seconds. Connection might be lost.";
        setStatus( status_levels::Warn, it->first, s.str() );
        it->second.update_time_ok = false;
      }
      if ( !it->second.update_time_ok && time_since_last_update <= 1.0 )
      {
        setStatus( status_levels::Ok, it->first, "OK" );
      }
    }
  }

  V_InteractiveMarkerMessage local_marker_queue;
  V_InteractiveMarkerPoseMessage local_pose_queue;

  // the queue is accessed from another thread, so we need to lock it
  // and swap it's contents to a copy
  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    local_marker_queue.swap( marker_queue_ );
    local_pose_queue.swap( pose_queue_ );
  }

  if ( !local_marker_queue.empty() )
  {
    V_InteractiveMarkerMessage::iterator message_it = local_marker_queue.begin();
    V_InteractiveMarkerMessage::iterator message_end = local_marker_queue.end();
    for ( ; message_it != message_end; ++message_it )
    {
      visualization_msgs::InteractiveMarker::ConstPtr& marker = *message_it;

      if ( !validateFloats( *marker ) )
      {
        setStatus( status_levels::Error, marker->name, "Message contains invalid floats!" );
        continue;
      }
      ROS_DEBUG("Processing interactive marker '%s'. %d", marker->name.c_str(), (int)marker->controls.size() );

      std::map< std::string, InteractiveMarkerPtr >::iterator int_marker_entry = interactive_markers_.find( marker->name );

      std::string topic = marker_update_topic_;

      topic = ros::names::clean( topic );
      topic = topic.substr( 0, topic.find_last_of( '/' ) );

      if ( int_marker_entry == interactive_markers_.end() )
      {
        int_marker_entry = interactive_markers_.insert( std::make_pair(marker->name, InteractiveMarkerPtr ( new InteractiveMarker(this, vis_manager_, topic) ) ) ).first;
      }

      if ( int_marker_entry->second->processMessage( marker ) )
      {
        int_marker_entry->second->setShowAxes(show_axes_);
        int_marker_entry->second->setShowName(show_names_);
      }
    }
  }

  if ( !local_pose_queue.empty() )
  {
    V_InteractiveMarkerPoseMessage::iterator message_it = local_pose_queue.begin();
    V_InteractiveMarkerPoseMessage::iterator message_end = local_pose_queue.end();
    for ( ; message_it != message_end; ++message_it )
    {
      visualization_msgs::InteractiveMarkerPose::ConstPtr& pose = *message_it;

      std::map< std::string, InteractiveMarkerPtr >::iterator int_marker_entry = interactive_markers_.find( pose->name );

      if ( int_marker_entry != interactive_markers_.end() )
      {
        int_marker_entry->second->processMessage( pose );
      }

    }
  }

  M_StringToInteractiveMarkerPtr::iterator it;
  for ( it = interactive_markers_.begin(); it != interactive_markers_.end(); it++ )
  {
    it->second->update( wall_dt );
  }
}

void InteractiveMarkerDisplay::targetFrameChanged()
{
}

void InteractiveMarkerDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame( fixed_frame_ );
  reset();
}

void InteractiveMarkerDisplay::reset()
{
  ROS_DEBUG("reset");
  Display::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::createProperties()
{
  // interactive marker update topic
  marker_update_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>(
      "Marker Array Topic", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getMarkerUpdateTopic, this ),
      boost::bind( &InteractiveMarkerDisplay::setMarkerUpdateTopic, this, _1 ), parent_category_, this );

  setPropertyHelpText(marker_update_topic_property_, "visualization_msgs::InteractiveMarkerUpdate topic to subscribe to.");
  ROSTopicStringPropertyPtr marker_update_topic_property_locked = marker_update_topic_property_.lock();
  marker_update_topic_property_locked->setMessageType(ros::message_traits::datatype<visualization_msgs::InteractiveMarkerUpdate>());

  // display options
  show_names_property_ = property_manager_->createProperty<BoolProperty>(
      "Show Names", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getShowNames, this ),
      boost::bind( &InteractiveMarkerDisplay::setShowNames, this, _1 ), parent_category_, this );

  setPropertyHelpText(show_names_property_, "Whether or not to show the name of each Interactive Marker.");

  show_tool_tips_property_ = property_manager_->createProperty<BoolProperty>(
      "Show Tool Tips", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getShowToolTips, this ),
      boost::bind( &InteractiveMarkerDisplay::setShowToolTips, this, _1 ), parent_category_, this );

  setPropertyHelpText(show_tool_tips_property_, "Whether or not to show tool tips for the Interactive Marker Controls.");

  show_axes_property_ = property_manager_->createProperty<BoolProperty>(
      "Show Axes", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getShowAxes, this ),
      boost::bind( &InteractiveMarkerDisplay::setShowAxes, this, _1 ), parent_category_, this );

  setPropertyHelpText(show_axes_property_, "Whether or not to show the axes of each Interactive Marker.");
}


void InteractiveMarkerDisplay::setShowNames( bool show )
{
  show_names_ = show;

  M_StringToInteractiveMarkerPtr::iterator it;
  for ( it = interactive_markers_.begin(); it != interactive_markers_.end(); it++ )
  {
    it->second->setShowName(show);
  }

  propertyChanged(show_names_property_);
}


void InteractiveMarkerDisplay::setShowToolTips( bool show )
{
  show_tool_tips_ = show;

  propertyChanged(show_names_property_);
}


void InteractiveMarkerDisplay::setShowAxes( bool show )
{
  show_axes_ = show;

  M_StringToInteractiveMarkerPtr::iterator it;
  for ( it = interactive_markers_.begin(); it != interactive_markers_.end(); it++ )
  {
    it->second->setShowAxes(show);
  }

  propertyChanged(show_axes_property_);
}



} // namespace rviz
