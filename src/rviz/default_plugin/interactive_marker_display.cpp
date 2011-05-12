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

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{


InteractiveMarkerDisplay::InteractiveMarkerDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, tf_filter_(*manager->getTFClient(), "", 100, update_nh_)
, marker_topic_("interactive_marker")
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  tf_filter_.connectInput(marker_sub_);
  tf_filter_.registerCallback(boost::bind(&InteractiveMarkerDisplay::incomingMarker, this, _1));
  tf_filter_.registerFailureCallback(boost::bind(&InteractiveMarkerDisplay::failedMarker, this, _1, _2));
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

void InteractiveMarkerDisplay::setMarkerTopic(const std::string& topic)
{
  if ( marker_topic_.empty() && topic.empty() )
  {
    return;
  }
  unsubscribe();
  marker_topic_ = topic;
  marker_array_topic_ = "";
  subscribe();
  propertyChanged(marker_topic_property_);
  propertyChanged(marker_array_topic_property_);
}

void InteractiveMarkerDisplay::setMarkerArrayTopic(const std::string& topic)
{
  if ( marker_array_topic_.empty() && topic.empty() )
  {
    return;
  }
  unsubscribe();
  marker_topic_ = "";
  marker_array_topic_ = topic;
  subscribe();
  propertyChanged(marker_topic_property_);
  propertyChanged(marker_array_topic_property_);
}

void InteractiveMarkerDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  marker_sub_.unsubscribe();
  marker_array_sub_.shutdown();

  try
  {
    if ( !marker_topic_.empty() )
    {
      marker_sub_.subscribe(update_nh_, marker_topic_, 1);
    }
    if ( !marker_array_topic_.empty() )
    {
      ROS_INFO( "Subscribing to %s", marker_array_topic_.c_str() );
      marker_array_sub_ = update_nh_.subscribe(marker_array_topic_, 1, &InteractiveMarkerDisplay::incomingMarkerArray, this);
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
  marker_sub_.unsubscribe();
  marker_array_sub_.shutdown();
}

void InteractiveMarkerDisplay::incomingMarker( const visualization_msgs::InteractiveMarker::ConstPtr& marker )
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  //ROS_INFO("Received interactive marker. controls: %d frame_id: %s", (int)marker->controls.size(), marker->header.frame_id.c_str());
  message_queue_.push_back(marker);
}

void InteractiveMarkerDisplay::incomingMarkerArray(const visualization_msgs::InteractiveMarkerArray::ConstPtr& array)
{
  std::set<std::string> names;
  //copy all the messages and pass them to our tf filter
  //they will get passed on to incomingMarker if everything goes right
  std::vector<visualization_msgs::InteractiveMarker>::const_iterator it = array->markers.begin();
  std::vector<visualization_msgs::InteractiveMarker>::const_iterator end = array->markers.end();
  for (; it != end; ++it)
  {
    const visualization_msgs::InteractiveMarker& marker = *it;
    if ( !names.insert( marker.name ).second )
    {
      setStatus(status_levels::Error, "Marker array", "The name '" + marker.name + "' was used multiple times.");
    }
    tf_filter_.add(visualization_msgs::InteractiveMarker::Ptr(new visualization_msgs::InteractiveMarker(marker)));
  }
}

void InteractiveMarkerDisplay::failedMarker(const visualization_msgs::InteractiveMarker::ConstPtr& marker, tf::FilterFailureReason reason)
{
  std::string error = FrameManager::instance()->discoverFailureReason(marker->header.frame_id, marker->header.stamp, marker->__connection_header ? (*marker->__connection_header)["callerid"] : "unknown", reason);
  setStatus( status_levels::Error, "Marker", error);
}

bool validateFloats(const visualization_msgs::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose);
  valid = valid && validateFloats(msg.size);
  for ( unsigned c=0; c<msg.controls.size(); c++)
  {
    valid = valid && validateFloats( msg.controls[c].vec );
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

void InteractiveMarkerDisplay::update(float wall_dt, float ros_dt)
{
  V_MarkerMessage local_queue;

  // the queue is accessed from another thread, so we need to lock it
  // and swap it's contents to a copy
  {
    boost::mutex::scoped_lock lock(queue_mutex_);
    local_queue.swap( message_queue_ );
  }

  if ( !local_queue.empty() )
  {
    V_MarkerMessage::iterator message_it = local_queue.begin();
    V_MarkerMessage::iterator message_end = local_queue.end();
    for ( ; message_it != message_end; ++message_it )
    {
      visualization_msgs::InteractiveMarker::ConstPtr& marker = *message_it;

      if ( !validateFloats( *marker ) )
      {
        setStatus( status_levels::Error, marker->name, "Message contains invalid floats!" );
        continue;
      }
      //ROS_INFO("Processing interactive marker '%s'. %d", marker->name.c_str(), (int)marker->controls.size() );

      if ( interactive_markers_.find( marker->name ) == interactive_markers_.end() )
      {
        interactive_markers_.insert( std::make_pair(marker->name, InteractiveMarkerPtr ( new InteractiveMarker(this, vis_manager_) ) ) );
      }

      interactive_markers_.find(marker->name)->second->processMessage( marker );
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
  Display::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::createProperties()
{
  marker_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>(
      "Marker Topic", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getMarkerTopic, this ),
      boost::bind( &InteractiveMarkerDisplay::setMarkerTopic, this, _1 ), parent_category_, this );

  setPropertyHelpText(marker_topic_property_, "visualization_msgs::InteractiveMarker topic to subscribe to.");
  ROSTopicStringPropertyPtr topic_prop = marker_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<visualization_msgs::InteractiveMarker>());

  marker_array_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>(
      "Marker Array Topic", property_prefix_, boost::bind( &InteractiveMarkerDisplay::getMarkerArrayTopic, this ),
      boost::bind( &InteractiveMarkerDisplay::setMarkerArrayTopic, this, _1 ), parent_category_, this );

  setPropertyHelpText(marker_topic_property_, "visualization_msgs::InteractiveMarkerArray topic to subscribe to.");
  ROSTopicStringPropertyPtr array_topic_prop = marker_array_topic_property_.lock();
  array_topic_prop->setMessageType(ros::message_traits::datatype<visualization_msgs::InteractiveMarkerArray>());
}

} // namespace rviz
