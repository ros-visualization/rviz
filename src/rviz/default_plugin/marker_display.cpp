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

#include <sstream>

#include <tf/transform_listener.h>

#include "rviz/default_plugin/markers/arrow_marker.h"
#include "rviz/default_plugin/markers/line_list_marker.h"
#include "rviz/default_plugin/markers/line_strip_marker.h"
#include "rviz/default_plugin/markers/mesh_resource_marker.h"
#include "rviz/default_plugin/markers/points_marker.h"
#include "rviz/default_plugin/markers/shape_marker.h"
#include "rviz/default_plugin/markers/text_view_facing_marker.h"
#include "rviz/default_plugin/markers/triangle_list_marker.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"

#include "rviz/default_plugin/marker_display.h"

namespace rviz
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerDisplay::MarkerDisplay()
  : Display()
{
  marker_topic_property_ = new RosTopicProperty( "Marker Topic", "visualization_marker",
                                                 QString::fromStdString( ros::message_traits::datatype<visualization_msgs::Marker>() ),
                                                 "visualization_msgs::Marker topic to subscribe to.  <topic>_array will also"
                                                 " automatically be subscribed with type visualization_msgs::MarkerArray.",
                                                 this, SLOT( updateTopic() ));

  queue_size_property_ = new IntProperty( "Queue Size", 100,
                                          "Advanced: set the size of the incoming Marker message queue.  Increasing this is"
                                          " useful if your incoming TF data is delayed significantly from your Marker data, "
                                          "but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));
  queue_size_property_->setMin( 0 );

  namespaces_category_ = new Property( "Namespaces", QVariant(), "", this );
}

void MarkerDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<visualization_msgs::Marker>( *context_->getTFClient(),
                                                                  fixed_frame_.toStdString(),
                                                                  queue_size_property_->getInt(),
                                                                  update_nh_ );

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));
  tf_filter_->registerFailureCallback(boost::bind(&MarkerDisplay::failedMarker, this, _1, _2));
}

MarkerDisplay::~MarkerDisplay()
{
  if ( initialized() )
  {
    unsubscribe();

    clearMarkers();

    delete tf_filter_;
  }
}

void MarkerDisplay::clearMarkers()
{
  markers_.clear();
  markers_with_expiration_.clear();
  frame_locked_markers_.clear();
  tf_filter_->clear();
  namespaces_category_->removeChildren();
  namespaces_.clear();
}

void MarkerDisplay::onEnable()
{
  subscribe();
}

void MarkerDisplay::onDisable()
{
  unsubscribe();
  tf_filter_->clear();

  clearMarkers();
}

void MarkerDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void MarkerDisplay::updateTopic()
{
  unsubscribe();
  subscribe();
}

void MarkerDisplay::subscribe()
{
  if( !isEnabled() )
  {
    return;
  }

  std::string marker_topic = marker_topic_property_->getTopicStd();
  if( !marker_topic.empty() )
  {
    array_sub_.shutdown();
    sub_.unsubscribe();

    try
    {
      sub_.subscribe( update_nh_, marker_topic, queue_size_property_->getInt() );
      array_sub_ = update_nh_.subscribe( marker_topic + "_array", queue_size_property_->getInt(), &MarkerDisplay::incomingMarkerArray, this );
      setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what() );
    }
  }
}

void MarkerDisplay::unsubscribe()
{
  sub_.unsubscribe();
  array_sub_.shutdown();
}

void MarkerDisplay::deleteMarker(MarkerID id)
{
  deleteMarkerStatus( id );

  M_IDToMarker::iterator it = markers_.find( id );
  if( it != markers_.end() )
  {
    markers_with_expiration_.erase(it->second);
    frame_locked_markers_.erase(it->second);
    markers_.erase(it);
  }
}

void MarkerDisplay::deleteMarkersInNamespace( const std::string& ns )
{
  std::vector<MarkerID> to_delete;

  // TODO: this is inefficient, should store every in-use id per namespace and lookup by that
  M_IDToMarker::iterator marker_it = markers_.begin();
  M_IDToMarker::iterator marker_end = markers_.end();
  for (; marker_it != marker_end; ++marker_it)
  {
    if (marker_it->first.first == ns)
    {
      to_delete.push_back(marker_it->first);
    }
  }

  {
    std::vector<MarkerID>::iterator it = to_delete.begin();
    std::vector<MarkerID>::iterator end = to_delete.end();
    for (; it != end; ++it)
    {
      deleteMarker(*it);
    }
  }
}

void MarkerDisplay::deleteAllMarkers()
{
  M_IDToMarker::iterator marker_it = markers_.begin();
  for (; marker_it != markers_.end(); ++marker_it)
  {
    deleteMarker( marker_it->first );
  }
}

void MarkerDisplay::setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text)
{
  std::stringstream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  setStatusStd(level, marker_name, text);
}

void MarkerDisplay::deleteMarkerStatus(MarkerID id)
{
  std::stringstream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  deleteStatusStd(marker_name);
}

void MarkerDisplay::incomingMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array)
{
  std::vector<visualization_msgs::Marker>::const_iterator it = array->markers.begin();
  std::vector<visualization_msgs::Marker>::const_iterator end = array->markers.end();
  for (; it != end; ++it)
  {
    const visualization_msgs::Marker& marker = *it;
    tf_filter_->add(visualization_msgs::Marker::Ptr(new visualization_msgs::Marker(marker)));
  }
}

void MarkerDisplay::incomingMarker( const visualization_msgs::Marker::ConstPtr& marker )
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  message_queue_.push_back(marker);
}

void MarkerDisplay::failedMarker(const ros::MessageEvent<visualization_msgs::Marker>& marker_evt, tf::FilterFailureReason reason)
{
  visualization_msgs::Marker::ConstPtr marker = marker_evt.getConstMessage();
  std::string authority = marker_evt.getPublisherName();
  std::string error = context_->getFrameManager()->discoverFailureReason(marker->header.frame_id, marker->header.stamp, authority, reason);
  setMarkerStatus(MarkerID(marker->ns, marker->id), StatusProperty::Error, error);
}

bool validateFloats(const visualization_msgs::Marker& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose);
  valid = valid && validateFloats(msg.scale);
  valid = valid && validateFloats(msg.color);
  valid = valid && validateFloats(msg.points);
  return valid;
}

void MarkerDisplay::processMessage( const visualization_msgs::Marker::ConstPtr& message )
{
  if (!validateFloats(*message))
  {
    setMarkerStatus(MarkerID(message->ns, message->id), StatusProperty::Error, "Contains invalid floating point values (nans or infs)");
    return;
  }

  switch ( message->action )
  {
  case visualization_msgs::Marker::ADD:
    processAdd( message );
    break;

  case visualization_msgs::Marker::DELETE:
    processDelete( message );
    break;

  case 3: // TODO: visualization_msgs::Marker::DELETEALL when message changes in a future version of ROS
    deleteAllMarkers();
    break;

  default:
    ROS_ERROR( "Unknown marker action: %d\n", message->action );
  }
}

void MarkerDisplay::processAdd( const visualization_msgs::Marker::ConstPtr& message )
{
  QString namespace_name = QString::fromStdString( message->ns );
  M_Namespace::iterator ns_it = namespaces_.find( namespace_name );
  if( ns_it == namespaces_.end() )
  {
    ns_it = namespaces_.insert( namespace_name, new MarkerNamespace( namespace_name, namespaces_category_, this ));
  }

  if( !ns_it.value()->isEnabled() )
  {
    return;
  }

  deleteMarkerStatus( MarkerID( message->ns, message->id ));

  bool create = true;
  MarkerBasePtr marker;

  M_IDToMarker::iterator it = markers_.find( MarkerID(message->ns, message->id) );
  if ( it != markers_.end() )
  {
    marker = it->second;
    markers_with_expiration_.erase(marker);
    if ( message->type == marker->getMessage()->type )
    {
      create = false;
    }
    else
    {
      markers_.erase( it );
    }
  }

  if ( create )
  {
    switch ( message->type )
    {
    case visualization_msgs::Marker::CUBE:
    case visualization_msgs::Marker::CYLINDER:
    case visualization_msgs::Marker::SPHERE:
      {
        marker.reset(new ShapeMarker(this, context_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::ARROW:
      {
        marker.reset(new ArrowMarker(this, context_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::LINE_STRIP:
      {
        marker.reset(new LineStripMarker(this, context_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::LINE_LIST:
      {
        marker.reset(new LineListMarker(this, context_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::SPHERE_LIST:
    case visualization_msgs::Marker::CUBE_LIST:
    case visualization_msgs::Marker::POINTS:
      {
        marker.reset(new PointsMarker(this, context_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        marker.reset(new TextViewFacingMarker(this, context_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      {
        marker.reset(new MeshResourceMarker(this, context_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::TRIANGLE_LIST:
    {
      marker.reset(new TriangleListMarker(this, context_, scene_node_));
    }
    break;
    default:
      ROS_ERROR( "Unknown marker type: %d", message->type );
    }

    markers_.insert(std::make_pair(MarkerID(message->ns, message->id), marker));
  }

  if (marker)
  {
    marker->setMessage(message);

    if (message->lifetime.toSec() > 0.0001f)
    {
      markers_with_expiration_.insert(marker);
    }

    if (message->frame_locked)
    {
      frame_locked_markers_.insert(marker);
    }

    context_->queueRender();
  }
}

void MarkerDisplay::processDelete( const visualization_msgs::Marker::ConstPtr& message )
{
  deleteMarker(MarkerID(message->ns, message->id));

  context_->queueRender();
}

void MarkerDisplay::update(float wall_dt, float ros_dt)
{
  V_MarkerMessage local_queue;

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
      visualization_msgs::Marker::ConstPtr& marker = *message_it;

      processMessage( marker );
    }
  }

  {
    S_MarkerBase::iterator it = markers_with_expiration_.begin();
    S_MarkerBase::iterator end = markers_with_expiration_.end();
    for (; it != end;)
    {
      MarkerBasePtr marker = *it;
      if (marker->expired())
      {
        ++it;
        deleteMarker(marker->getID());
      }
      else
      {
        ++it;
      }
    }
  }

  {
    S_MarkerBase::iterator it = frame_locked_markers_.begin();
    S_MarkerBase::iterator end = frame_locked_markers_.end();
    for (; it != end; ++it)
    {
      MarkerBasePtr marker = *it;
      marker->updateFrameLocked();
    }
  }
}

void MarkerDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );

  clearMarkers();
}

void MarkerDisplay::reset()
{
  Display::reset();
  clearMarkers();
}

void MarkerDisplay::setTopic( const QString &topic, const QString &datatype )
{
  marker_topic_property_->setString( topic );
}

/////////////////////////////////////////////////////////////////////////////////
// MarkerNamespace

MarkerNamespace::MarkerNamespace( const QString& name, Property* parent_property, MarkerDisplay* owner )
  : BoolProperty( name, true,
                  "Enable/disable all markers in this namespace.",
                  parent_property )
  , owner_( owner )
{
  // Can't do this connect in chained constructor above because at
  // that point it doesn't really know that "this" is a
  // MarkerNamespace*, so the signal doesn't get connected.
  connect( this, SIGNAL( changed() ), this, SLOT( onEnableChanged() ));
}

void MarkerNamespace::onEnableChanged()
{
  if( !isEnabled() )
  {
    owner_->deleteMarkersInNamespace( getName().toStdString() );
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::MarkerDisplay, rviz::Display )
