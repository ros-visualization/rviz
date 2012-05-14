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

#include "marker_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/uniform_string_stream.h"

#include "markers/shape_marker.h"
#include "markers/arrow_marker.h"
#include "markers/line_list_marker.h"
#include "markers/line_strip_marker.h"
#include "markers/points_marker.h"
#include "markers/text_view_facing_marker.h"
#include "markers/mesh_resource_marker.h"
#include "markers/triangle_list_marker.h"

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerDisplay::MarkerDisplay()
  : Display()
  , marker_topic_("visualization_marker")
  , hidden_ (false)
{
}

void MarkerDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<visualization_msgs::Marker>(*vis_manager_->getTFClient(), "", 100, update_nh_);
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  tf_filter_->connectInput(sub_);
  tf_filter_->registerCallback(boost::bind(&MarkerDisplay::incomingMarker, this, _1));
  tf_filter_->registerFailureCallback(boost::bind(&MarkerDisplay::failedMarker, this, _1, _2));
}

MarkerDisplay::~MarkerDisplay()
{
  unsubscribe();

  clearMarkers();

  delete tf_filter_;
}

MarkerBasePtr MarkerDisplay::getMarker(MarkerID id)
{
  M_IDToMarker::iterator it = markers_.find(id);
  if (it != markers_.end())
  {
    return it->second;
  }

  return MarkerBasePtr();
}

void MarkerDisplay::clearMarkers()
{
  markers_.clear();
  markers_with_expiration_.clear();
  frame_locked_markers_.clear();
  tf_filter_->clear();

  if (property_manager_)
  {
    M_Namespace::iterator it = namespaces_.begin();
    M_Namespace::iterator end = namespaces_.end();
    for (; it != end; ++it)
    {
      property_manager_->deleteProperty(it->second.prop.lock());
    }
  }

  namespaces_.clear();
}

void MarkerDisplay::onEnable()
{
  subscribe();
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MarkerDisplay::onDisable()
{
  unsubscribe();
  tf_filter_->clear();
  clearMarkers();
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MarkerDisplay::hideVisible()
{
  hidden_ = true;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MarkerDisplay::restoreVisible()
{
  hidden_ = false;
  scene_node_->setVisible( enabled_ && !hidden_ );
}

void MarkerDisplay::setQueueSize( int size )
{
  if( size != (int) tf_filter_->getQueueSize() )
  {
    tf_filter_->setQueueSize( (uint32_t) size );
    propertyChanged( queue_size_property_ );
  }
}

int MarkerDisplay::getQueueSize()
{
  return (int) tf_filter_->getQueueSize();
}

void MarkerDisplay::setMarkerTopic(const std::string& topic)
{
  unsubscribe();
  marker_topic_ = topic;
  subscribe();

  propertyChanged(marker_topic_property_);
}

void MarkerDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if (!marker_topic_.empty())
  {
    array_sub_.shutdown();
    sub_.unsubscribe();

    try
    {
      sub_.subscribe(update_nh_, marker_topic_, 1000);
      array_sub_ = update_nh_.subscribe(marker_topic_ + "_array", 1000, &MarkerDisplay::incomingMarkerArray, this);
      setStatus(status_levels::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(status_levels::Error, "Topic", std::string("Error subscribing: ") + e.what());
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
  deleteMarkerStatus(id);

  M_IDToMarker::iterator it = markers_.find(id);
  if (it != markers_.end())
  {
    markers_with_expiration_.erase(it->second);
    frame_locked_markers_.erase(it->second);
    markers_.erase(it);
  }
}

void MarkerDisplay::setNamespaceEnabled(const std::string& ns, bool enabled)
{
  M_Namespace::iterator it = namespaces_.find(ns);
  if (it != namespaces_.end())
  {
    it->second.enabled = enabled;

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
}

bool MarkerDisplay::isNamespaceEnabled(const std::string& ns)
{
  M_Namespace::iterator it = namespaces_.find(ns);
  if (it != namespaces_.end())
  {
    return it->second.enabled;
  }

  return true;
}

void MarkerDisplay::setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text)
{
  UniformStringStream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  setStatus(level, marker_name, text);
}

void MarkerDisplay::deleteMarkerStatus(MarkerID id)
{
  UniformStringStream ss;
  ss << id.first << "/" << id.second;
  std::string marker_name = ss.str();
  deleteStatus(marker_name);
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

void MarkerDisplay::failedMarker(const visualization_msgs::Marker::ConstPtr& marker, tf::FilterFailureReason reason)
{
  std::string error = FrameManager::instance()->discoverFailureReason(marker->header.frame_id, marker->header.stamp, marker->__connection_header ? (*marker->__connection_header)["callerid"] : "unknown", reason);
  setMarkerStatus(MarkerID(marker->ns, marker->id), status_levels::Error, error);
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
    setMarkerStatus(MarkerID(message->ns, message->id), status_levels::Error, "Contains invalid floating point values (nans or infs)");
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

  default:
    ROS_ERROR( "Unknown marker action: %d\n", message->action );
  }
}

void MarkerDisplay::processAdd( const visualization_msgs::Marker::ConstPtr& message )
{
  //
  M_Namespace::iterator ns_it = namespaces_.find(message->ns);
  if (ns_it == namespaces_.end())
  {
    Namespace ns;
    ns.name = message->ns;
    ns.enabled = true;

    if (property_manager_)
    {
      ns.prop = property_manager_->createProperty<BoolProperty>(ns.name, property_prefix_, boost::bind(&MarkerDisplay::isNamespaceEnabled, this, ns.name),
                                                                              boost::bind(&MarkerDisplay::setNamespaceEnabled, this, ns.name, _1), namespaces_category_, this);
      setPropertyHelpText(ns.prop, "Enable/disable all markers in this namespace.");
    }

    ns_it = namespaces_.insert(std::make_pair(ns.name, ns)).first;
  }

  if (!ns_it->second.enabled)
  {
    return;
  }

  deleteMarkerStatus(MarkerID(message->ns, message->id));

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
        marker.reset(new ShapeMarker(this, vis_manager_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::ARROW:
      {
        marker.reset(new ArrowMarker(this, vis_manager_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::LINE_STRIP:
      {
        marker.reset(new LineStripMarker(this, vis_manager_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::LINE_LIST:
      {
        marker.reset(new LineListMarker(this, vis_manager_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::SPHERE_LIST:
    case visualization_msgs::Marker::CUBE_LIST:
    case visualization_msgs::Marker::POINTS:
      {
        marker.reset(new PointsMarker(this, vis_manager_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        marker.reset(new TextViewFacingMarker(this, vis_manager_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      {
        marker.reset(new MeshResourceMarker(this, vis_manager_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::TRIANGLE_LIST:
    {
      marker.reset(new TriangleListMarker(this, vis_manager_, scene_node_));
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

    causeRender();
  }
}

void MarkerDisplay::processDelete( const visualization_msgs::Marker::ConstPtr& message )
{
  deleteMarker(MarkerID(message->ns, message->id));
  causeRender();
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
        S_MarkerBase::iterator copy = it;
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
  tf_filter_->setTargetFrame( fixed_frame_ );

  clearMarkers();
}

void MarkerDisplay::reset()
{
  Display::reset();
  clearMarkers();
}

void MarkerDisplay::createProperties()
{
  marker_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Marker Topic", property_prefix_, boost::bind( &MarkerDisplay::getMarkerTopic, this ),
                                                                                boost::bind( &MarkerDisplay::setMarkerTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(marker_topic_property_, "visualization_msgs::Marker topic to subscribe to.  <topic>_array will also automatically be subscribed with type visualization_msgs::MarkerArray.");
  ROSTopicStringPropertyPtr topic_prop = marker_topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<visualization_msgs::Marker>());

  queue_size_property_ = property_manager_->createProperty<IntProperty>( "Queue Size", property_prefix_,
                                                                         boost::bind( &MarkerDisplay::getQueueSize, this ),
                                                                         boost::bind( &MarkerDisplay::setQueueSize, this, _1 ),
                                                                         parent_category_, this );
  setPropertyHelpText( queue_size_property_, "Advanced: set the size of the incoming Marker message queue.  Increasing this is useful if your incoming TF data is delayed significantly from your Marker data, but it can greatly increase memory usage if the messages are big." );

  namespaces_category_ = property_manager_->createCategory("Namespaces", property_prefix_, parent_category_, this);
}

} // namespace rviz
