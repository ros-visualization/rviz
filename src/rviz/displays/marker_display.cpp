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
#include "visualization_manager.h"
#include "properties/property_manager.h"
#include "properties/property.h"
#include "common.h"
#include "selection/selection_manager.h"
#include "robot/robot.h"
#include "markers/shape_marker.h"
#include "markers/arrow_marker.h"
#include "markers/line_list_marker.h"
#include "markers/line_strip_marker.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

using visualization_msgs::Marker;
using visualization_msgs::MarkerPtr;

namespace rviz
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MarkerDisplay::MarkerDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  notifier_ = new tf::MessageNotifier<visualization_msgs::Marker>(tf_, ros_node_, boost::bind(&MarkerDisplay::incomingMarker, this, _1), "", "", 0);
}

MarkerDisplay::~MarkerDisplay()
{
  unsubscribe();

  delete notifier_;

  clearMarkers();
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
}

void MarkerDisplay::onEnable()
{
  subscribe();

  scene_node_->setVisible( true );
}

void MarkerDisplay::onDisable()
{
  unsubscribe();
  notifier_->clear();

  clearMarkers();

  scene_node_->setVisible( false );
}

void MarkerDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  notifier_->setTopic("visualization_marker");
  ros_node_->subscribe("visualization_marker_array", marker_array_, &MarkerDisplay::incomingMarkerArray, this, 1000);
}

void MarkerDisplay::unsubscribe()
{
  notifier_->setTopic("");
  ros_node_->unsubscribe("visualization_marker_array", &MarkerDisplay::incomingMarkerArray, this);
}

void MarkerDisplay::incomingMarkerArray()
{
  std::vector<visualization_msgs::Marker>::iterator it = marker_array_.markers.begin();
  std::vector<visualization_msgs::Marker>::iterator end = marker_array_.markers.end();
  for (; it != end; ++it)
  {
    const visualization_msgs::Marker& marker = *it;
    notifier_->enqueueMessage(visualization_msgs::MarkerPtr(new visualization_msgs::Marker(marker)));
  }
}

void MarkerDisplay::incomingMarker( const MarkerPtr& message )
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  message_queue_.push_back( message );
}

void MarkerDisplay::processMessage( const MarkerPtr& message )
{
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

void MarkerDisplay::processAdd( const MarkerPtr& message )
{
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
        marker.reset(new ShapeMarker(vis_manager_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::ARROW:
      {
        marker.reset(new ArrowMarker(vis_manager_, scene_node_));
      }
      break;

    case visualization_msgs::Marker::LINE_STRIP:
      {
        marker.reset(new LineStripMarker(vis_manager_, scene_node_));
      }
      break;
    case visualization_msgs::Marker::LINE_LIST:
      {
        marker.reset(new LineListMarker(vis_manager_, scene_node_));
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

    causeRender();
  }
}

void MarkerDisplay::processDelete( const MarkerPtr& message )
{
  M_IDToMarker::iterator it = markers_.find( MarkerID(message->ns, message->id) );
  if ( it != markers_.end() )
  {
    markers_.erase( it );
  }

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
      MarkerPtr& marker = *message_it;

      processMessage( marker );
    }
  }

  S_MarkerBase::iterator it = markers_with_expiration_.begin();
  S_MarkerBase::iterator end = markers_with_expiration_.end();
  for (; it != end;)
  {
    MarkerBasePtr marker = *it;
    if (marker->expired())
    {
      S_MarkerBase::iterator copy = it;
      ++it;
      markers_with_expiration_.erase(copy);
      markers_.erase(MarkerID(marker->getMessage()->ns, marker->getMessage()->id));
    }
    else
    {
      ++it;
    }
  }
}

void MarkerDisplay::targetFrameChanged()
{
}

void MarkerDisplay::fixedFrameChanged()
{
  notifier_->setTargetFrame( fixed_frame_ );

  clearMarkers();
}

void MarkerDisplay::reset()
{
  clearMarkers();
}

const char* MarkerDisplay::getDescription()
{
  return "Displays visualization markers sent over the visualization_marker and visualization_marker_array topics.";
}

} // namespace rviz
