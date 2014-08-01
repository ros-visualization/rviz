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

#ifndef RVIZ_MARKER_DISPLAY_H
#define RVIZ_MARKER_DISPLAY_H

#include <map>
#include <set>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#ifndef Q_MOC_RUN
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#endif

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "rviz/display.h"
#include "rviz/properties/bool_property.h"
#include "rviz/selection/forwards.h"

namespace rviz
{
class IntProperty;
class MarkerBase;
class MarkerNamespace;
class MarkerSelectionHandler;
class Object;
class RosTopicProperty;

typedef boost::shared_ptr<MarkerSelectionHandler> MarkerSelectionHandlerPtr;
typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;
typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class MarkerDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as visualization_msgs::Marker messages.  See the Marker message for more information.
 */
class MarkerDisplay: public Display
{
Q_OBJECT
public:
  MarkerDisplay();
  virtual ~MarkerDisplay();

  virtual void onInitialize();

  virtual void update(float wall_dt, float ros_dt);

  virtual void fixedFrameChanged();
  virtual void reset();

  void deleteMarker(MarkerID id);

  void setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text);
  void deleteMarkerStatus(MarkerID id);

  virtual void setTopic( const QString &topic, const QString &datatype );

protected:
  virtual void onEnable();
  virtual void onDisable();

  /** @brief Subscribes to the "visualization_marker" and
   * "visualization_marker_array" topics. */
  virtual void subscribe();

  /** @brief Unsubscribes from the "visualization_marker"
   * "visualization_marker_array" topics. */
  virtual void unsubscribe();

  /** @brief Process a MarkerArray message. */
  void incomingMarkerArray( const visualization_msgs::MarkerArray::ConstPtr& array );

  ros::Subscriber array_sub_;

  RosTopicProperty* marker_topic_property_;
  IntProperty* queue_size_property_;

private Q_SLOTS:
  void updateQueueSize();
  void updateTopic();

private:
  /** @brief Delete all the markers within the given namespace. */
  void deleteMarkersInNamespace( const std::string& ns );

  /**
   * \brief Removes all the markers
   */
  void clearMarkers();

  /**
   * \brief Processes a marker message
   * @param message The message to process
   */
  void processMessage( const visualization_msgs::Marker::ConstPtr& message );
  /**
   * \brief Processes an "Add" marker message
   * @param message The message to process
   */
  void processAdd( const visualization_msgs::Marker::ConstPtr& message );
  /**
   * \brief Processes a "Delete" marker message
   * @param message The message to process
   */
  void processDelete( const visualization_msgs::Marker::ConstPtr& message );

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker(const visualization_msgs::Marker::ConstPtr& marker);

  void failedMarker(const ros::MessageEvent<visualization_msgs::Marker>& marker_evt, tf::FilterFailureReason reason);

  typedef std::map<MarkerID, MarkerBasePtr> M_IDToMarker;
  typedef std::set<MarkerBasePtr> S_MarkerBase;
  M_IDToMarker markers_;                                ///< Map of marker id to the marker info structure
  S_MarkerBase markers_with_expiration_;
  S_MarkerBase frame_locked_markers_;
  typedef std::vector<visualization_msgs::Marker::ConstPtr> V_MarkerMessage;
  V_MarkerMessage message_queue_;                       ///< Marker message queue.  Messages are added to this as they are received, and then processed
                                                        ///< in our update() function
  boost::mutex queue_mutex_;

  message_filters::Subscriber<visualization_msgs::Marker> sub_;
  tf::MessageFilter<visualization_msgs::Marker>* tf_filter_;

  typedef QHash<QString, MarkerNamespace*> M_Namespace;
  M_Namespace namespaces_;

  Property* namespaces_category_;

  friend class MarkerNamespace;
};

/** @brief Manager of a single marker namespace.  Keeps a hash from
 * marker IDs to MarkerBasePtr, and creates or destroys them when . */
class MarkerNamespace: public BoolProperty
{
Q_OBJECT
public:
  MarkerNamespace( const QString& name, Property* parent_property, MarkerDisplay* owner );
  bool isEnabled() const { return getBool(); }

public Q_SLOTS:
  void onEnableChanged();

private:
  MarkerDisplay* owner_;
};

} // namespace rviz

#endif /* RVIZ_MARKER_DISPLAY_H */
