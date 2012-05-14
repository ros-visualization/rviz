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

#include "rviz/display.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/forwards.h"

#include <map>
#include <set>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace rviz
{
class Object;
}

namespace rviz
{

class MarkerSelectionHandler;
typedef boost::shared_ptr<MarkerSelectionHandler> MarkerSelectionHandlerPtr;

class MarkerBase;
typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;

typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class MarkerDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as visualization_msgs::Marker messages.  See the Marker message for more information.
 */
class MarkerDisplay : public Display
{
public:
  MarkerDisplay();
  virtual ~MarkerDisplay();

  virtual void onInitialize();

  virtual void update(float wall_dt, float ros_dt);

  virtual void fixedFrameChanged();
  virtual void reset();

  void setMarkerTopic(const std::string& topic);
  const std::string& getMarkerTopic() { return marker_topic_; }

  virtual void createProperties();

  void setNamespaceEnabled(const std::string& ns, bool enabled);
  bool isNamespaceEnabled(const std::string& ns);

  void deleteMarker(MarkerID id);

  void setMarkerStatus(MarkerID id, StatusLevel level, const std::string& text);
  void deleteMarkerStatus(MarkerID id);

  /** Set the incoming message queue size. */
  void setQueueSize( int size );
  int getQueueSize();

  /** @brief Hides all visible parts of this display, so they do not show up when the scene is rendered. */
  virtual void hideVisible();

  /** @brief Restores the display to the state it was in before hideVisible() was called. */
  virtual void restoreVisible();

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Subscribes to the "visualization_marker" and "visualization_marker_array" topics
   */
  virtual void subscribe();
  /**
   * \brief Unsubscribes from the "visualization_marker" "visualization_marker_array" topics
   */
  virtual void unsubscribe();

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

  MarkerBasePtr getMarker(MarkerID id);

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker(const visualization_msgs::Marker::ConstPtr& marker);

  void incomingMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& array);

  void failedMarker(const visualization_msgs::Marker::ConstPtr& marker, tf::FilterFailureReason reason);

  typedef std::map<MarkerID, MarkerBasePtr> M_IDToMarker;
  typedef std::set<MarkerBasePtr> S_MarkerBase;
  M_IDToMarker markers_;                                ///< Map of marker id to the marker info structure
  S_MarkerBase markers_with_expiration_;
  S_MarkerBase frame_locked_markers_;
  typedef std::vector<visualization_msgs::Marker::ConstPtr> V_MarkerMessage;
  V_MarkerMessage message_queue_;                       ///< Marker message queue.  Messages are added to this as they are received, and then processed
                                                        ///< in our update() function
  boost::mutex queue_mutex_;

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  message_filters::Subscriber<visualization_msgs::Marker> sub_;
  tf::MessageFilter<visualization_msgs::Marker>* tf_filter_;
  ros::Subscriber array_sub_;

  std::string marker_topic_;

  struct Namespace
  {
    std::string name;
    bool enabled;
    BoolPropertyWPtr prop;
  };
  typedef std::map<std::string, Namespace> M_Namespace;
  M_Namespace namespaces_;

  ROSTopicStringPropertyWPtr marker_topic_property_;
  CategoryPropertyWPtr namespaces_category_;
  IntPropertyWPtr queue_size_property_;

  bool hidden_;
};

} // namespace rviz

#endif /* RVIZ_MARKER_DISPLAY_H */
