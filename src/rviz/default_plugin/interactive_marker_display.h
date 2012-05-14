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

#ifndef RVIZ_INTERACTIVE_MARKER_DISPLAY_H
#define RVIZ_INTERACTIVE_MARKER_DISPLAY_H

#include "rviz/default_plugin/interactive_markers/interactive_marker.h"
#include "rviz/default_plugin/interactive_markers/interactive_marker_client.h"

#include "rviz/display.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/forwards.h"

#include <map>
#include <set>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

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
 * \class InteractiveMarkerDisplay
 * \brief Displays "markers" sent in by other ROS nodes on the "visualization_marker" topic
 *
 * Markers come in as visualization_msgs::Marker messages.  See the Marker message for more information.
 */
class InteractiveMarkerDisplay : public Display, public InteractiveMarkerReceiver
{
public:
  InteractiveMarkerDisplay();
  virtual ~InteractiveMarkerDisplay();

  virtual void onInitialize();

  virtual void update(float wall_dt, float ros_dt);

  virtual void fixedFrameChanged();
  virtual void reset();

  void setMarkerUpdateTopic(const std::string& topic);
  const std::string& getMarkerUpdateTopic() { return marker_update_topic_; }

  virtual void createProperties();

  bool getShowDescriptions() { return show_descriptions_; }
  void setShowDescriptions( bool show );

  bool getShowToolTips() { return show_tool_tips_; }
  void setShowToolTips( bool show );

  bool getShowAxes() { return show_axes_; }
  void setShowAxes( bool show );

  ///// InteractiveMarkerReceiver interface
  void setStatusOk(const std::string& name, const std::string& text);
  void setStatusWarn(const std::string& name, const std::string& text);
  void setStatusError(const std::string& name, const std::string& text);

  /**
   * Subscribe to just the init messages.
   * @return true on success, false on failure.
   */
  bool subscribeToInit();

  // Clear the marker display stuff from the visual scene.
  void clearMarkers();

  // Update the display's versions of the markers.
  void processMarkerChanges( const std::vector<visualization_msgs::InteractiveMarker>* markers = NULL,
                             const std::vector<visualization_msgs::InteractiveMarkerPose>* poses = NULL,
                             const std::vector<std::string>* erases = NULL );

  void unsubscribeFromInit();

  /** @brief Hides all visible parts of this display, so they do not show up when the scene is rendered. */
  virtual void hideVisible();

  /** @brief Restores the display to the state it was in before hideVisible() was called. */
  virtual void restoreVisible();

protected:

  virtual void onEnable();
  virtual void onDisable();

  // Subscribe to all message topics
  void subscribe();

  // Unsubscribe from all message topics
  void unsubscribe();

  // put the marker into the message queue where it can be read out by the main thread (in update())
  void tfMarkerSuccess(const visualization_msgs::InteractiveMarker::ConstPtr& marker);

  // message filter callback for failed marker transformation
  void tfMarkerFail(const visualization_msgs::InteractiveMarker::ConstPtr& marker, tf::FilterFailureReason reason);

  // put the pose update into the message queue where it can be read out by the main thread (in update())
  void tfPoseSuccess(const visualization_msgs::InteractiveMarkerPose::ConstPtr& marker_pose);

  // message filter callback for failed pose transformation
  void tfPoseFail(const visualization_msgs::InteractiveMarkerPose::ConstPtr& marker_pose, tf::FilterFailureReason reason);

  void updateMarker( visualization_msgs::InteractiveMarker::ConstPtr& marker );
  void updatePose( visualization_msgs::InteractiveMarkerPose::ConstPtr& pose );

  InteractiveMarkerClient im_client_;

  // Ogre objects
  Ogre::SceneNode* scene_node_;

  typedef boost::shared_ptr<InteractiveMarker> InteractiveMarkerPtr;
  typedef std::map< std::string, InteractiveMarkerPtr > M_StringToInteractiveMarkerPtr;
  M_StringToInteractiveMarkerPtr interactive_markers_;

  // Message interface

  tf::MessageFilter<visualization_msgs::InteractiveMarker>* tf_filter_;
  tf::MessageFilter<visualization_msgs::InteractiveMarkerPose>* tf_pose_filter_;

  ros::Subscriber marker_update_sub_;
  ros::Subscriber marker_init_sub_;

  // messages are placed here before being processed in update()
  typedef std::vector<visualization_msgs::InteractiveMarker::ConstPtr> V_InteractiveMarkerMessage;
  V_InteractiveMarkerMessage marker_queue_;
  typedef std::vector<visualization_msgs::InteractiveMarkerPose::ConstPtr> V_InteractiveMarkerPoseMessage;
  V_InteractiveMarkerPoseMessage pose_queue_;
  boost::mutex queue_mutex_;

  unsigned num_publishers_;

  std::string client_id_;

  // Properties

  std::string marker_update_topic_;
  ROSTopicStringPropertyWPtr marker_update_topic_property_;

  bool show_descriptions_;
  BoolPropertyWPtr show_descriptions_property_;

  bool show_tool_tips_;
  BoolPropertyWPtr show_tool_tips_property_;

  bool show_axes_;
  BoolPropertyWPtr show_axes_property_;

};

} // namespace rviz

#endif /* RVIZ_MARKER_DISPLAY_H */
