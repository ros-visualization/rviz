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

#include <map>
#include <set>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <interactive_markers/interactive_marker_client.h>
#endif

#include "rviz/display.h"
#include "rviz/selection/forwards.h"

#include "rviz/default_plugin/interactive_markers/interactive_marker.h"

namespace rviz
{
class BoolProperty;
class Object;
class RosTopicProperty;
class MarkerBase;

typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;
typedef std::pair<std::string, int32_t> MarkerID;

/**
 * \class InteractiveMarkerDisplay
 * \brief Displays Interactive Markers
 */
class InteractiveMarkerDisplay : public Display
{
Q_OBJECT
public:
  InteractiveMarkerDisplay();

  virtual void onInitialize();

  virtual void update(float wall_dt, float ros_dt);

  virtual void fixedFrameChanged();

  virtual void reset();

  virtual void setTopic( const QString &topic, const QString &datatype );

protected:
  virtual void onEnable();
  virtual void onDisable();

protected Q_SLOTS:
  void updateTopic();
  void updateShowDescriptions();
  void updateShowAxes();
  void updateShowVisualAids();
  void updateEnableTransparency();
  void publishFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void onStatusUpdate( StatusProperty::Level level, const std::string& name, const std::string& text );

private:

  // Subscribe to all message topics
  void subscribe();

  // Unsubscribe from all message topics
  void unsubscribe();

  void initCb( visualization_msgs::InteractiveMarkerInitConstPtr msg );
  void updateCb( visualization_msgs::InteractiveMarkerUpdateConstPtr msg );

  void resetCb( std::string server_id );

  void statusCb( interactive_markers::InteractiveMarkerClient::StatusT,
      const std::string& server_id,
      const std::string& msg );

  void updateMarkers(
      const std::string& server_id,
      const std::vector<visualization_msgs::InteractiveMarker>& markers );

  void updatePoses(
      const std::string& server_id,
      const std::vector<visualization_msgs::InteractiveMarkerPose>& marker_poses );

  void eraseMarkers(
      const std::string& server_id,
      const std::vector<std::string>& names );

  // Update the display's versions of the markers.
  void processMarkerChanges( const std::vector<visualization_msgs::InteractiveMarker>* markers = NULL,
                             const std::vector<visualization_msgs::InteractiveMarkerPose>* poses = NULL,
                             const std::vector<std::string>* erases = NULL );

  typedef boost::shared_ptr<InteractiveMarker> IMPtr;
  typedef std::map< std::string, IMPtr > M_StringToIMPtr;
  typedef std::map< std::string, M_StringToIMPtr > M_StringToStringToIMPtr;
  M_StringToStringToIMPtr interactive_markers_;

  M_StringToIMPtr& getImMap( std::string server_id );

  std::string client_id_;

  // Properties
  RosTopicProperty* marker_update_topic_property_;
  BoolProperty* show_descriptions_property_;
  BoolProperty* show_axes_property_;
  BoolProperty* show_visual_aids_property_;
  BoolProperty* enable_transparency_property_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerClient> im_client_;

  ros::Publisher feedback_pub_;

  std::string topic_ns_;
};

} // namespace rviz

#endif /* RVIZ_MARKER_DISPLAY_H */
