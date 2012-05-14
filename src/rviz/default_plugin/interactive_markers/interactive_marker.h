/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef RVIZ_INTERACTIVE_MARKER_H_
#define RVIZ_INTERACTIVE_MARKER_H_

#include "rviz/selection/forwards.h"

#include "interactive_marker_control.h"

#include <rviz/ogre_helpers/axes.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <geometry_msgs/Pose.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/shared_ptr.hpp>

#include <ros/publisher.h>

namespace Ogre {
class SceneNode;
}

class QMenu;

namespace rviz
{
class VisualizationManager;
class InteractiveMarkerDisplay;

class InteractiveMarker : public QObject
{
Q_OBJECT
public:
  InteractiveMarker( InteractiveMarkerDisplay *owner, VisualizationManager *vis_manager, std::string topic_ns, std::string client_id );
  virtual ~InteractiveMarker();

  // reset contents to reflect the data from a new message
  // @return success
  bool processMessage( visualization_msgs::InteractiveMarkerConstPtr message );

  // reset contents to reflect the data from a new message
  // @return success
  void processMessage( visualization_msgs::InteractiveMarkerPoseConstPtr message );

  // called every frame update
  void update(float wall_dt);

  // set the pose of the parent frame, relative to the fixed frame
  void setReferencePose( Ogre::Vector3 position, Ogre::Quaternion orientation );

  // directly set the pose, relative to parent frame
  // if publish is set to true, publish the change
  void setPose( Ogre::Vector3 position, Ogre::Quaternion orientation, const std::string &control_name );

  void translate( Ogre::Vector3 delta_position, const std::string &control_name );
  void rotate( Ogre::Quaternion delta_orientation, const std::string &control_name );

  // schedule a pose reset once dragging is finished
  void requestPoseUpdate( Ogre::Vector3 position, Ogre::Quaternion orientation );

  void startDragging();
  void stopDragging();

  const Ogre::Vector3& getPosition() { return position_; }
  const Ogre::Quaternion& getOrientation() { return orientation_; }

  float getSize() { return scale_; }
  const std::string &getReferenceFrame() { return reference_frame_; }
  const std::string& getName() { return name_; }

  // show name above marker
  void setShowDescription( bool show );

  // show axes in origin
  void setShowAxes( bool show );

  // @return true if the mouse event was intercepted, false if it was ignored
  bool handleMouseEvent(ViewportMouseEvent& event, const std::string &control_name );

  // fill in current marker pose & name, publish
  void publishFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback,
                       bool mouse_point_valid = false,
                       const Ogre::Vector3& mouse_point_rel_world = Ogre::Vector3(0,0,0) );

  void hideVisible();

  void restoreVisible();

protected Q_SLOTS:
  void handleMenuSelect( int menu_item_id );

protected:

  void publishPose();

  void reset();

  void updateReferencePose();

  QString makeMenuString( const std::string &entry );

  // Recursively append menu and submenu entries to menu, based on a
  // vector of menu entry id numbers describing the menu entries at the
  // current level.
  void populateMenu( QMenu* menu, std::vector<uint32_t>& ids );

  InteractiveMarkerDisplay *owner_;
  VisualizationManager *vis_manager_;

  // pose of parent coordinate frame
  std::string reference_frame_;
  ros::Time reference_time_;
  bool frame_locked_;

  // node representing reference frame in tf, like /map, /base_link, /head, etc.
  Ogre::SceneNode *reference_node_;

  // pose being controlled, relative to reference frame
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  // has the pose changed since the last feedback was sent?
  bool pose_changed_;
  double time_since_last_feedback_;

  typedef boost::shared_ptr<InteractiveMarkerControl> InteractiveMarkerControlPtr;
  typedef std::map<std::string, InteractiveMarkerControlPtr> M_ControlPtr;
  M_ControlPtr controls_;

  std::string name_;
  std::string description_;

  bool dragging_;
  std::string old_target_frame_;

  // pose being controlled
  bool pose_update_requested_;
  Ogre::Vector3 requested_position_;
  Ogre::Quaternion requested_orientation_;

  float scale_;

  boost::shared_ptr<QMenu> menu_;

  // Helper to more simply represent the menu tree.
  struct MenuNode
  {
    visualization_msgs::MenuEntry entry;
    std::vector<uint32_t> child_ids;
  };

  // maps menu index to menu entry and item
  std::map< uint32_t, MenuNode > menu_entries_;

  // Helper to store the top level of the menu tree.
  std::vector<uint32_t> top_level_menu_ids_;

  // which control has popped up the menu
  std::string last_control_name_;

  double heart_beat_t_;

  // visual aids

  Axes *axes_;
  Ogre::SceneNode *axes_node_;

  InteractiveMarkerControlPtr description_control_;

  ros::Publisher feedback_pub_;
  std::string topic_ns_;
  std::string client_id_;

  boost::recursive_mutex mutex_;

  boost::shared_ptr< boost::thread > sys_thread_;

  bool got_3d_point_for_menu_;
  Ogre::Vector3 three_d_point_for_menu_;
};


}

#endif /* INTERACTIVE_MARKER_H_ */
