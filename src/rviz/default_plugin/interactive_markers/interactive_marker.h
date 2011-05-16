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

#include <ogre_tools/axes.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <geometry_msgs/Pose.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/shared_ptr.hpp>

namespace Ogre {
class SceneNode;
}

class wxMenu;

namespace rviz
{
class VisualizationManager;
class InteractiveMarkerDisplay;

class InteractiveMarker
{
public:
  InteractiveMarker( InteractiveMarkerDisplay *owner, VisualizationManager *vis_manager );
  virtual ~InteractiveMarker();

  // reset contents to reflect the data from a new message
  // @return success
  bool processMessage( visualization_msgs::InteractiveMarkerConstPtr message );

  // called every frame update
  void update(float wall_dt);

  // set the pose of the parent frame, relative to the fixed frame
  void setParentPose( Ogre::Vector3 position, Ogre::Quaternion orientation );

  // directly set the pose, relative to parent frame
  void setPose( Ogre::Vector3 position, Ogre::Quaternion orientation );
  void translate( Ogre::Vector3 delta_position );
  void rotate( Ogre::Quaternion delta_orientation );

  // schedule a pose reset once dragging is finished
  void requestPoseUpdate( Ogre::Vector3 position, Ogre::Quaternion orientation );

  void startDragging();
  void stopDragging();

  const Ogre::Vector3& getParentPosition() { return parent_position_; }
  const Ogre::Quaternion& getParentOrientation() { return parent_orientation_; }

  const Ogre::Vector3& getPosition() { return position_; }
  const Ogre::Quaternion& getOrientation() { return orientation_; }

  float getSize() { return size_; }

  // @return true if the mouse event was intercepted, false if it was ignored
  bool handleMouseEvent(ViewportMouseEvent& event);

protected:

  void reset();

  InteractiveMarkerDisplay *owner_;
  VisualizationManager *vis_manager_;

  typedef boost::shared_ptr<InteractiveMarkerControl> InteractiveMarkerControlPtr;
  std::list<InteractiveMarkerControlPtr> controls_;

  // pose of parent coordinate frame
  Ogre::Vector3 parent_position_;
  Ogre::Quaternion parent_orientation_;

  std::string name_;

  // pose being controlled
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  bool dragging_;

  // pose being controlled
  bool pose_update_requested_;
  Ogre::Vector3 requested_position_;
  Ogre::Quaternion requested_orientation_;

  // if true, always re-transform into parent frame
  bool frame_locked_;

  float size_;

  ogre_tools::Axes axes_;

  wxMenu* menu_;

  double heart_beat_t_;
};


}

#endif /* INTERACTIVE_MARKER_H_ */
