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

#ifndef INTERACTIVE_MARKER_CONTROL_H_
#define INTERACTIVE_MARKER_CONTROL_H_

#include <visualization_msgs/InteractiveMarkerControl.h>

#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/selection/forwards.h"
#include "rviz/viewport_mouse_event.h"

#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>

#include <boost/shared_ptr.hpp>

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
class VisualizationManager;
class InteractiveMarker;

class InteractiveMarkerControl : public Ogre::SceneManager::Listener
{
public:

  InteractiveMarkerControl(VisualizationManager* vis_manager, const visualization_msgs::InteractiveMarkerControl &message, InteractiveMarker *parent );
  virtual ~InteractiveMarkerControl();

  // called when interactive mode is globally switched on/off
  virtual void enableInteraction(bool enable);

  // is called when the mouse passes over the object
  virtual void onReceiveFocus();

  // is called when the mouse leaves the object or when draggging is finished
  virtual void onLoseFocus();

  // will receive all mouse events while the handler has focus
  virtual void handleMouseEvent(ViewportMouseEvent& event);

  // set the pose of the parent frame, relative to the fixed frame
  void setParentPose( Ogre::Vector3 parent_position, Ogre::Quaternion parent_orientation );

  // set the pose of the frame being controlled
  void setPose( Ogre::Vector3 position, Ogre::Quaternion orientation );

  bool isInteractive() { return mode_ != visualization_msgs::InteractiveMarkerControl::NONE; }

protected:

  // when this is called, we will face the camera
  virtual void preFindVisibleObjects (Ogre::SceneManager *source, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport *v);

  // rotate the pose, following the mouse movement
  void rotate(Ogre::Ray &mouse_ray);

  // move the pose, following the mouse movement
  void movePlane(Ogre::Ray &mouse_ray );

  // move in plane so that the mouse stays within a max_dist radius to the center
  void followMouse(Ogre::Ray &mouse_ray, float max_dist );

  /// compute intersection between mouse ray and y-z plane given in local coordinates
  bool intersectYzPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t );

  /// get closest position on this control's axis
  bool getClosestPosOnAxis( Ogre::Ray mouse_ray, float &pos );

  /// take all the materials of the entity, add a highlight pass and store a pointer to the pass or later use
  void addHighlightPass( Ogre::Entity* entity );

  VisualizationManager* vis_manager_;

  CollObjectHandle coll_object_handle_;

  Ogre::SceneNode *scene_node_;

  bool dragging_;

  // when dragging, what was the last mouse ray (used to compute deltas)
  Ogre::Ray last_mouse_ray_;

  // interaction mode
  int mode_;

  // defines the axis / plane along which to transform
  Ogre::Vector3 x_axis_;
  Ogre::Vector3 y_axis_;
  Ogre::Vector3 z_axis_;

  bool fixed_orientation_;
  bool view_facing_;
  bool always_visible_;

  typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;
  std::vector< MarkerBasePtr > markers_;

  InteractiveMarker *parent_;

  std::set<Ogre::Pass*> highlight_passes_;
};

}

#endif /* INTERACTIVE_MARKER_CONTROL_H_ */
