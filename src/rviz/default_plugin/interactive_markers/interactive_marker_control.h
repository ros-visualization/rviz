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

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <visualization_msgs/InteractiveMarkerControl.h>

#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/selection/forwards.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/interactive_object.h"

#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneManager.h>

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
class VisualizationManager;
class InteractiveMarker;
class PointsMarker;

/**
 * A single control element of an InteractiveMarker.
 */
class InteractiveMarkerControl: public Ogre::SceneManager::Listener,
                                public InteractiveObject,
                                public boost::enable_shared_from_this<InteractiveMarkerControl>
{
public:
  /** @brief Constructor.
   *
   * Just creates Ogre::SceneNodes and sets some defaults.  To
   * actually make it look like a
   * visualization_msgs::InteractiveMarkerControl message specifies,
   * call processMessage().
   */
  InteractiveMarkerControl(VisualizationManager* vis_manager,
                           Ogre::SceneNode *reference_node,
                           InteractiveMarker *parent );

  virtual ~InteractiveMarkerControl();

  /** @brief Set up or update the contents of this control to match the
   *         specification in the message. */
  void processMessage( const visualization_msgs::InteractiveMarkerControl &message );

  // called when interactive mode is globally switched on/off
  virtual void enableInteraction(bool enable);

  // will receive all mouse events while the handler has focus
  virtual void handleMouseEvent(ViewportMouseEvent& event);

  /** Update the pose of the interactive marker being controlled,
   * relative to the reference frame.  Each InteractiveMarkerControl
   * maintains its pose relative to the reference frame independently,
   * so when the parent InteractiveMarker mvoes, it calls this
   * function on all its child controls. */
  void interactiveMarkerPoseChanged( Ogre::Vector3 int_marker_position, Ogre::Quaternion int_marker_orientation );

  bool isInteractive() { return interaction_mode_ != visualization_msgs::InteractiveMarkerControl::NONE; }

  // Called every frame by parent's update() function.
  void update();

  void setVisible( bool visible );

  void hideVisible();

  void restoreVisible();

protected:

  // when this is called, we will face the camera
  virtual void preFindVisibleObjects(Ogre::SceneManager *source, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport *v);

  void updateControlOrientationForViewFacing( Ogre::Viewport* v );

  /** Rotate the pose, following the mouse movement.  mouse_ray is
   * relative to the reference frame. */
  void rotate(Ogre::Ray &mouse_ray);

  /** Rotate and translate to follow the mouse movement.  mouse_ray is
   * relative to the reference frame. */
  void moveRotate( Ogre::Ray &mouse_ray );

  /** Translate, following the mouse movement. */
  void movePlane(Ogre::Ray &mouse_ray);

  // Move the position along the control ray given the latest mouse ray.
  void moveAxis( const Ogre::Ray& mouse_ray, const ViewportMouseEvent& event );

  /// compute intersection between mouse ray and y-z plane given in local coordinates
  bool intersectYzPlane( const Ogre::Ray& mouse_ray,
                         Ogre::Vector3& intersection_3d,
                         Ogre::Vector2& intersection_2d,
                         float& ray_t );

  /// compute intersection between mouse ray and a y-z plane.
  bool intersectSomeYzPlane( const Ogre::Ray& mouse_ray,
                             const Ogre::Vector3& point_in_plane,
                             const Ogre::Quaternion& plane_orientation,
                             Ogre::Vector3& intersection_3d,
                             Ogre::Vector2& intersection_2d,
                             float& ray_t );

  /** Find the closest point on target_ray to mouse_ray.
   * @param closest_point contains result point on target_ray if rays are not effectively parallel.
   * @returns false if rays are effectively parallel, true otherwise. */
  bool findClosestPoint( const Ogre::Ray& target_ray,
                         const Ogre::Ray& mouse_ray,
                         Ogre::Vector3& closest_point );

  /** Project a reference position onto the viewport to find screen coordinates in pixels.
   * @param screen_pos the resultant screen position, in pixels. */
  void worldToScreen( const Ogre::Vector3& pos_rel_reference,
                      const Ogre::Viewport* viewport,
                      Ogre::Vector2& screen_pos );

  /// take all the materials, add a highlight pass and store a pointer to the pass for later use
  void addHighlightPass( S_MaterialPtr materials );

  // set the highlight color to (a,a,a)
  void setHighlight( float a );

  // Save a copy of the latest mouse event with the event type set to
  // QEvent::MouseMove, so that update() can resend the mouse event during
  // drag actions to maintain consistent behavior.
  void recordDraggingInPlaceEvent( ViewportMouseEvent& event );

  // Motion part of mouse event handling.
  void handleMouseMovement( ViewportMouseEvent& event );

  // Return closest point on a line to a test point.
  Ogre::Vector3 closestPointOnLineToPoint( const Ogre::Vector3& line_start,
                                           const Ogre::Vector3& line_dir,
                                           const Ogre::Vector3& test_point );

  /** @brief Create marker objects from the message and add them to the internal marker arrays. */
  void makeMarkers( const visualization_msgs::InteractiveMarkerControl &message );

  bool dragging_;
  Ogre::Viewport* drag_viewport_;

  ViewportMouseEvent dragging_in_place_event_;

  VisualizationManager* vis_manager_;

  CollObjectHandle coll_object_handle_;

  /** Node representing reference frame in tf, like /map, /base_link,
   * /head, etc.  Same as the field in InteractiveMarker. */
  Ogre::SceneNode *reference_node_;

  /** Represents the local frame of this control relative to reference
   * node/frame.  There is no intermediate InteractiveMarker node or
   * frame, each control keeps track of its pose relative to the
   * reference frame independently.  In INHERIT mode, this will have
   * an identical pose as the rest of the interactive marker,
   * otherwise its orientation might be different. */
  Ogre::SceneNode *control_frame_node_;

  // this is a child of scene_node, but might be oriented differently
  Ogre::SceneNode *markers_node_;

  // interaction mode
  int interaction_mode_;
  int orientation_mode_;

  // if in view facing mode, the markers should be
  // view facing as well
  // if set to false, they will follow the parent's transformations
  bool independent_marker_orientation_;

  /** Defines the axis / plane along which to transform.  This is not
   * keeping track of rotations applied to the control by the user,
   * this is just a copy of the "orientation" parameter from the
   * InteractiveMarkerControl message. */
  Ogre::Quaternion control_orientation_;

  bool always_visible_;

  std::string description_;

  std::string name_;

  typedef boost::shared_ptr<MarkerBase> MarkerBasePtr;
  std::vector< MarkerBasePtr > markers_;

  InteractiveMarker *parent_;

  std::set<Ogre::Pass*> highlight_passes_;

  // PointsMarkers are rendered by special shader programs, so the
  // regular highlighting method does not work for them.  Keep a
  // vector of them so we can call their setHighlightColor() function.
  typedef boost::shared_ptr<PointsMarker> PointsMarkerPtr;
  std::vector< PointsMarkerPtr > points_markers_;

  /** Stores the rotation around the x axis of the control.  Only
   * relevant for fixed-orientation rotation controls. */
  Ogre::Radian rotation_;

  /** Stores the rotation around the x axis of the control as it was
   * when the mouse-down event happened.  Only relevant for
   * fixed-orientation rotation controls. */
  Ogre::Radian rotation_at_mouse_down_;

  /** The 3D position of the mouse click when the mouse button is
   * pressed, relative to the reference frame. */
  Ogre::Vector3 grab_point_;

  // The 2D position in pixel coordinates of the mouse-down location.
  Ogre::Vector2 grab_pixel_;
  // The position of the parent when the mouse button is pressed.
  Ogre::Vector3 parent_position_at_mouse_down_;

  /** The orientation of the control_frame_node_ when the mouse button
   * is pressed. */
  Ogre::Quaternion control_frame_orientation_at_mouse_down_;

  /** The orientation of the parent when the mouse button
   * is pressed. */
  Ogre::Quaternion parent_orientation_at_mouse_down_;

  /** The direction vector of the axis of rotation during a mouse
   * drag, relative to the reference frame.  Computed on mouse down
   * event. */
  Ogre::Vector3 rotation_axis_;

  /** The center of rotation during a mouse drag, relative to the
   * control frame.  Computed on mouse down event. */
  Ogre::Vector3 rotation_center_rel_control_;

  /** The grab point during a mouse drag, relative to the control
   * frame.  Computed on mouse down event. */
  Ogre::Vector3 grab_point_rel_control_;

  bool has_focus_;
  bool interaction_enabled_;

  bool visible_;
  bool saved_visibility_state_;
  bool view_facing_;
};

}

#endif /* INTERACTIVE_MARKER_CONTROL_H_ */
