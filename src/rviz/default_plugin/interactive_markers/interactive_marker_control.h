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


#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <OgreRay.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#endif

#include <QCursor>

#include <visualization_msgs/InteractiveMarkerControl.h>

#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/selection/forwards.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/interactive_object.h"

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
class DisplayContext;
class InteractiveMarker;
class PointsMarker;
class Line;

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
  InteractiveMarkerControl( DisplayContext* context,
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

  /**
   * This is the main entry-point for interaction using a 3D cursor.
   * <p>
   * The ViewportMouseEvent struct is used to "fake" a mouse event.
   * An event must have the panel, viewport, and type members filled in.
   * The acting_button and buttons_down members can be specified as well, if appropriate.
   * All other fields are currently ignored.
   * <p>
   * A sample construction of a "right-button mouse-up" event:
   * @code
   * ViewportMouseEvent event;
   * event.panel = context_->getViewManager()->getRenderPanel();
   * event.viewport = context_->getViewManager()->getRenderPanel()->getRenderWindow()->getViewport(0);
   * event.type = QEvent::MouseButtonRelease;
   * event.acting_button = Qt::RightButton;
   * event.buttons_down = Qt::NoButton;
   * @endcode
   * <p>
   * For more examples, see the implementation in the interaction_cursor_rviz package.
   *
   * @param  event        A struct holding certain event data (see description above).
   * @param  cursor_pos   The world-relative position of the 3D cursor.
   * @param  cursor_rot   The world-relative orientation of the 3D cursor.
   * @param  control_name The name of the child InteractiveMarkerControl calling this function.
   */
  virtual void handle3DCursorEvent( ViewportMouseEvent event, const Ogre::Vector3& cursor_3D_pos, const Ogre::Quaternion& cursor_3D_orientation);

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

  bool getVisible();

  // Highlight types
  enum ControlHighlight { NO_HIGHLIGHT = 0,
         HOVER_HIGHLIGHT = 3,
         ACTIVE_HIGHLIGHT = 5};

  // Public access to highlight controls
  void setHighlight( const ControlHighlight &hl  );

  /**
   * @return pointer to the parent InteractiveMarker
   */
  InteractiveMarker* getParent() { return parent_ ;}

  /**
   * @return the name of this control
   */
  const std::string& getName() { return name_; }

  /**
   * @return the description for this control
   */
  const QString& getDescription() { return description_; }

  /**
   * @return the visualization_msgs::InteractiveMarkerControl interaction_mode for this control
   */
  int getInteractionMode() { return interaction_mode_; }

  /**
   * @return the visualization_msgs::InteractiveMarkerControl orientation_mode for this control
   */
  int getOrientationMode() { return orientation_mode_; }

  /**
   * @brief If true, will show some geometric helpers while dragging
   */
  void setShowVisualAids( bool show ) { show_visual_aids_ = show; }

protected:

  // when this is called, we will face the camera
  virtual void preFindVisibleObjects(Ogre::SceneManager *source, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport *v);

  void updateControlOrientationForViewFacing( Ogre::Viewport* v );

  /** calculate a mouse ray in the reference frame.
   *  A mouse ray is a ray starting at the camera and pointing towards the mouse position. */
  Ogre::Ray getMouseRayInReferenceFrame( const ViewportMouseEvent& event, int x, int y );

  /** begin a relative-motion drag. */
  void beginRelativeMouseMotion( const ViewportMouseEvent& event );

  /** get the relative motion of the mouse, and put the mouse back
   *  where it was when beginRelativeMouseMotion() was called. */
  bool getRelativeMouseMotion( const ViewportMouseEvent& event, int& dx, int& dy );

  /** Rotate the pose around the camera-frame XY (right/up) axes, based on relative mouse movement. */
  void rotateXYRelative( const ViewportMouseEvent& event );

  /** Rotate the pose around the camera-frame Z (look) axis, based on relative mouse movement. */
  void rotateZRelative( const ViewportMouseEvent& event );

  /** Move the pose along the mouse ray, based on relative mouse movement. */
  void moveZAxisRelative( const ViewportMouseEvent& event );

  /** Move the pose along the mouse ray, based on mouse wheel movement. */
  void moveZAxisWheel( const ViewportMouseEvent& event );

  /** Move the pose around the XY view plane (perpendicular to the camera direction). */
  void moveViewPlane( Ogre::Ray &mouse_ray, const ViewportMouseEvent& event );

  /** Rotate the pose around the local X-axis, following the mouse movement.
   *  mouse_ray is relative to the reference frame. */
  void rotate( Ogre::Ray &mouse_ray );

  /** Rotate the pose around the local X axis, following the 3D cursor movement. */
  void rotate(const Ogre::Vector3& cursor_position_in_reference_frame);

  /** Rotate about, and translate perpendicular to, the local X-axis, following the mouse movement.
   *  mouse_ray is relative to the reference frame. */
  void moveRotate( Ogre::Ray &mouse_ray );

  /** Rotate about, and translate perpendicular to, the local X-axis, following the 3D cursor movement. */
  void moveRotate( const Ogre::Vector3& cursor_position_in_reference_frame, bool lock_axis = true);

  /** Translate in the plane perpendicular to the local X-axis, following the mouse movement.
   *  mouse_ray is relative to the reference frame. */
  void movePlane( Ogre::Ray &mouse_ray );

  /** Translate in the plane perpendicular to the local X-axis, following the 3D cursor movement. */
  void movePlane( const Ogre::Vector3& cursor_position_in_reference_frame );

  /** Translate along the local X-axis, following the mouse movement.
   *  mouse_ray is relative to the reference frame. */
  void moveAxis( const Ogre::Ray& mouse_ray, const ViewportMouseEvent& event );

  /** Translate along the local X-axis, following the 3D cursor movement. */
  void moveAxis( const Ogre::Vector3& cursor_position_in_reference_frame );

  /** Translate in 3-degrees-of-freedom, following the 3D cursor translation. */
  void move3D( const Ogre::Vector3& cursor_position_in_reference_frame,
               const Ogre::Quaternion &cursor_orientation_in_reference_frame );

  /** Rotate in 3-degrees-of-freedom, following the 3D cursor rotation. */
  void rotate3D( const Ogre::Vector3& cursor_position_in_reference_frame,
                 const Ogre::Quaternion &cursor_orientation_in_reference_frame );

  /** Rotate and translate in full 6-DOF, following the 3D cursor movement. */
  void moveRotate3D( const Ogre::Vector3& cursor_position_in_reference_frame,
                     const Ogre::Quaternion& cursor_orientation_in_reference_frame );

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

  // Begin a new mouse motion.  Called when left button is pressed to begin a drag.
  void beginMouseMovement( ViewportMouseEvent& event, bool line_visible );

  // Motion part of mouse event handling.
  void handleMouseMovement( ViewportMouseEvent& event );

  // Mouse wheel part of mouse event handling.
  void handleMouseWheelMovement( ViewportMouseEvent& event );

  // Return closest point on a line to a test point.
  Ogre::Vector3 closestPointOnLineToPoint( const Ogre::Vector3& line_start,
                                           const Ogre::Vector3& line_dir,
                                           const Ogre::Vector3& test_point );

  /** @brief Create marker objects from the message and add them to the internal marker arrays. */
  void makeMarkers( const visualization_msgs::InteractiveMarkerControl &message );

  void stopDragging( bool force = false );

  virtual const QCursor& getCursor() const { return cursor_; }

  bool mouse_dragging_;
  Ogre::Viewport* drag_viewport_;

  ViewportMouseEvent dragging_in_place_event_;

  DisplayContext* context_;

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

  QString description_;

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

  /** The 3D position of the mouse click/cursor when the 'grab' button is
   * pressed, relative to the reference frame. */
  Ogre::Vector3 grab_point_in_reference_frame_;

  /** The orientation of the cursor when the 'grab' button is
   * pressed, relative to the reference frame. */
  Ogre::Quaternion grab_orientation_in_reference_frame_;

  /** Records the 3D position of the cursor relative to the parent marker,
   *  expressed in the cursor frame, when the 'grab' button is pressed. */
  Ogre::Vector3 parent_to_cursor_in_cursor_frame_at_grab_;

  /** Records the rotation of the parent from the cursor frame,
   *  when the 'grab' button is pressed. */
  Ogre::Quaternion rotation_cursor_to_parent_at_grab_;

  /** The modifier state when drag begins. */
  Qt::KeyboardModifiers modifiers_at_drag_begin_;

  /** position of mouse when drag begins. */
  int mouse_x_at_drag_begin_;
  int mouse_y_at_drag_begin_;

  /* mouse ray when drag begins. */
  Ogre::Ray mouse_ray_at_drag_begin_;

  /* how far to move in Z when mouse moves 1 pixel. */
  double mouse_z_scale_;

  /** offset of the absolute mouse position from the relative mouse position */
  int mouse_relative_to_absolute_x_;
  int mouse_relative_to_absolute_y_;

  /** position of grab relative to parent in world coordinates. */
  Ogre::Vector3 parent_to_grab_position_; // obsolete now, but left for ABI compatibility

  /** The position of the parent when the mouse button is pressed. */
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
  bool view_facing_;

  QCursor cursor_;
  QString status_msg_;

  bool mouse_down_;

  boost::shared_ptr<Line> line_;

  bool show_visual_aids_;
};

}

#endif /* INTERACTIVE_MARKER_CONTROL_H_ */
