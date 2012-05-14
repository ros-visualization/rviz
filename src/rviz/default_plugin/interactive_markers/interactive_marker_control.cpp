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

#include "interactive_marker_control.h"

#include "rviz/default_plugin/markers/marker_base.h"
#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/render_panel.h"
#include "interactive_marker.h"

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgrePass.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>

#include "markers/shape_marker.h"
#include "markers/arrow_marker.h"
#include "markers/line_list_marker.h"
#include "markers/line_strip_marker.h"
#include "markers/points_marker.h"
#include "markers/text_view_facing_marker.h"
#include "markers/mesh_resource_marker.h"
#include "markers/triangle_list_marker.h"

#define ACTIVE_HIGHLIGHT 0.5
#define HOVER_HIGHLIGHT 0.3

namespace rviz
{

InteractiveMarkerControl::InteractiveMarkerControl( VisualizationManager* vis_manager,
                                                    Ogre::SceneNode *reference_node,
                                                    InteractiveMarker *parent )
: dragging_(false)
, drag_viewport_( NULL )
, vis_manager_(vis_manager)
, reference_node_(reference_node)
, control_frame_node_(reference_node_->createChildSceneNode())
, markers_node_(reference_node_->createChildSceneNode())
, parent_(parent)
, rotation_(0)
, grab_point_(0,0,0)
, interaction_enabled_(false)
, visible_(true)
, view_facing_( false )
{
}

void InteractiveMarkerControl::makeMarkers( const visualization_msgs::InteractiveMarkerControl& message )
{
  for (unsigned i = 0; i < message.markers.size(); i++)
  {
    MarkerBasePtr marker;

    // create a marker with the given type
    switch (message.markers[i].type)
    {
      case visualization_msgs::Marker::CUBE:
      case visualization_msgs::Marker::CYLINDER:
      case visualization_msgs::Marker::SPHERE:
      {
        marker.reset(new ShapeMarker(0, vis_manager_, markers_node_));
      }
        break;

      case visualization_msgs::Marker::ARROW:
      {
        marker.reset(new ArrowMarker(0, vis_manager_, markers_node_));
      }
        break;

      case visualization_msgs::Marker::LINE_STRIP:
      {
        marker.reset(new LineStripMarker(0, vis_manager_, markers_node_));
      }
        break;
      case visualization_msgs::Marker::LINE_LIST:
      {
        marker.reset(new LineListMarker(0, vis_manager_, markers_node_));
      }
        break;
      case visualization_msgs::Marker::SPHERE_LIST:
      case visualization_msgs::Marker::CUBE_LIST:
      case visualization_msgs::Marker::POINTS:
      {
        PointsMarkerPtr points_marker;
        points_marker.reset(new PointsMarker(0, vis_manager_, markers_node_));
        points_markers_.push_back( points_marker );
        marker = points_marker;
      }
        break;
      case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        marker.reset(new TextViewFacingMarker(0, vis_manager_, markers_node_));
      }
        break;
      case visualization_msgs::Marker::MESH_RESOURCE:
      {
        marker.reset(new MeshResourceMarker(0, vis_manager_, markers_node_));
      }
        break;

      case visualization_msgs::Marker::TRIANGLE_LIST:
      {
        marker.reset(new TriangleListMarker(0, vis_manager_, markers_node_));
      }
        break;
      default:
        ROS_ERROR( "Unknown marker type: %d", message.markers[i].type );
    }

    marker->setMessage( message.markers[ i ]);
    marker->setInteractiveObject( shared_from_this() );

    addHighlightPass(marker->getMaterials());

    // The marker will set its position relative to the fixed frame,
    // but we have attached it our own scene node, so we will have to
    // correct for that.
    marker->setPosition( markers_node_->convertWorldToLocalPosition( marker->getPosition() ) );
    marker->setOrientation( markers_node_->convertWorldToLocalOrientation( marker->getOrientation() ) );

    markers_.push_back(marker);
  }
}

InteractiveMarkerControl::~InteractiveMarkerControl()
{
  vis_manager_->getSceneManager()->destroySceneNode(control_frame_node_);
  vis_manager_->getSceneManager()->destroySceneNode(markers_node_);

  if( view_facing_ )
  {
    vis_manager_->getSceneManager()->removeListener(this);
  }
}

void InteractiveMarkerControl::processMessage( const visualization_msgs::InteractiveMarkerControl &message )
{
  name_ = message.name;
  description_ = message.description;
  interaction_mode_ = message.interaction_mode;
  always_visible_ = message.always_visible;
  orientation_mode_ = message.orientation_mode;

  control_orientation_ = Ogre::Quaternion(message.orientation.w,
      message.orientation.x, message.orientation.y, message.orientation.z);
  control_orientation_.normalise();

  bool new_view_facingness = (message.orientation_mode == visualization_msgs::InteractiveMarkerControl::VIEW_FACING);
  if( new_view_facingness != view_facing_ )
  {
    if( new_view_facingness )
    {
      vis_manager_->getSceneManager()->addListener(this);
    }
    else
    {
      vis_manager_->getSceneManager()->removeListener(this);
    }
    view_facing_ = new_view_facingness;
  }
  
  independent_marker_orientation_ = message.independent_marker_orientation;

  // highlight_passes_ have raw pointers into the markers_, so must
  // clear them at the same time.
  highlight_passes_.clear();
  markers_.clear();
  points_markers_.clear();

  // Initially, the pose of this marker's node and the interactive
  // marker are identical, but that may change.
  control_frame_node_->setPosition(parent_->getPosition());
  markers_node_->setPosition(parent_->getPosition());

  if( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::INHERIT )
  {
    control_frame_node_->setOrientation(parent_->getOrientation());
    markers_node_->setOrientation(parent_->getOrientation());
  }
  else
  {
    control_frame_node_->setOrientation( Ogre::Quaternion::IDENTITY );
    markers_node_->setOrientation( Ogre::Quaternion::IDENTITY );
  }

  makeMarkers( message );

  // It's not clear to me why this one setOrientation() call needs to
  // be here and not above makeMarkers() with the other
  // setOrientation() calls, but it works correctly when here and
  // incorrectly when there.  Sorry. -hersh
  if( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::VIEW_FACING &&
      independent_marker_orientation_ )
  {
    markers_node_->setOrientation( parent_->getOrientation() );
  }

  enableInteraction(vis_manager_->getSelectionManager()->getInteractionEnabled());
}

// This is an Ogre::SceneManager::Listener function, and is configured
// to be called only if this is a VIEW_FACING control.
void InteractiveMarkerControl::preFindVisibleObjects(
    Ogre::SceneManager *source,
    Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport *v )
{
  updateControlOrientationForViewFacing( v );
}

void InteractiveMarkerControl::updateControlOrientationForViewFacing( Ogre::Viewport* v )
{
  Ogre::Quaternion x_view_facing_rotation =
      control_orientation_.xAxis().getRotationTo( v->getCamera()->getDerivedDirection());

  // rotate so z axis is up
  Ogre::Vector3 z_axis_2 = x_view_facing_rotation * control_orientation_.zAxis();
  Ogre::Quaternion align_yz_rotation = z_axis_2.getRotationTo(v->getCamera()->getDerivedUp());

  // rotate
  Ogre::Quaternion rotate_around_x = Ogre::Quaternion(rotation_, v->getCamera()->getDerivedDirection());

  Ogre::Quaternion rotation = reference_node_->convertWorldToLocalOrientation(
      rotate_around_x * align_yz_rotation * x_view_facing_rotation );

  control_frame_node_->setOrientation( rotation );

  if ( !independent_marker_orientation_ )
  {
    markers_node_->setOrientation( rotation );
    // we need to refresh the node manually, since the scene manager will only do this one frame
    // later otherwise
    markers_node_->_update(true, false);
  }
}

void InteractiveMarkerControl::hideVisible()
{
  saved_visibility_state_ = visible_;
  setVisible(false);
}

void InteractiveMarkerControl::restoreVisible()
{
  setVisible(saved_visibility_state_);
}

void InteractiveMarkerControl::setVisible( bool visible )
{
  visible_ = visible;

  if (always_visible_)
  {
    markers_node_->setVisible(visible_);
  } else
  {
    markers_node_->setVisible(interaction_enabled_ && visible_);
  }
}

void InteractiveMarkerControl::update()
{
  if( dragging_ )
  {
    handleMouseMovement( dragging_in_place_event_ );
  }
}

void InteractiveMarkerControl::enableInteraction( bool enable )
{
  interaction_enabled_ = enable;
  setVisible(visible_);
  if (!enable)
  {
    setHighlight(0.0);
  }
}

void InteractiveMarkerControl::interactiveMarkerPoseChanged(
    Ogre::Vector3 int_marker_position, Ogre::Quaternion int_marker_orientation )
{
  control_frame_node_->setPosition(int_marker_position);
  markers_node_->setPosition(int_marker_position);

  switch (orientation_mode_)
  {
    case visualization_msgs::InteractiveMarkerControl::INHERIT:
      control_frame_node_->setOrientation(int_marker_orientation);
      markers_node_->setOrientation(control_frame_node_->getOrientation());
      break;

    case visualization_msgs::InteractiveMarkerControl::FIXED:
    {
      control_frame_node_->setOrientation( Ogre::Quaternion( rotation_, control_orientation_.xAxis() ));
      markers_node_->setOrientation(control_frame_node_->getOrientation());
      break;
    }

    case visualization_msgs::InteractiveMarkerControl::VIEW_FACING:
      if( drag_viewport_ )
      {
        updateControlOrientationForViewFacing( drag_viewport_ );
      }
      if ( independent_marker_orientation_ )
      {
        markers_node_->setOrientation(int_marker_orientation);
      }
      break;

    default:
      break;
  }
}

Ogre::Vector3 InteractiveMarkerControl::closestPointOnLineToPoint( const Ogre::Vector3& line_start,
                                                                   const Ogre::Vector3& line_dir,
                                                                   const Ogre::Vector3& test_point )
{
  // Find closest point on projected ray to mouse point
  // Math: if P is the start of the ray, v is the direction vector of
  //       the ray (not normalized), and X is the test point, then the
  //       closest point on the line to X is given by:
  //
  //               (X-P).v
  //       P + v * -------
  //                 v.v
  //       where "." is the dot product.
  double factor = ( test_point - line_start ).dotProduct( line_dir ) / line_dir.dotProduct( line_dir );
  Ogre::Vector3 closest_point = line_start + line_dir * factor;
  return closest_point;
}

void InteractiveMarkerControl::rotate( Ogre::Ray &mouse_ray )
{
  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  Ogre::Vector3 rotation_axis = control_frame_orientation_at_mouse_down_ * control_orientation_.xAxis();

  // Find rotation_center = 3D point closest to grab_point_ which is
  // on the rotation axis, relative to the reference frame.
  Ogre::Vector3 rotation_center = closestPointOnLineToPoint( control_frame_node_->getPosition(),
                                                             rotation_axis,
                                                             grab_point_ );

  // Find intersection of mouse_ray with plane centered at rotation_center.
  if( intersectSomeYzPlane( mouse_ray, rotation_center, control_frame_orientation_at_mouse_down_,
                            intersection_3d, intersection_2d, ray_t ))
  {
    // Find rotation
    Ogre::Vector3 grab_rel_center = grab_point_ - rotation_center;
    Ogre::Vector3 mouse_rel_center = intersection_3d - rotation_center;

    Ogre::Quaternion orientation_change_since_mouse_down =
      grab_rel_center.getRotationTo( mouse_rel_center, rotation_axis );

    Ogre::Radian rot;
    Ogre::Vector3 axis;
    orientation_change_since_mouse_down.ToAngleAxis( rot, axis );
    // Quaternion::ToAngleAxis() always gives a positive angle.  The
    // axis it emits (in this case) will either be equal to
    // rotation_axis or will be the negative of it.  Doing the
    // dot-product then gives either 1.0 or -1.0, which is just what
    // we need to get the sign for our angle.
    Ogre::Radian rotation_since_mouse_down = rot * axis.dotProduct( rotation_axis );

    rotation_ = rotation_at_mouse_down_ + rotation_since_mouse_down;
    parent_->setPose( parent_->getPosition(),
                      orientation_change_since_mouse_down * parent_orientation_at_mouse_down_,
                      name_ );
  }
}

void InteractiveMarkerControl::movePlane( Ogre::Ray &mouse_ray )
{
  if( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::VIEW_FACING &&
      drag_viewport_ )
  {
    updateControlOrientationForViewFacing( drag_viewport_ );
  }

  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  if( intersectSomeYzPlane( mouse_ray, grab_point_, control_frame_node_->getOrientation(),
                            intersection_3d, intersection_2d, ray_t ))
  {
    parent_->setPose( intersection_3d - grab_point_ + parent_position_at_mouse_down_, parent_->getOrientation(), name_ );
  }
}

/** Project a world position onto the viewport to find screen coordinates in pixels.
 * @param screen_pos the resultant screen position, in pixels. */
void InteractiveMarkerControl::worldToScreen( const Ogre::Vector3& pos_rel_reference,
                                              const Ogre::Viewport* viewport,
                                              Ogre::Vector2& screen_pos )
{
  Ogre::Vector3 world_pos = reference_node_->convertLocalToWorldPosition( pos_rel_reference );

  const Ogre::Camera* cam = viewport->getCamera();
  Ogre::Vector3 homogeneous_screen_position = cam->getProjectionMatrix() * (cam->getViewMatrix() * world_pos);

  double half_width = viewport->getActualWidth() / 2.0;
  double half_height = viewport->getActualHeight() / 2.0;

  screen_pos.x = half_width + (half_width * homogeneous_screen_position.x) - .5;
  screen_pos.y = half_height + (half_height * -homogeneous_screen_position.y) - .5;
}

/** Find the closest point on target_ray to mouse_ray.
 * @returns false if rays are effectively parallel, true otherwise.
 */
bool InteractiveMarkerControl::findClosestPoint( const Ogre::Ray& target_ray,
                                                 const Ogre::Ray& mouse_ray,
                                                 Ogre::Vector3& closest_point )
{
  // Find the closest point on target_ray to any point on mouse_ray.
  //
  // Math taken from http://paulbourke.net/geometry/lineline3d/
  // line P1->P2 is target_ray
  // line P3->P4 is mouse_ray

  Ogre::Vector3 v13 = target_ray.getOrigin() - mouse_ray.getOrigin();
  Ogre::Vector3 v43 = mouse_ray.getDirection();
  Ogre::Vector3 v21 = target_ray.getDirection();
  double d1343 = v13.dotProduct( v43 );
  double d4321 = v43.dotProduct( v21 );
  double d1321 = v13.dotProduct( v21 );
  double d4343 = v43.dotProduct( v43 );
  double d2121 = v21.dotProduct( v21 );

  double denom = d2121 * d4343 - d4321 * d4321;
  if( fabs( denom ) <= Ogre::Matrix3::EPSILON )
  {
    return false;
  }
  double numer = d1343 * d4321 - d1321 * d4343;

  double mua = numer / denom;
  closest_point = target_ray.getPoint( mua );
  return true;
}

void InteractiveMarkerControl::moveAxis( const Ogre::Ray& mouse_ray, const ViewportMouseEvent& event )
{
  // compute control-axis ray based on grab_point_, etc.
  Ogre::Ray control_ray;
  control_ray.setOrigin( grab_point_ );
  control_ray.setDirection( control_frame_node_->getOrientation() * control_orientation_.xAxis() );
  
  // project control-axis ray onto screen.
  Ogre::Vector2 control_ray_screen_start, control_ray_screen_end;
  worldToScreen( control_ray.getOrigin(), event.viewport, control_ray_screen_start );
  worldToScreen( control_ray.getPoint( 1 ), event.viewport, control_ray_screen_end );

  Ogre::Vector2 mouse_point( event.x, event.y );

  // Find closest point on projected ray to mouse point
  // Math: if P is the start of the ray, v is the direction vector of
  //       the ray (not normalized), and X is the test point, then the
  //       closest point on the line to X is given by:
  //
  //               (X-P).v
  //       P + v * -------
  //                 v.v
  //       where "." is the dot product.
  Ogre::Vector2 control_ray_screen_dir = control_ray_screen_end - control_ray_screen_start;
  double denominator = control_ray_screen_dir.dotProduct( control_ray_screen_dir );
  if( fabs( denominator ) > Ogre::Matrix3::EPSILON ) // If the control ray is not straight in line with the view.
  {
    double factor =
      ( mouse_point - control_ray_screen_start ).dotProduct( control_ray_screen_dir ) / denominator;
    
    Ogre::Vector2 closest_screen_point = control_ray_screen_start + control_ray_screen_dir * factor;

    // make a new "mouse ray" for the point on the projected ray
    int width = event.viewport->getActualWidth() - 1;
    int height = event.viewport->getActualHeight() - 1;
    Ogre::Ray new_mouse_ray = event.viewport->getCamera()->getCameraToViewportRay( (closest_screen_point.x+.5) / width,
                                                                                   (closest_screen_point.y+.5) / height );
    new_mouse_ray.setOrigin( reference_node_->convertWorldToLocalPosition( new_mouse_ray.getOrigin() ) );
    new_mouse_ray.setDirection( reference_node_->convertWorldToLocalOrientation( Ogre::Quaternion::IDENTITY ) * new_mouse_ray.getDirection() );

    // find closest point on control-axis ray to new mouse ray (should intersect actually)
    Ogre::Vector3 closest_point;
    if( findClosestPoint( control_ray, new_mouse_ray, closest_point ))
    {
      // set position of parent to closest_point - grab_point_ + parent_position_at_mouse_down_.
      parent_->setPose( closest_point - grab_point_ + parent_position_at_mouse_down_,
                        parent_->getOrientation(), name_ );
    }
  }
}

void InteractiveMarkerControl::moveRotate( Ogre::Ray &mouse_ray )
{
  if( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::VIEW_FACING &&
      drag_viewport_ )
  {
    updateControlOrientationForViewFacing( drag_viewport_ );
  }

  Ogre::Vector3 new_drag_rel_ref;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  // get rotation axis rel ref (constant for entire drag)
  //  - rotation_axis_

  // get current rotation center rel ref
  //  - compute rotation center rel control frame at mouse-down (constant for entire drag)
  //  - current rotation center rel ref = current control frame * above
  Ogre::Matrix4 control_rel_ref;
  control_rel_ref.makeTransform( control_frame_node_->getPosition(),
                                 Ogre::Vector3::UNIT_SCALE,
                                 control_frame_node_->getOrientation() );
  Ogre::Vector3 rotation_center = control_rel_ref * rotation_center_rel_control_;

  // get previous 3D drag point rel ref
  //  - compute grab point rel control frame at mouse-down (constant for entire drag)
  //  - prev_drag_rel_ref = current control frame + above
  Ogre::Vector3 prev_drag_rel_ref = control_rel_ref * grab_point_rel_control_;

  // get new 3D drag point rel ref (this is "intersection_3d" in rotate().)
  //  - intersectSomeYzPlane( mouse_ray, rotation_center, control_frame_orientation )
  if( intersectSomeYzPlane( mouse_ray, rotation_center, control_frame_node_->getOrientation(),
                            new_drag_rel_ref, intersection_2d, ray_t ))
  {
    // compute rotation angle from old drag point to new.
    //  - prev_rel_center = prev_drag_rel_ref - rotation_center
    //  - new_rel_center = new_drag_rel_ref - rotation_center
    //  - rotation_change = prev_rel_center.getRotationTo( new_rel_center, rotation_axis )
    //  - get Radians of angle change
    //  - rotation_ += angle_change
    //  - parent_->rotate(rotation_change)
    Ogre::Vector3 prev_rel_center = prev_drag_rel_ref - rotation_center;
    Ogre::Vector3 new_rel_center = new_drag_rel_ref - rotation_center;
    if( new_rel_center.length() > Ogre::Matrix3::EPSILON )
    {
      Ogre::Quaternion rotation_change = prev_rel_center.getRotationTo( new_rel_center, rotation_axis_ );
      Ogre::Radian rot;
      Ogre::Vector3 axis;
      rotation_change.ToAngleAxis( rot, axis );
      // Quaternion::ToAngleAxis() always gives a positive angle.  The
      // axis it emits (in this case) will either be equal to
      // rotation_axis or will be the negative of it.  Doing the
      // dot-product then gives either 1.0 or -1.0, which is just what
      // we need to get the sign for our angle.
      Ogre::Radian angle_change = rot * axis.dotProduct( rotation_axis_ );
      rotation_ += angle_change;
      parent_->rotate( rotation_change, name_ );

      // compute translation from rotated old drag point to new drag point.
      //  - pos_change = new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length())
      //  - parent_->translate(pos_change)
      parent_->translate( new_rel_center * (1.0 - prev_rel_center.length() / new_rel_center.length()), name_ );
    }    
  }
}

void InteractiveMarkerControl::setHighlight( float a )
{
  std::set<Ogre::Pass*>::iterator it;
  for (it = highlight_passes_.begin(); it != highlight_passes_.end(); it++)
  {
    (*it)->setAmbient(a,a,a);
  }

  std::vector<PointsMarkerPtr>::iterator pm_it;
  for( pm_it = points_markers_.begin(); pm_it != points_markers_.end(); pm_it++ )
  {
    (*pm_it)->setHighlightColor( a, a, a );
  }
}

void InteractiveMarkerControl::recordDraggingInPlaceEvent( ViewportMouseEvent& event )
{
  dragging_in_place_event_ = event;
  dragging_in_place_event_.type = QEvent::MouseMove;
}

void InteractiveMarkerControl::handleMouseEvent( ViewportMouseEvent& event )
{
  // * check if this is just a receive/lost focus event
  // * try to hand over the mouse event to the parent interactive marker
  // * otherwise, execute mouse move handling

  // handle receive/lose focus
  if( event.type == QEvent::FocusIn )
  {
    has_focus_ = true;
    std::set<Ogre::Pass*>::iterator it;
    setHighlight( HOVER_HIGHLIGHT );
  }
  else if( event.type == QEvent::FocusOut )
  {
    has_focus_ = false;
    setHighlight(0.0);
    return;
  }

  // change dragging state
  switch( interaction_mode_ )
  {
  case visualization_msgs::InteractiveMarkerControl::BUTTON:
    if( event.leftUp() )
    {
      Ogre::Vector3 point_rel_world;
      bool got_3D_point = vis_manager_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, point_rel_world );

      visualization_msgs::InteractiveMarkerFeedback feedback;
      feedback.event_type = visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK;
      feedback.control_name = name_;
      feedback.marker_name = parent_->getName();
      parent_->publishFeedback( feedback, got_3D_point, point_rel_world );
    }
    break;

  case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS:
  case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE:
  case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE:
  case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
    if( event.leftDown() )
    {
      parent_->startDragging();
      dragging_ = true;
      drag_viewport_ = event.viewport;

      recordDraggingInPlaceEvent( event );
      if( ! vis_manager_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, grab_point_ ))
      {
        // If we couldn't get a 3D point for the grab, just use the
        // current relative position of the control frame.
        grab_point_ = control_frame_node_->getPosition();
      }
      else
      {
        // If we could get a 3D point for the grab, convert it from
        // the world frame to the reference frame (in case those are different).
        grab_point_ = reference_node_->convertWorldToLocalPosition(grab_point_);
      }
      grab_pixel_.x = event.x;
      grab_pixel_.y = event.y;
      parent_position_at_mouse_down_ = parent_->getPosition();
      parent_orientation_at_mouse_down_ = parent_->getOrientation();

      if( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::VIEW_FACING &&
          drag_viewport_ )
      {
        updateControlOrientationForViewFacing( drag_viewport_ );
      }
      control_frame_orientation_at_mouse_down_ = control_frame_node_->getOrientation();
      rotation_at_mouse_down_ = rotation_;

      rotation_axis_ = control_frame_node_->getOrientation() * control_orientation_.xAxis();

      // Find rotation_center = 3D point closest to grab_point_ which is
      // on the rotation axis, relative to the reference frame.
      Ogre::Vector3 rotation_center_rel_ref = closestPointOnLineToPoint( parent_->getPosition(),
                                                                         rotation_axis_,
                                                                         grab_point_ );
      Ogre::Matrix4 reference_rel_control_frame;
      reference_rel_control_frame.makeInverseTransform( control_frame_node_->getPosition(),
                                                        Ogre::Vector3::UNIT_SCALE,
                                                        control_frame_node_->getOrientation() );
      rotation_center_rel_control_ = reference_rel_control_frame * rotation_center_rel_ref;
      grab_point_rel_control_ = reference_rel_control_frame * grab_point_;
    }
    if( event.leftUp() )
    {
      dragging_ = false;
      drag_viewport_ = NULL;
      parent_->stopDragging();
    }
    break;

  default:
    break;
  }

  if( event.leftDown() )
  {
    setHighlight( ACTIVE_HIGHLIGHT );
  }
  else if( event.leftUp() )
  {
    setHighlight( HOVER_HIGHLIGHT );
  }

  if (!parent_->handleMouseEvent(event, name_))
  {
    if( event.type == QEvent::MouseMove && event.left() )
    {
      recordDraggingInPlaceEvent( event );
      handleMouseMovement( event );
    }
  }
}

void InteractiveMarkerControl::handleMouseMovement( ViewportMouseEvent& event )
{
  // handle mouse movement
  float width = event.viewport->getActualWidth() - 1;
  float height = event.viewport->getActualHeight() - 1;

  Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay( (event.x + .5) / width,
                                                                             (event.y + .5) / height );

  Ogre::Ray last_mouse_ray =
    event.viewport->getCamera()->getCameraToViewportRay(
      (event.last_x + .5) / width, (event.last_y + .5) / height);

  //convert rays into reference frame
  mouse_ray.setOrigin( reference_node_->convertWorldToLocalPosition( mouse_ray.getOrigin() ) );
  mouse_ray.setDirection( reference_node_->convertWorldToLocalOrientation( Ogre::Quaternion::IDENTITY ) * mouse_ray.getDirection() );

  last_mouse_ray.setOrigin( reference_node_->convertWorldToLocalPosition( last_mouse_ray.getOrigin() ) );
  last_mouse_ray.setDirection( reference_node_->convertWorldToLocalOrientation( Ogre::Quaternion::IDENTITY ) * last_mouse_ray.getDirection() );

  switch (interaction_mode_)
  {
  case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS:
    moveAxis( mouse_ray, event );
    break;

  case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE:
    movePlane( mouse_ray );
    break;

  case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE:
    moveRotate( mouse_ray );
    break;

  case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
    rotate(mouse_ray);
    break;

  default:
    break;
  }
}

bool InteractiveMarkerControl::intersectYzPlane( const Ogre::Ray& mouse_ray,
                                                 Ogre::Vector3& intersection_3d,
                                                 Ogre::Vector2& intersection_2d,
                                                 float &ray_t )
{
  return intersectSomeYzPlane( mouse_ray,
                               control_frame_node_->getPosition(),
                               control_frame_node_->getOrientation(),
                               intersection_3d, intersection_2d, ray_t );
}

bool InteractiveMarkerControl::intersectSomeYzPlane( const Ogre::Ray& mouse_ray,
                                                     const Ogre::Vector3& point_on_plane,
                                                     const Ogre::Quaternion& plane_orientation,
                                                     Ogre::Vector3& intersection_3d,
                                                     Ogre::Vector2& intersection_2d,
                                                     float& ray_t )
{
  Ogre::Vector3 normal = plane_orientation * control_orientation_.xAxis();
  Ogre::Vector3 axis_1 = plane_orientation * control_orientation_.yAxis();
  Ogre::Vector3 axis_2 = plane_orientation * control_orientation_.zAxis();

  Ogre::Plane plane(normal, point_on_plane);

  Ogre::Vector2 origin_2d(point_on_plane.dotProduct(axis_1), point_on_plane.dotProduct(axis_2));

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(plane);
  if (intersection.first)
  {
    intersection_3d = mouse_ray.getPoint(intersection.second);
    intersection_2d = Ogre::Vector2(intersection_3d.dotProduct(axis_1), intersection_3d.dotProduct(axis_2));
    intersection_2d -= origin_2d;

    ray_t = intersection.second;
    return true;
  }

  ray_t = 0;
  return false;
}

void InteractiveMarkerControl::addHighlightPass( S_MaterialPtr materials )
{
  S_MaterialPtr::iterator it;

  for (it = materials.begin(); it != materials.end(); it++)
  {
    Ogre::MaterialPtr material = *it;
    Ogre::Pass *original_pass = material->getTechnique(0)->getPass(0);
    Ogre::Pass *pass = material->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_ADD);
    pass->setDepthWriteEnabled(true);
    pass->setDepthCheckEnabled(true);
    pass->setLightingEnabled(true);
    pass->setAmbient(0, 0, 0);
    pass->setDiffuse(0, 0, 0, 0);
    pass->setSpecular(0, 0, 0, 0);
    pass->setCullingMode(original_pass->getCullingMode());

    highlight_passes_.insert(pass);
  }
}

}
