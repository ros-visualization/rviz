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

#include <wx/tooltip.h>

namespace rviz
{

InteractiveMarkerControl::InteractiveMarkerControl(VisualizationManager* vis_manager,
    const visualization_msgs::InteractiveMarkerControl &message, InteractiveMarker *parent ) :
    vis_manager_(vis_manager)
,   parent_(parent)
,   rotation_(0)
,   interaction_enabled_(false)
{
  scene_node_ = vis_manager->getSceneManager()->getRootSceneNode()->createChildSceneNode();

  interaction_mode_ = message.interaction_mode;
  always_visible_ = message.always_visible;

  orientation_mode_ = message.orientation_mode;

  tool_tip_ = message.tool_tip;

  if ( message.orientation_mode == visualization_msgs::InteractiveMarkerControl::VIEW_FACING )
  {
    vis_manager->getSceneManager()->addListener( this );
  }

  control_orientation_ = Ogre::Quaternion( message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z );
  control_orientation_.normalise();

  intitial_orientation_ = parent->getReferenceOrientation().Inverse() * parent->getOrientation();

  //initially, the pose of this marker's node and the interactive marker are identical, but that may change
  scene_node_->setPosition( parent_->getPosition() );
  scene_node_->setOrientation( parent_->getOrientation() );

  for ( unsigned i = 0; i<message.markers.size(); i++ )
  {
    MarkerBasePtr marker;

    // create a marker with the given type
    switch (message.markers[i].type)
    {
      case visualization_msgs::Marker::CUBE:
      case visualization_msgs::Marker::CYLINDER:
      case visualization_msgs::Marker::SPHERE:
      {
        marker.reset(new ShapeMarker(0, vis_manager_, scene_node_));
      }
        break;

      case visualization_msgs::Marker::ARROW:
      {
        marker.reset(new ArrowMarker(0, vis_manager_, scene_node_));
      }
        break;

      case visualization_msgs::Marker::LINE_STRIP:
      {
        marker.reset(new LineStripMarker(0, vis_manager_, scene_node_));
      }
        break;
      case visualization_msgs::Marker::LINE_LIST:
      {
        marker.reset(new LineListMarker(0, vis_manager_, scene_node_));
      }
        break;
      case visualization_msgs::Marker::SPHERE_LIST:
      case visualization_msgs::Marker::CUBE_LIST:
      case visualization_msgs::Marker::POINTS:
      {
        marker.reset(new PointsMarker(0, vis_manager_, scene_node_));
      }
        break;
      case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        marker.reset(new TextViewFacingMarker(0, vis_manager_, scene_node_));
      }
        break;
      case visualization_msgs::Marker::MESH_RESOURCE:
      {
        marker.reset(new MeshResourceMarker(0, vis_manager_, scene_node_));
      }
        break;

      case visualization_msgs::Marker::TRIANGLE_LIST:
      {
        marker.reset(new TriangleListMarker(0, vis_manager_, scene_node_));
      }
        break;
      default:
        ROS_ERROR( "Unknown marker type: %d", message.markers[i].type );
    }

    //this is a bit hacky, sorry..

    marker->setMessage(message.markers[i]);
    marker->setControl(this);

    addHighlightPass( marker->getMaterials() );

    // the marker will set it's position relative to the fixed frame,
    // but we have attached it our own non-root scene node,
    // so we will have to correct for that

    // get translation from scene node to marker in fixed frame coords, transform into scene node frame
    marker->setPosition( scene_node_->getOrientation().Inverse() * ( marker->getPosition() - scene_node_->getPosition() ) );
    marker->setOrientation( scene_node_->getOrientation().Inverse() * marker->getOrientation() );

    markers_.push_back( marker );
  }
}

InteractiveMarkerControl::~InteractiveMarkerControl()
{
  vis_manager_->getSceneManager()->destroySceneNode( scene_node_ );

  if ( orientation_mode_ == visualization_msgs::InteractiveMarkerControl::VIEW_FACING )
  {
    vis_manager_->getSceneManager()->removeListener( this );
  }
}

void InteractiveMarkerControl::preFindVisibleObjects(Ogre::SceneManager *source, Ogre::SceneManager::IlluminationRenderStage irs, Ogre::Viewport *v)
{
  Ogre::Quaternion x_view_facing_rotation = control_orientation_.xAxis().getRotationTo( v->getCamera()->getDerivedDirection() );

  // rotate so z axis is up
  Ogre::Vector3 z_axis_2 = x_view_facing_rotation * control_orientation_.zAxis();
  Ogre::Quaternion align_yz_rotation = z_axis_2.getRotationTo( v->getCamera()->getDerivedUp() );

  // rotate
  Ogre::Quaternion rotate_around_x = Ogre::Quaternion( rotation_, v->getCamera()->getDerivedDirection() );

  scene_node_->setOrientation( rotate_around_x * align_yz_rotation * x_view_facing_rotation );

  // we need to refresh the node manually, since the scene manager will only do this one frame
  // later otherwise
  scene_node_->_update(true,false);
}

void InteractiveMarkerControl::setVisible( bool visible )
{
  scene_node_->setVisible( visible );
}

void InteractiveMarkerControl::update( float heart_beat )
{
  if ( interaction_enabled_ && !has_focus_ )
  {
    std::set<Ogre::Pass*>::iterator it;
    for ( it=highlight_passes_.begin(); it!=highlight_passes_.end(); it++ )
    {
      (*it)->setAmbient( heart_beat, heart_beat, heart_beat );
    }
  }
}

void InteractiveMarkerControl::enableInteraction(bool enable)
{
  interaction_enabled_ = enable;
  if ( !always_visible_ )
  {
    scene_node_->setVisible(enable);
  }
  if ( !enable )
  {
    std::set<Ogre::Pass*>::iterator it;
    for ( it=highlight_passes_.begin(); it!=highlight_passes_.end(); it++ )
    {
      (*it)->setAmbient( 0,0,0 );
    }
  }
}


void InteractiveMarkerControl::interactiveMarkerPoseChanged( Ogre::Vector3 int_marker_position, Ogre::Quaternion int_marker_orientation )
{
  scene_node_->setPosition( int_marker_position );

  switch ( orientation_mode_ )
  {
    case visualization_msgs::InteractiveMarkerControl::INHERIT:
      scene_node_->setOrientation( int_marker_orientation );
      break;

    case visualization_msgs::InteractiveMarkerControl::FIXED:
    {
      scene_node_->setOrientation( parent_->getReferenceOrientation() * intitial_orientation_ * Ogre::Quaternion( rotation_, control_orientation_.xAxis() ) );
      break;
    }

    case visualization_msgs::InteractiveMarkerControl::VIEW_FACING:
      break;

    default:
      break;
  }
}


void InteractiveMarkerControl::rotate(Ogre::Ray &mouse_ray)
{
  Ogre::Vector3 last_intersection_3d, intersection_3d;
  Ogre::Vector2 last_intersection_2d, intersection_2d;
  float last_ray_t, ray_t;

  intersectYzPlane( mouse_ray, intersection_3d, intersection_2d, ray_t );

  // we don't want to come too close to the center of rotation,
  // as things will get unstable there
  if ( intersection_2d.length() < 0.2 * parent_->getSize() )
  {
    return;
  }

  intersectYzPlane( last_mouse_ray_, last_intersection_3d, last_intersection_2d, last_ray_t );

  double last_angle = atan2( last_intersection_2d.x, last_intersection_2d.y );
  double angle = atan2( intersection_2d.x, intersection_2d.y );

  Ogre::Radian delta_angle( (last_angle-angle) );
  Ogre::Quaternion delta_orientation( delta_angle, scene_node_->getOrientation() * control_orientation_.xAxis() );

  rotation_ += delta_angle;
  parent_->rotate( delta_orientation );
}

void InteractiveMarkerControl::movePlane(Ogre::Ray &mouse_ray )
{
  Ogre::Vector3 last_intersection_3d, intersection_3d;
  Ogre::Vector2 last_intersection_2d, intersection_2d;
  float last_ray_t, ray_t;

  if ( intersectYzPlane( last_mouse_ray_, last_intersection_3d, last_intersection_2d, last_ray_t ) &&
       intersectYzPlane( mouse_ray, intersection_3d, intersection_2d, ray_t ) )
  {
    Ogre::Vector3 translate_delta = intersection_3d - last_intersection_3d;
    parent_->translate( translate_delta );
  }
}

void InteractiveMarkerControl::followMouse(Ogre::Ray &mouse_ray, float max_dist )
{
  Ogre::Vector3 intersection_3d;
  Ogre::Vector2 intersection_2d;
  float ray_t;

  if ( intersectYzPlane( mouse_ray, intersection_3d, intersection_2d, ray_t ) )
  {
    Ogre::Vector3 diff = intersection_3d - scene_node_->getPosition();
    if ( diff.length() > max_dist )
    {
      Ogre::Vector3 dir = diff.normalisedCopy();

      Ogre::Vector3 translate_delta = (diff.length() - max_dist) * dir;
      parent_->translate( translate_delta );
    }
  }
}

void InteractiveMarkerControl::handleMouseEvent(ViewportMouseEvent& event)
{
  // * check if this is just a receive/lost focus event
  // * try to hand over the mouse event to the parent interactive marker
  // * otherwise, execute mouse move handling

  if ( event.event.GetEventType() == wxEVT_SET_FOCUS )
  {
    //event.panel->SetToolTip( wxString::FromAscii( tool_tip_.c_str() ) );
    has_focus_ = true;
    std::set<Ogre::Pass*>::iterator it;
    for ( it=highlight_passes_.begin(); it!=highlight_passes_.end(); it++ )
    {
      (*it)->setAmbient(0.5,0.5,0.5);
    }
    return;
  }
  else if ( event.event.GetEventType() == wxEVT_KILL_FOCUS )
  {
    //event.panel->UnsetToolTip();
    has_focus_ = false;
    std::set<Ogre::Pass*>::iterator it;
    for ( it=highlight_passes_.begin(); it!=highlight_passes_.end(); it++ )
    {
      (*it)->setAmbient( 0,0,0 );
    }
    return;
  }

  int width = event.viewport->getActualWidth();
  int height = event.viewport->getActualHeight();

  Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
      (float)event.event.GetX() / (float)width, (float)event.event.GetY() / (float)height );

  last_mouse_ray_  = event.viewport->getCamera()->getCameraToViewportRay(
      (float)event.last_x / (float)width, (float)event.last_y / (float)height );


  if ( event.event.LeftDown() )
  {
    old_target_frame_ = vis_manager_->getTargetFrame();
    ROS_INFO_STREAM( "Saving old target frame: " << old_target_frame_ );
    vis_manager_->setTargetFrame( parent_->getReferenceFrame() );
  }
  if ( event.event.LeftUp() )
  {
    ROS_INFO_STREAM( "Setting old target frame: " << old_target_frame_ );
    vis_manager_->setTargetFrame( old_target_frame_ );
  }

  if ( !parent_->handleMouseEvent( event ) )
  {
    switch ( interaction_mode_ )
    {
      case visualization_msgs::InteractiveMarkerControl::BUTTON:
        if ( event.event.LeftUp() )
        {
          ROS_INFO( "click" );
        }
        break;

      case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS:

        if ( event.event.Dragging() )
        {
          float last_pos, pos;
          if ( getClosestPosOnAxis(last_mouse_ray_,last_pos) && getClosestPosOnAxis(mouse_ray,pos) )
          {
            float delta = pos - last_pos;
            Ogre::Vector3 translate_delta = scene_node_->getOrientation() * control_orientation_.xAxis() * delta;
            parent_->translate( translate_delta );
          }
        }
        break;

      case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE:
        if ( event.event.Dragging() )
        {
          movePlane( mouse_ray );
        }
        break;

      case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE:
        if ( event.event.Dragging() )
        {
          rotate(mouse_ray);
          followMouse(mouse_ray, parent_->getSize()*0.7);
          break;
        }

      case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
        if ( event.event.Dragging() )
        {
          rotate(mouse_ray);
          break;
        }

      case visualization_msgs::InteractiveMarkerControl::MENU:
        break;

      default:
        break;
    }
  }

//  last_mouse_ray_ = mouse_ray;
}

bool InteractiveMarkerControl::intersectYzPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d, Ogre::Vector2 &intersection_2d, float &ray_t )
{
  Ogre::Vector3 position = scene_node_->getPosition();

  Ogre::Vector3 normal = scene_node_->getOrientation() * control_orientation_.xAxis();
  Ogre::Vector3 axis_1 = scene_node_->getOrientation() * control_orientation_.yAxis();
  Ogre::Vector3 axis_2 = scene_node_->getOrientation() * control_orientation_.zAxis();

  Ogre::Plane plane( normal, position );

  Ogre::Vector2 origin_2d( position.dotProduct( axis_1 ), position.dotProduct( axis_2 ) );

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects( plane );
  if ( intersection.first )
  {
    intersection_3d = mouse_ray.getPoint( intersection.second );
    intersection_2d = Ogre::Vector2( intersection_3d.dotProduct(axis_1), intersection_3d.dotProduct(axis_2) );
    intersection_2d -= origin_2d;

    //ROS_INFO( "Mouse ray intersects plane at %f %f %f (%f %f)",
      //  intersection_3d.x, intersection_3d.y, intersection_3d.z, intersection_2d.x, intersection_2d.y );

    ray_t = intersection.second;
    return true;
  }

  ray_t = 0;
  return false;
}

bool InteractiveMarkerControl::getClosestPosOnAxis( Ogre::Ray mouse_ray, float &ray_t )
{
  Ogre::Vector3 axis = scene_node_->getOrientation() * control_orientation_.xAxis();

  //axis2 is perpendicular to mouse ray and axis ray
  Ogre::Vector3 axis2 = mouse_ray.getDirection().crossProduct( axis );

  //axis3 is perpendicular to axis and axis2, thus the normal of the plane
  //that contains the shortest connection line
  Ogre::Vector3 normal = axis2.crossProduct( mouse_ray.getDirection() );

  Ogre::Plane mouse_plane( normal, mouse_ray.getOrigin() );
  Ogre::Ray axis_ray( scene_node_->getPosition() - 1000*axis, axis );

  std::pair<bool,float> result = axis_ray.intersects( mouse_plane );

  //ROS_INFO_STREAM( "pos " << getPosition() << " dir " << axis_vec << " t " << result.second);

  ray_t = result.second;
  return result.first;
}


void InteractiveMarkerControl::addHighlightPass( S_MaterialPtr materials )
{
  S_MaterialPtr::iterator it;

  for ( it=materials.begin(); it!=materials.end(); it++ )
  {
    Ogre::MaterialPtr material = *it;
    Ogre::Pass *original_pass = material->getTechnique(0)->getPass(0);
    Ogre::Pass *pass = material->getTechnique(0)->createPass();

    pass->setSceneBlending( Ogre::SBT_ADD );
    pass->setDepthWriteEnabled( false );
    pass->setDepthCheckEnabled( false );
    pass->setLightingEnabled(true);
    pass->setAmbient( 0,0,0 );
    pass->setDiffuse( 0,0,0,0 );
    pass->setSpecular( 0,0,0,0 );
    pass->setCullingMode( original_pass->getCullingMode() );

    highlight_passes_.insert(pass);
  }
}

}
