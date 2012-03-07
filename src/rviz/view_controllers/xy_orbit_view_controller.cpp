/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "xy_orbit_view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>

#include <ogre_helpers/shape.h>

#include <stdint.h>

namespace rviz
{

static const float MIN_DISTANCE = 0.01;
static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI / 2.0;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5;

// move camera up so the focal point appears in the lower image half
static const float CAMERA_OFFSET = 0.2;


XYOrbitViewController::XYOrbitViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node)
: OrbitViewController(manager, name, target_scene_node)
{
  focal_shape_->setColor(0.0f, 1.0f, 1.0f, 0.5f);
}

bool XYOrbitViewController::intersectGroundPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d )
{
  //convert rays into reference frame
  mouse_ray.setOrigin( target_scene_node_->convertWorldToLocalPosition( mouse_ray.getOrigin() ) );
  mouse_ray.setDirection( target_scene_node_->convertWorldToLocalOrientation( Ogre::Quaternion::IDENTITY ) * mouse_ray.getDirection() );

  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0 );

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
  if (!intersection.first)
  {
    return false;
  }

  intersection_3d = mouse_ray.getPoint(intersection.second);
  return true;
}

void XYOrbitViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  bool moved = false;
  if( event.type == QEvent::MouseButtonPress )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
  }
  else if( event.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
  }
  else if( event.type == QEvent::MouseMove )
  {
    int32_t diff_x = event.x - event.last_x;
    int32_t diff_y = event.y - event.last_y;

    if( diff_x != 0 || diff_y != 0 )
    {
      if( event.left() && !event.shift() )
      {
        yaw( diff_x*0.005 );
        pitch( -diff_y*0.005 );
      }
      else if( event.middle() || (event.left() && event.shift()) )
      {
        // handle mouse movement
        int width = event.viewport->getActualWidth();
        int height = event.viewport->getActualHeight();

        Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay( event.x / (float) width,
                                                                                   event.y / (float) height );

        Ogre::Ray last_mouse_ray =
          event.viewport->getCamera()->getCameraToViewportRay( event.last_x / (float) width,
                                                               event.last_y / (float) height );

        Ogre::Vector3 last_intersect, intersect;

        if( intersectGroundPlane( last_mouse_ray, last_intersect ) &&
            intersectGroundPlane( mouse_ray, intersect ))
        {
          Ogre::Vector3 motion = last_intersect - intersect;

          // When dragging near the horizon, the motion can get out of
          // control.  This throttles it to an arbitrary limit per mouse
          // event.
          float motion_distance_limit = 1; /*meter*/
          if( motion.length() > motion_distance_limit )
          {
            motion.normalise();
            motion *= motion_distance_limit;
          }

          focal_point_ += motion;
          emitConfigChanged();
        }
      }
      else if( event.right() )
      {
        zoom( -diff_y * 0.1 * (distance_ / 10.0f) );
      }

      moved = true;
    }
  }

  if( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;
    zoom( diff * 0.001 * distance_ );
    moved = true;
  }

  if( moved )
  {
    manager_->queueRender();
  }
}

void XYOrbitViewController::onActivate()
{
  if( camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC )
  {
    camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  }
  else
  {
    // do some trigonometry
    Ogre::Ray camera_dir_ray( camera_->getRealPosition(), camera_->getRealDirection() );
    Ogre::Ray camera_down_ray( camera_->getRealPosition(), -1.0 * camera_->getRealUp() );

    Ogre::Vector3 a,b;

    if( intersectGroundPlane( camera_dir_ray, b ) &&
        intersectGroundPlane( camera_down_ray, a ) )
    {
      float l_a = camera_->getPosition().distance( b );
      float l_b = camera_->getPosition().distance( a );

      distance_ = ( l_a * l_b ) / ( CAMERA_OFFSET * l_a + l_b );

      camera_dir_ray.setOrigin( camera_->getRealPosition() - camera_->getRealUp() * distance_ * CAMERA_OFFSET );
      intersectGroundPlane( camera_dir_ray, focal_point_ );

      ROS_INFO_STREAM( distance_ << " xx " << (camera_->getPosition() - camera_->getUp() * distance_ * CAMERA_OFFSET).distance( focal_point_ ) );

      calculatePitchYawFromPosition( camera_->getPosition() - camera_->getUp() * distance_ * CAMERA_OFFSET );
    }

    updateCamera();
  }
}

void XYOrbitViewController::updateCamera()
{
  OrbitViewController::updateCamera();
  camera_->setPosition( camera_->getPosition() + camera_->getUp() * distance_ * CAMERA_OFFSET );
}

void XYOrbitViewController::lookAt( const Ogre::Vector3& point )
{
  Ogre::Vector3 camera_position = camera_->getPosition();
  focal_point_ = target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  focal_point_.z = 0;
  distance_ = focal_point_.distance( camera_position );

  calculatePitchYawFromPosition(camera_position);
}

}
