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

#include "simple_orbit_view_controller.h"
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

#include <ogre_tools/axes.h>

#include <stdint.h>
#include <sstream>

namespace rviz
{

static const float MIN_DISTANCE = 0.01;
static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI / 2.0;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5;

// move camera up so the focal point appears in the lower image half
static const float CAMERA_OFFSET = 0.2;


SimpleOrbitViewController::SimpleOrbitViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node)
: ViewController(manager, name, target_scene_node)
, focal_point_( Ogre::Vector3::ZERO )
, yaw_( YAW_START )
, pitch_( PITCH_START )
, distance_( 10.0f )
{
  axes_ = new ogre_tools::Axes( manager->getSceneManager(), target_scene_node, 1.0, 0.05 );
  axes_->getSceneNode()->setVisible( false );

  axes_->setOrientation( target_scene_node_->getOrientation().Inverse() );
}

SimpleOrbitViewController::~SimpleOrbitViewController()
{
  delete axes_;
}

bool SimpleOrbitViewController::intersectGroundPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d )
{
  //convert rays into reference frame
  mouse_ray.setOrigin( target_scene_node_->convertWorldToLocalPosition( mouse_ray.getOrigin() ) );
  mouse_ray.setDirection( target_scene_node_->convertWorldToLocalOrientation( Ogre::Quaternion::IDENTITY ) * mouse_ray.getDirection() );

  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Y, 0 );

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
  if (!intersection.first)
  {
    return false;
  }

  intersection_3d = mouse_ray.getPoint(intersection.second);
  return true;
}

void SimpleOrbitViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  bool moved = false;
  if ( event.event.LeftDown() || event.event.RightDown() || event.event.MiddleDown() )
  {
    axes_->getSceneNode()->setVisible(true);
    moved = true;
  }
  else if ( event.event.LeftUp() || event.event.RightUp() || event.event.MiddleUp() )
  {
    axes_->getSceneNode()->setVisible(false);
    moved = true;
  }
  else if ( event.event.Dragging() )
  {
    int32_t diff_x = event.event.GetX() - event.last_x;
    int32_t diff_y = event.event.GetY() - event.last_y;

    if ( event.event.LeftIsDown() )
    {
      yaw( diff_x*0.005 );
      pitch( -diff_y*0.005 );
    }
    else if ( event.event.MiddleIsDown() || (event.event.RightIsDown() && event.event.ShiftDown()) )
    {
      // handle mouse movement
      int width = event.viewport->getActualWidth();
      int height = event.viewport->getActualHeight();

      Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
          (float) event.event.GetX() / (float) width, (float) event.event.GetY()/ (float) height);

      Ogre::Ray last_mouse_ray =
          event.viewport->getCamera()->getCameraToViewportRay(
          (float) event.last_x / (float) width, (float) event.last_y / (float) height);

      Ogre::Vector3 last_intersect, intersect;

      if ( intersectGroundPlane( last_mouse_ray, last_intersect ) && intersectGroundPlane( mouse_ray, intersect ) )
      {
        focal_point_ += last_intersect - intersect;
      }
    }
    else if ( event.event.RightIsDown() )
    {
      zoom( -diff_y * 0.1 * (distance_ / 10.0f) );
    }

    moved = true;
  }

  if ( event.event.GetWheelRotation() != 0 )
  {
    int diff = (float)event.event.GetWheelRotation() / (float)event.event.GetWheelDelta();
    zoom( diff * 0.1 * distance_ );
    moved = true;
  }

  if (moved)
  {
    manager_->queueRender();
  }
}

void SimpleOrbitViewController::onActivate()
{
  if (camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
  {
    camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  }
  else
  {
    // do some trigonometry
    Ogre::Ray camera_dir_ray( camera_->getRealPosition(), camera_->getRealDirection() );
    Ogre::Ray camera_down_ray( camera_->getRealPosition(), -1.0 * camera_->getRealUp() );

    Ogre::Vector3 a,b;

    if ( intersectGroundPlane( camera_dir_ray, b ) &&
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

void SimpleOrbitViewController::onDeactivate()
{
  axes_->getSceneNode()->setVisible(false);
  camera_->setFixedYawAxis(false);
}

void SimpleOrbitViewController::onUpdate(float dt, float ros_dt)
{
  updateCamera();
  axes_->setScale( Ogre::Vector3(distance_,distance_,distance_) * 0.05 );
}

void SimpleOrbitViewController::lookAt( const Ogre::Vector3& point )
{
  Ogre::Vector3 camera_position = camera_->getPosition();
  focal_point_ = target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  distance_ = focal_point_.distance( camera_position );

  calculatePitchYawFromPosition(camera_position);
}

void SimpleOrbitViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  // Compute camera's global coords
  Ogre::Vector3 old_camera_pos = old_reference_position + old_reference_orientation * camera_->getPosition();
//  Ogre::Vector3 old_camera_orient = old_reference_orientation * camera_->getOrientation();

  // Compute camera's new coords relative to the reference frame
  Ogre::Vector3 camera_position = target_scene_node_->getOrientation().Inverse() * (old_camera_pos - target_scene_node_->getPosition());

  focal_point_ = Ogre::Vector3::ZERO;
  distance_ = focal_point_.distance( camera_position );

  calculatePitchYawFromPosition(camera_position);
}

void SimpleOrbitViewController::normalizePitch()
{
  if ( pitch_ < PITCH_LIMIT_LOW )
  {
    pitch_ = PITCH_LIMIT_LOW;
  }
  else if ( pitch_ > PITCH_LIMIT_HIGH )
  {
    pitch_ = PITCH_LIMIT_HIGH;
  }
}

void SimpleOrbitViewController::normalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void SimpleOrbitViewController::updateCamera()
{
  float x = distance_ * cos( yaw_ ) * sin( pitch_ ) + focal_point_.x;
  float y = distance_ * cos( pitch_ ) + focal_point_.y;
  float z = distance_ * sin( yaw_ ) * sin( pitch_ ) + focal_point_.z;

  Ogre::Vector3 pos( x, y, z );

  camera_->setFixedYawAxis(true, target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Y);
  camera_->setDirection(target_scene_node_->getOrientation() * (focal_point_ - pos));

  pos += camera_->getUp() * distance_ * CAMERA_OFFSET;

  camera_->setPosition(pos);

//  ROS_INFO_STREAM( camera_->getDerivedDirection() );

  axes_->setPosition(focal_point_);
}

void SimpleOrbitViewController::yaw( float angle )
{
  yaw_ += angle;

  normalizeYaw();
}

void SimpleOrbitViewController::pitch( float angle )
{
  pitch_ += angle;

  normalizePitch();
}

void SimpleOrbitViewController::calculatePitchYawFromPosition( const Ogre::Vector3& position )
{
  float x = position.x - focal_point_.x;
  float y = position.y - focal_point_.y;
  pitch_ = acos( y / distance_ );

  normalizePitch();

  float val = x / ( distance_ * sin( pitch_ ) );

  yaw_ = acos( val );

  Ogre::Vector3 direction = focal_point_ - position;

  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    yaw_ = Ogre::Math::TWO_PI - yaw_;
  }
}

void SimpleOrbitViewController::zoom( float amount )
{
  distance_ -= amount;

  if ( distance_ <= MIN_DISTANCE )
  {
    distance_ = MIN_DISTANCE;
  }
}

void SimpleOrbitViewController::fromString(const std::string& str)
{
  std::istringstream iss(str);

  iss >> pitch_;
  iss.ignore();
  iss >> yaw_;
  iss.ignore();
  iss >> distance_;
  iss.ignore();
  iss >> focal_point_.x;
  iss.ignore();
  iss >> focal_point_.y;
  iss.ignore();
  iss >> focal_point_.z;
}

std::string SimpleOrbitViewController::toString()
{
  std::ostringstream oss;
  oss << pitch_ << " " << yaw_ << " " << distance_ << " " << focal_point_.x << " " << focal_point_.y << " " << focal_point_.z;

  return oss.str();
}


}
