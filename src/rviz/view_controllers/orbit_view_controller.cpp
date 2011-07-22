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

#include "orbit_view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreViewport.h>

#include <ogre_tools/shape.h>

#include <stdint.h>
#include <sstream>

namespace rviz
{

static const float MIN_DISTANCE = 0.01;
static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI / 2.0;
static const float YAW_START = Ogre::Math::HALF_PI * 0.5;

OrbitViewController::OrbitViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node)
: ViewController(manager, name, target_scene_node)
, focal_point_( Ogre::Vector3::ZERO )
, yaw_( YAW_START )
, pitch_( PITCH_START )
, distance_( 10.0f )
{
  focal_shape_ = new ogre_tools::Shape(ogre_tools::Shape::Sphere, manager_->getSceneManager(), target_scene_node_);
  focal_shape_->setScale(Ogre::Vector3(0.05f, 0.01f, 0.05f));
  focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_shape_->getRootNode()->setVisible(false);
}

OrbitViewController::~OrbitViewController()
{
  delete focal_shape_;
}

void OrbitViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  bool moved = false;
  if ( event.event.LeftDown() || event.event.RightDown() || event.event.MiddleDown() )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
  }
  else if ( event.event.LeftUp() || event.event.RightUp() || event.event.MiddleUp() )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
  }
  else if ( event.event.Dragging() )
  {
    int32_t diff_x = event.event.GetX() - event.last_x;
    int32_t diff_y = event.event.GetY() - event.last_y;

    if ( event.event.LeftIsDown() && !event.event.ShiftDown() )
    {
      yaw( diff_x*0.005 );
      pitch( -diff_y*0.005 );
    }
    else if ( event.event.MiddleIsDown() || 
	      ( event.event.ShiftDown() && event.event.LeftIsDown() ))
    {
      float fovY = camera_->getFOVy().valueRadians();
      float fovX = 2.0f * atan( tan( fovY / 2.0f ) * camera_->getAspectRatio() );

      int width = camera_->getViewport()->getActualWidth();
      int height = camera_->getViewport()->getActualHeight();

      move( -((float)diff_x / (float)width) * distance_ * tan( fovX / 2.0f ) * 2.0f, ((float)diff_y / (float)height) * distance_ * tan( fovY / 2.0f ) * 2.0f, 0.0f );
    }
    else if ( event.event.RightIsDown() )
    {
      if (event.event.ShiftDown())
      {
        move(0.0f, 0.0f, diff_y * 0.1 * (distance_ / 10.0f));
      }
      else
      {
        zoom( -diff_y * 0.1 * (distance_ / 10.0f) );
      }
    }

    moved = true;
  }

  if ( event.event.GetWheelRotation() != 0 )
  {
    int diff = event.event.GetWheelRotation();
    if (event.event.ShiftDown())
    {
      move(0.0f, 0.0f, -diff * 0.01 * (distance_ / 10.0f));
    }
    else
    {
      zoom( diff * 0.01 * (distance_ / 10.0f) );
    }

    moved = true;
  }

  if (moved)
  {
    manager_->queueRender();
  }
}

void OrbitViewController::onActivate()
{
  if (camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
  {
    camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  }
  else
  {
    Ogre::Vector3 position = camera_->getPosition();
    Ogre::Quaternion orientation = camera_->getOrientation();

    // Determine the distance from here to the reference frame, and use that as the distance our focal point should be at
    distance_ = position.length();

    Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);
    focal_point_ = position + direction;

    calculatePitchYawFromPosition( position );
  }
}

void OrbitViewController::onDeactivate()
{
  focal_shape_->getRootNode()->setVisible(false);
  camera_->setFixedYawAxis(false);
}

void OrbitViewController::onUpdate(float dt, float ros_dt)
{
  updateCamera();
}

void OrbitViewController::lookAt( const Ogre::Vector3& point )
{
  Ogre::Vector3 camera_position = camera_->getPosition();
  focal_point_ = target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  distance_ = focal_point_.distance( camera_position );

  calculatePitchYawFromPosition(camera_position);
}

void OrbitViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
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

void OrbitViewController::normalizePitch()
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

void OrbitViewController::normalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void OrbitViewController::updateCamera()
{
  float x = distance_ * cos( yaw_ ) * sin( pitch_ ) + focal_point_.x;
  float y = distance_ * cos( pitch_ ) + focal_point_.y;
  float z = distance_ * sin( yaw_ ) * sin( pitch_ ) + focal_point_.z;

  Ogre::Vector3 pos( x, y, z );

  camera_->setPosition(pos);
  camera_->setFixedYawAxis(true, target_scene_node_->getOrientation() * Ogre::Vector3::UNIT_Y);
  camera_->setDirection(target_scene_node_->getOrientation() * (focal_point_ - pos));

//  ROS_INFO_STREAM( camera_->getDerivedDirection() );

  focal_shape_->setPosition(focal_point_);
}

void OrbitViewController::yaw( float angle )
{
  yaw_ += angle;

  normalizeYaw();
}

void OrbitViewController::pitch( float angle )
{
  pitch_ += angle;

  normalizePitch();
}

void OrbitViewController::calculatePitchYawFromPosition( const Ogre::Vector3& position )
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

void OrbitViewController::zoom( float amount )
{
  distance_ -= amount;

  if ( distance_ <= MIN_DISTANCE )
  {
    distance_ = MIN_DISTANCE;
  }
}

void OrbitViewController::move( float x, float y, float z )
{
  focal_point_ += camera_->getOrientation() * Ogre::Vector3( x, y, z );
}

void OrbitViewController::fromString(const std::string& str)
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

std::string OrbitViewController::toString()
{
  std::ostringstream oss;
  oss << pitch_ << " " << yaw_ << " " << distance_ << " " << focal_point_.x << " " << focal_point_.y << " " << focal_point_.z;

  return oss.str();
}


}
