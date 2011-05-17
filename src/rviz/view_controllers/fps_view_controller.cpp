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

#include "fps_view_controller.h"
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

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

FPSViewController::FPSViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node)
: ViewController(manager, name, target_scene_node)
, yaw_(0.0f)
, pitch_(0.0f)
{
}

FPSViewController::~FPSViewController()
{
}

void FPSViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  bool moved = false;
  if ( event.event.Dragging() )
  {
    int32_t diff_x = event.event.GetX() - event.last_x;
    int32_t diff_y = event.event.GetY() - event.last_y;

    if ( event.event.LeftIsDown() )
    {
      yaw( -diff_x*0.005 );
      pitch( -diff_y*0.005 );
    }
    else if ( event.event.MiddleIsDown() )
    {
      move( diff_x*0.01, -diff_y*0.01, 0.0f );
    }
    else if ( event.event.RightIsDown() )
    {
      move( 0.0f, 0.0f, diff_y*0.1 );
    }

    moved = true;
  }

  if ( event.event.GetWheelRotation() != 0 )
  {
    int diff = event.event.GetWheelRotation();
    move( 0.0f, 0.0f, -diff * 0.01 );

    moved = true;
  }

  if (moved)
  {
    manager_->queueRender();
  }
}

void FPSViewController::setOrientation(const Ogre::Quaternion& orientation)
{
  Ogre::Quaternion quat = orientation;
  yaw_ = quat.getYaw( false ).valueRadians();
  pitch_ = quat.getPitch( false ).valueRadians();

  Ogre::Vector3 direction = quat * Ogre::Vector3::NEGATIVE_UNIT_Z;
  if ( direction.dotProduct( Ogre::Vector3::NEGATIVE_UNIT_Z ) < 0 )
  {
    if ( pitch_ > Ogre::Math::HALF_PI )
    {
      pitch_ = -Ogre::Math::HALF_PI + (pitch_ - Ogre::Math::HALF_PI);
    }
    else if ( pitch_ < -Ogre::Math::HALF_PI )
    {
      pitch_ = Ogre::Math::HALF_PI - (-pitch_ - Ogre::Math::HALF_PI);
    }

    yaw_ = -yaw_;

    if ( direction.dotProduct( Ogre::Vector3::UNIT_X ) < 0 )
    {
      yaw_ -= Ogre::Math::PI;
    }
    else
    {
      yaw_ += Ogre::Math::PI;
    }
  }

  normalizePitch();
  normalizeYaw();
}

void FPSViewController::onActivate()
{
  if (camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
  {
    camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  }
  else
  {
    setOrientation(camera_->getOrientation());
  }
}

void FPSViewController::onDeactivate()
{
}

void FPSViewController::onUpdate(float dt, float ros_dt)
{
  updateCamera();
}

void FPSViewController::lookAt( const Ogre::Vector3& point )
{
  camera_->lookAt( point );
  setOrientation( camera_->getOrientation() );
}

void FPSViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  lookAt(target_scene_node_->getPosition());
}

void FPSViewController::normalizePitch()
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

void FPSViewController::normalizeYaw()
{
  yaw_ = fmod( yaw_, Ogre::Math::TWO_PI );

  if ( yaw_ < 0.0f )
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

void FPSViewController::updateCamera()
{
  Ogre::Matrix3 pitch, yaw;

  yaw.FromAxisAngle( Ogre::Vector3::UNIT_Y, Ogre::Radian( yaw_ ) );
  pitch.FromAxisAngle( Ogre::Vector3::UNIT_X, Ogre::Radian( pitch_ ) );

  camera_->setOrientation( yaw * pitch );
}

void FPSViewController::yaw( float angle )
{
  yaw_ += angle;

  normalizeYaw();
}

void FPSViewController::pitch( float angle )
{
  pitch_ += angle;

  normalizePitch();
}

void FPSViewController::move( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  camera_->setPosition( camera_->getPosition() + camera_->getOrientation() * translate );
}

void FPSViewController::fromString(const std::string& str)
{
  std::istringstream iss(str);

  iss >> pitch_;
  iss.ignore();
  iss >> yaw_;
  iss.ignore();

  Ogre::Vector3 vec;
  iss >> vec.x;
  iss.ignore();
  iss >> vec.y;
  iss.ignore();
  iss >> vec.z;
  iss.ignore();
  camera_->setPosition(vec);
}

std::string FPSViewController::toString()
{
  std::ostringstream oss;
  oss << pitch_ << " " << yaw_ << " " << camera_->getPosition().x << " " << camera_->getPosition().y << " " << camera_->getPosition().z;

  return oss.str();
}


}
