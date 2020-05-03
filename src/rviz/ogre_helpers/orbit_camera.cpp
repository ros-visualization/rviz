/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "orbit_camera.h"
#include "shape.h"

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreViewport.h>

#include <cmath>
#include <stdint.h>

#include <sstream>

#define MIN_DISTANCE 0.01

namespace rviz
{
static const float PITCH_LIMIT_LOW = 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::PI - 0.001;
static const float YAW_START = Ogre::Math::PI; // - 0.001;
static const float PITCH_START = Ogre::Math::HALF_PI;

OrbitCamera::OrbitCamera(Ogre::SceneManager* scene_manager)
  : CameraBase(scene_manager)
  , focal_point_(Ogre::Vector3::ZERO)
  , yaw_(YAW_START)
  , pitch_(PITCH_START)
  , distance_(10.0f)
{
  focal_point_object_ = new Shape(Shape::Sphere, scene_manager);
  focal_point_object_->setScale(Ogre::Vector3(0.05f, 0.01f, 0.05f));
  focal_point_object_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
  focal_point_object_->getRootNode()->setVisible(false);

  update();
}

OrbitCamera::~OrbitCamera()
{
  delete focal_point_object_;
}

void OrbitCamera::normalizePitch()
{
  if (pitch_ < PITCH_LIMIT_LOW)
  {
    pitch_ = PITCH_LIMIT_LOW;
  }
  else if (pitch_ > PITCH_LIMIT_HIGH)
  {
    pitch_ = PITCH_LIMIT_HIGH;
  }
}

void OrbitCamera::normalizeYaw()
{
  yaw_ = fmod(yaw_, Ogre::Math::TWO_PI);

  if (yaw_ < 0.0f)
  {
    yaw_ = Ogre::Math::TWO_PI + yaw_;
  }
}

Ogre::Vector3 OrbitCamera::getGlobalFocalPoint()
{
  Ogre::Vector3 global_focal_point = focal_point_;

  if (relative_node_)
  {
    global_focal_point = relative_node_->getOrientation() * focal_point_ + relative_node_->getPosition();
  }

  return global_focal_point;
}

void OrbitCamera::update()
{
  Ogre::Vector3 global_focal_point = getGlobalFocalPoint();

  float x = distance_ * std::cos(yaw_) * std::sin(pitch_) + global_focal_point.x;
  float y = distance_ * std::cos(pitch_) + global_focal_point.y;
  float z = distance_ * std::sin(yaw_) * std::sin(pitch_) + global_focal_point.z;

  Ogre::Vector3 pos(x, y, z);

  if (relative_node_)
  {
    Ogre::Vector3 vec = pos - global_focal_point;
    pos = relative_node_->getOrientation() * vec + global_focal_point;

    camera_->setFixedYawAxis(true, relative_node_->getOrientation() * Ogre::Vector3::UNIT_Y);
  }

  camera_->setPosition(pos);
  camera_->lookAt(global_focal_point);

  focal_point_object_->setPosition(global_focal_point);
}

void OrbitCamera::yaw(float angle)
{
  yaw_ += angle;

  normalizeYaw();

  update();
}

void OrbitCamera::pitch(float angle)
{
  pitch_ += angle;

  normalizePitch();

  update();
}

void OrbitCamera::roll(float /*angle*/)
{
}

Ogre::Vector3 OrbitCamera::getPosition()
{
  return camera_->getPosition();
}

Ogre::Quaternion OrbitCamera::getOrientation()
{
  return camera_->getOrientation();
}

void OrbitCamera::calculatePitchYawFromPosition(const Ogre::Vector3& position)
{
  float x = position.x - focal_point_.x;
  float y = position.y - focal_point_.y;
  pitch_ = std::acos(y / distance_);

  normalizePitch();

  float val = x / (distance_ * std::sin(pitch_));

  yaw_ = std::acos(val);

  Ogre::Vector3 direction = focal_point_ - position;

  if (direction.dotProduct(Ogre::Vector3::NEGATIVE_UNIT_Z) < 0)
  {
    yaw_ = Ogre::Math::TWO_PI - yaw_;
  }
}

void OrbitCamera::setFrom(CameraBase* camera)
{
  Ogre::Vector3 position = camera->getPosition();
  Ogre::Quaternion orientation = camera->getOrientation();

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);
  focal_point_ = position + direction;

  calculatePitchYawFromPosition(position);

  update();
}

void OrbitCamera::setOrientation(float x, float y, float z, float w)
{
  Ogre::Vector3 position = camera_->getPosition();
  Ogre::Quaternion orientation(w, x, y, z);

  Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_);
  focal_point_ = position + direction;

  calculatePitchYawFromPosition(position);

  update();
}

void OrbitCamera::zoom(float amount)
{
  distance_ -= amount;

  if (distance_ <= MIN_DISTANCE)
  {
    distance_ = MIN_DISTANCE;
  }

  update();
}

void OrbitCamera::setFocalPoint(const Ogre::Vector3& focal_point)
{
  focal_point_ = focal_point;

  update();
}

void OrbitCamera::move(float x, float y, float z)
{
  Ogre::Quaternion orientation = camera_->getOrientation();

  if (relative_node_)
  {
    orientation = relative_node_->getOrientation().Inverse() * orientation;
  }

  focal_point_ += orientation * Ogre::Vector3(x, y, z);

  update();
}

void OrbitCamera::setPosition(float x, float y, float z)
{
  Ogre::Vector3 pos(x, y, z);
  distance_ = (pos - getGlobalFocalPoint()).length();

  calculatePitchYawFromPosition(Ogre::Vector3(x, y, z));

  update();
}

void OrbitCamera::lookAt(const Ogre::Vector3& point)
{
  Ogre::Vector3 focal_point = point;
  Ogre::Vector3 camera_position = camera_->getPosition();

  if (relative_node_)
  {
    Ogre::Vector3 rel_pos = relative_node_->getPosition();
    Ogre::Quaternion rel_orient = relative_node_->getOrientation();

    focal_point = rel_orient.Inverse() * (focal_point - rel_pos);
    camera_position = rel_orient.Inverse() * (camera_position - rel_pos);
  }

  distance_ = focal_point.distance(camera_position);
  focal_point_ = focal_point;

  update();
}

void OrbitCamera::mouseLeftDrag(int diff_x, int diff_y, bool /*ctrl*/, bool /*alt*/, bool /*shift*/)
{
  yaw(diff_x * 0.005);
  pitch(-diff_y * 0.005);
}

void OrbitCamera::mouseMiddleDrag(int diff_x, int diff_y, bool /*ctrl*/, bool /*alt*/, bool /*shift*/)
{
  float fovY = camera_->getFOVy().valueRadians();
  float fovX = 2.0f * std::atan(std::tan(fovY / 2.0f) * camera_->getAspectRatio());

  int width = camera_->getViewport()->getActualWidth();
  int height = camera_->getViewport()->getActualHeight();

  move(-((float)diff_x / (float)width) * distance_ * std::tan(fovX / 2.0f) * 2.0f,
       ((float)diff_y / (float)height) * distance_ * std::tan(fovY / 2.0f) * 2.0f, 0.0f);
}

void OrbitCamera::mouseRightDrag(int /*diff_x*/, int diff_y, bool /*ctrl*/, bool /*alt*/, bool shift)
{
  if (shift)
  {
    move(0.0f, 0.0f, diff_y * 0.1 * (distance_ / 10.0f));
  }
  else
  {
    zoom(-diff_y * 0.1 * (distance_ / 10.0f));
  }
}

void OrbitCamera::scrollWheel(int diff, bool /*ctrl*/, bool /*alt*/, bool shift)
{
  if (shift)
  {
    move(0.0f, 0.0f, -diff * 0.01 * (distance_ / 10.0f));
  }
  else
  {
    zoom(diff * 0.01 * (distance_ / 10.0f));
  }
}

void OrbitCamera::mouseLeftDown(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(true);
}

void OrbitCamera::mouseMiddleDown(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(true);
}

void OrbitCamera::mouseRightDown(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(true);
}

void OrbitCamera::mouseLeftUp(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(false);
}

void OrbitCamera::mouseMiddleUp(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(false);
}

void OrbitCamera::mouseRightUp(int /*x*/, int /*y*/)
{
  focal_point_object_->getRootNode()->setVisible(false);
}

void OrbitCamera::fromString(const std::string& str)
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

  update();
}

std::string OrbitCamera::toString()
{
  std::ostringstream oss;
  oss << pitch_ << " " << yaw_ << " " << distance_ << " " << focal_point_.x << " " << focal_point_.y
      << " " << focal_point_.z;

  return oss.str();
}

} // namespace rviz
