/*
 * Copyright (c) 2019, Bielefeld University
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
 *     * Neither the name of Bielefeld University nor the names of its
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

#include "frame_view_controller.h"
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <Eigen/Geometry>
#include <QSignalBlocker>

namespace rviz
{
static const QString ANY_AXIS("arbitrary");

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
    Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
    Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

// helper function to create axis strings from option ID
inline QString fmtAxis(int i)
{
  return QStringLiteral("%1%2 axis").arg(QChar(i % 2 ? '+' : '-')).arg(QChar('x' + (i - 1) / 2));
}

FrameViewController::FrameViewController()
{
  invert_z_->hide();

  axis_property_ = new EnumProperty("Point towards", fmtAxis(6),
                                    "Point the camera along the given axis of the frame.", this,
                                    SLOT(changedAxis()), this);
  axis_property_->addOption(ANY_AXIS, -1);
  // x,y,z axes get integers from 1..6: +x, -x, +y, -y, +z, -z
  for (int i = 1; i <= 6; ++i)
    axis_property_->addOption(fmtAxis(i), i);
  previous_axis_ = axis_property_->getOptionInt();

  yaw_property_ = new FloatProperty("Yaw", 0, "Rotation of the camera around the Z (up) axis.", this,
                                    SLOT(changedOrientation()), this);
  yaw_property_->setMax(Ogre::Math::PI);
  yaw_property_->setMin(-Ogre::Math::PI);

  pitch_property_ = new FloatProperty("Pitch", 0, "How much the camera is tipped downward.", this,
                                      SLOT(changedOrientation()), this);
  pitch_property_->setMax(Ogre::Math::PI);
  pitch_property_->setMin(-Ogre::Math::PI);

  roll_property_ = new FloatProperty("Roll", 0, "Rotation about the camera's view direction.", this,
                                     SLOT(changedOrientation()), this);
  roll_property_->setMax(Ogre::Math::PI);
  roll_property_->setMin(-Ogre::Math::PI);

  position_property_ = new VectorProperty("Position", Ogre::Vector3::ZERO, "Position of the camera.",
                                          this, SLOT(changedPosition()), this);

  locked_property_ = new BoolProperty("Lock Camera", false,
                                      "Lock camera in its current pose relative to the frame", this);
}

void FrameViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);
  changedPosition();
  changedAxis();
}

void FrameViewController::reset()
{
  camera_->setPosition(Ogre::Vector3::ZERO);
  Eigen::Vector3d axis(0, 0, 0);
  int option = previous_axis_;
  if (option >= 1 && option <= 6)
  {
    axis[(option - 1) / 2] = (option % 2) ? +1 : -1;
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), axis);
    camera_->setOrientation(Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()) * ROBOT_TO_CAMERA_ROTATION);
  }
  setPropertiesFromCamera();
}

void FrameViewController::mimic(ViewController* source_view)
{
  FramePositionTrackingViewController::mimic(source_view);
  auto source_camera = source_view->getCamera();
  camera_->setPosition(source_camera->getPosition());
  camera_->setOrientation(source_camera->getOrientation());
  setPropertiesFromCamera();
}

void FrameViewController::lookAt(const Ogre::Vector3& point)
{
  camera_->lookAt(target_scene_node_->convertWorldToLocalPosition(point));
  setPropertiesFromCamera();
}

void FrameViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if (locked_property_->getBool())
  {
    setStatus("Unlock camera in settings to enable mouse interaction.");
    return;
  }

  setStatus("<b>Left-Click:</b> Rotate Yaw/Pitch.  <b>Shift Left-Click</b>: Rotate Roll.  "
            "<b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z.");

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if (event.type == QEvent::MouseMove)
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
  }

  if (event.left() && !event.shift())
  {
    setCursor(Rotate3D);
    rotate(-diff_x * 0.005, diff_y * 0.005, 0.0f);
  }
  else if (event.left() && event.shift())
  {
    setCursor(Rotate2D);
    int cx = event.viewport->getActualWidth() / 2;
    int cy = event.viewport->getActualHeight() / 2;

    float roll = atan2(event.last_y - cy, event.last_x - cx) - atan2(event.y - cy, event.x - cx);
    if (std::isfinite(roll))
      rotate(0.0f, 0.0f, roll);
  }
  else if (event.middle())
  {
    setCursor(MoveXY);
    move(diff_x * 0.01, -diff_y * 0.01, 0.0f);
  }
  else if (event.right())
  {
    setCursor(MoveZ);
    move(0.0f, 0.0f, diff_y * 0.1);
  }
  else
  {
    setCursor(event.shift() ? Rotate2D : Rotate3D);
  }

  if (event.wheel_delta != 0)
  {
    int diff = event.wheel_delta;
    move(0.0f, 0.0f, -diff * 0.01);
  }
}

Ogre::Quaternion FrameViewController::getOrientation(float yaw, float pitch, float roll)
{
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  return Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()) * ROBOT_TO_CAMERA_ROTATION;
}

void FrameViewController::setPropertiesFromCamera()
{
  Ogre::Quaternion q = camera_->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse();
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  auto ypr = quat.toRotationMatrix().eulerAngles(2, 1, 0);

  QSignalBlocker blockY(yaw_property_);
  QSignalBlocker blockP(pitch_property_);
  QSignalBlocker blockR(roll_property_);
  QSignalBlocker block(position_property_);

  yaw_property_->setFloat(ypr[0]);
  pitch_property_->setFloat(ypr[1]);
  roll_property_->setFloat(ypr[2]);
  position_property_->setVector(camera_->getPosition());
}

int FrameViewController::actualCameraAxisOption(double precision) const
{
  // compare current camera direction with unit axes
  Ogre::Vector3 actual =
      (camera_->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse()) * Ogre::Vector3::UNIT_X;
  for (unsigned int i = 0; i < 3; ++i)
  {
    Ogre::Vector3 axis(0, 0, 0);
    axis[i] = 1.0;
    auto scalar_product = axis.dotProduct(actual);
    if (std::abs(scalar_product) > 1.0 - precision)
      return 1 + 2 * i + (scalar_product > 0 ? 0 : 1);
  }
  return -1;
}

void FrameViewController::setAxisFromCamera()
{
  int actual = actualCameraAxisOption();
  if (axis_property_->getOptionInt() == actual) // no change?
    return;

  QSignalBlocker block(axis_property_);
  axis_property_->setString(actual == -1 ? ANY_AXIS : fmtAxis(actual));
  rememberAxis(actual);
}

void FrameViewController::move(float x, float y, float z)
{
  Ogre::Vector3 translate(x, y, z);
  position_property_->add(camera_->getOrientation() * translate);
}

void FrameViewController::changedPosition()
{
  camera_->setPosition(position_property_->getVector());
  context_->queueRender();
}

inline void _rotate(FloatProperty* prop, float angle)
{
  if (angle == 0)
    return;

  QSignalBlocker block(prop);
  angle = fmod(prop->getFloat() + angle, Ogre::Math::TWO_PI);

  // map angle onto range -PI .. PI
  if (angle > Ogre::Math::PI)
    angle = -Ogre::Math::PI + fmod(angle, Ogre::Math::PI);
  if (angle < -Ogre::Math::PI)
    angle = Ogre::Math::PI + fmod(angle, Ogre::Math::PI);

  prop->setFloat(angle);
}

void FrameViewController::rotate(float yaw, float pitch, float roll)
{
  _rotate(yaw_property_, yaw);
  _rotate(pitch_property_, pitch);
  _rotate(roll_property_, roll);
  changedOrientation();
}

void FrameViewController::changedOrientation()
{
  camera_->setOrientation(getOrientation(yaw_property_->getFloat(), pitch_property_->getFloat(),
                                         roll_property_->getFloat()));
  setAxisFromCamera();
  context_->queueRender();
}

void FrameViewController::changedAxis()
{
  rememberAxis(axis_property_->getOptionInt());
  reset();
}

inline void FrameViewController::rememberAxis(int current)
{
  if (current >= 1) // remember previous axis selection
    previous_axis_ = current;
}

void FrameViewController::onTargetFrameChanged(const Ogre::Vector3& /* old_reference_position */,
                                               const Ogre::Quaternion& /* old_reference_orientation */)
{
  // don't adapt the camera pose to the old reference position, but just jump to new frame
}

void FrameViewController::updateTargetSceneNode()
{
  if (getNewTransform())
  {
    // track both, position and orientation of reference frame
    target_scene_node_->setPosition(reference_position_);
    target_scene_node_->setOrientation(reference_orientation_);
    context_->queueRender();
  }
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::FrameViewController, rviz::ViewController)
