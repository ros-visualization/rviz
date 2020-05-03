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

// helper function to create axis strings from option ID
inline QString fmtAxis(int i)
{
  return QStringLiteral("%1%2 axis").arg(QChar(i % 2 ? '+' : '-')).arg(QChar('x' + (i - 1) / 2));
}

FrameViewController::FrameViewController()
{
  axis_property_ = new EnumProperty("Point towards", fmtAxis(6),
                                    "Point the camera along the given axis of the frame.", nullptr,
                                    SLOT(changedAxis()), this);
  axis_property_->addOption(ANY_AXIS, -1);
  this->addChild(axis_property_, yaw_property_->rowNumberInParent());
  // x,y,z axes get integers from 1..6: +x, -x, +y, -y, +z, -z
  for (int i = 1; i <= 6; ++i)
    axis_property_->addOption(fmtAxis(i), i);
  previous_axis_ = axis_property_->getOptionInt();

  locked_property_ = new BoolProperty("Lock Camera", false,
                                      "Lock camera in its current pose relative to the frame", this);
}

void FrameViewController::onInitialize()
{
  FPSViewController::onInitialize();
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

void FrameViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if (locked_property_->getBool())
  {
    setStatus("Unlock camera in settings to enable mouse interaction.");
    return;
  }
  FPSViewController::handleMouseEvent(event);
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

void FrameViewController::changedOrientation()
{
  FPSViewController::changedOrientation();
  setAxisFromCamera();
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
