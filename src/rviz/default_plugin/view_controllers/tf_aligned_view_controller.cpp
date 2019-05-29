/*
 * Copyright (c) 2019, PickNik, LLC.
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
 *     * Neither the name of PickNik, LLC nor the names of its
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

#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreCamera.h>

#include "rviz/viewport_mouse_event.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"

#include "tf_aligned_view_controller.h"

namespace rviz
{

TfAlignedViewController::TfAlignedViewController()
{
  axis_property_ = new EnumProperty("Align to", "z-axis", "Align the camera to this axis.", this);
  axis_property_->addOptionStd("x-axis", 1);
  axis_property_->addOptionStd("y-axis", 2);
  axis_property_->addOptionStd("z-axis", 3);

  invert_axis_ = new BoolProperty("Invert Axis", true, "Align camera with the negative axis.", this);
  camera_roll_property_ = new FloatProperty("Camera Roll", 0, "Roll about the camera's view axis.", this);
}

TfAlignedViewController::~TfAlignedViewController()
{
}

void TfAlignedViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();
  camera_->setProjectionType( Ogre::PT_PERSPECTIVE );
  invert_z_->hide();
}

void TfAlignedViewController::reset()
{
  camera_->setOrientation(reference_orientation_);
  camera_->setPosition(reference_position_);
  updateCamera();
}

void TfAlignedViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update( dt, ros_dt );
  updateCamera();
}

void TfAlignedViewController::onTargetFrameChanged(const Ogre::Vector3& /* old_reference_position */,
                                                   const Ogre::Quaternion& /* old_reference_orientation */)
{
  camera_->setOrientation(reference_orientation_);
  camera_->setPosition(reference_position_);
}

void TfAlignedViewController::updateCamera()
{
  Ogre::Quaternion q_offset;
  int view_direction = invert_axis_->getBool() ? -1 : 1;
  int view_axis = axis_property_->getOptionInt() * view_direction;

  // reference_orientation_ will be aligned to -z-axis by default.
  switch (view_axis) {
  case -3: // -z-axis
    // Do nothing, this is the default case
    break;
  case -2: // -y-axis
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X);
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_Z);
    break;
  case -1: // -x-axis
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y);
    break;
  case 1: // x-axis
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
    break;
  case 2: // y-axis
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
    break;
  case 3: // z-axis
    q_offset = q_offset * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_Y);
    break;
  }

  // Add camera roll, clockwise positive for all axes
  q_offset = q_offset * Ogre::Quaternion(Ogre::Radian(camera_roll_property_->getFloat()),
                                         Ogre::Vector3::UNIT_Z);

  camera_->setOrientation(reference_orientation_ * q_offset);
  camera_->setPosition(Ogre::Vector3::ZERO);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::TfAlignedViewController, rviz::ViewController )
