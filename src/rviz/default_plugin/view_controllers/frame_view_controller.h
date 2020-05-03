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

#ifndef RVIZ_FRAME_VIEW_CONTROLLER_H
#define RVIZ_FRAME_VIEW_CONTROLLER_H

#include <rviz/frame_position_tracking_view_controller.h>

namespace rviz
{
class FloatProperty;
class VectorProperty;
class EnumProperty;

/** @brief A camera tied to a given frame. */
class FrameViewController : public FramePositionTrackingViewController
{
  Q_OBJECT

public:
  FrameViewController();
  ~FrameViewController() override = default;
  void onInitialize() override;

  void reset() override;
  void mimic(ViewController* source_view) override;
  void lookAt(const Ogre::Vector3& point) override;
  void handleMouseEvent(ViewportMouseEvent& event) override;

  void move(float x, float y, float z);
  void rotate(float yaw, float pitch, float roll);

protected:
  void onTargetFrameChanged(const Ogre::Vector3& /* old_reference_position */,
                            const Ogre::Quaternion& /* old_reference_orientation */) override;
  void updateTargetSceneNode() override;

  Ogre::Quaternion getOrientation(float yaw, float pitch, float roll);
  /// set yaw, pitch, roll, position properties from camera
  void setPropertiesFromCamera();
  /// set axis_property_ from camera
  void setAxisFromCamera();
  /// find enum ID from camera's current pose
  int actualCameraAxisOption(double precision = 0.001) const;

  EnumProperty* axis_property_;       ///< The axis that the camera aligns to
  FloatProperty* yaw_property_;       ///< The camera's yaw (rotation around the z-axis), in radians
  FloatProperty* pitch_property_;     ///< The camera's pitch (rotation around the y-axis), in radians
  FloatProperty* roll_property_;      ///< The camera's roll (rotation around the x-axis), in radians
  VectorProperty* position_property_; ///< The camera's position
  BoolProperty* locked_property_;     ///< Lock camera, i.e. disable mouse interaction?

protected Q_SLOTS:
  void changedPosition();
  void changedOrientation();
  void changedAxis();

private:
  void rememberAxis(int current);
  int previous_axis_;
};

} // end namespace rviz

#endif // RVIZ_FRAME_VIEW_CONTROLLER_H
