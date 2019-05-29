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

#ifndef RVIZ_TF_ALIGNED_VIEW_CONTROLLER_H
#define RVIZ_TF_ALIGNED_VIEW_CONTROLLER_H

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include "rviz/frame_position_tracking_view_controller.h"

namespace rviz
{
class EnumProperty;
class FloatProperty;
class BoolPropery;

/** @brief A camera tied to a given frame. */
class TfAlignedViewController : public FramePositionTrackingViewController
{
Q_OBJECT
public:
  TfAlignedViewController();
  virtual ~TfAlignedViewController();

  virtual void onInitialize();
  virtual void reset();
  virtual void update(float dt, float ros_dt);

protected:
  virtual void onTargetFrameChanged(const Ogre::Vector3& /* old_reference_position */,
                                    const Ogre::Quaternion& /* old_reference_orientation */);

  void updateCamera();

  EnumProperty* axis_property_;  // The axis that the camera aligns to
  BoolProperty* invert_axis_;  // Flag for positive or negative direction along axis
  FloatProperty* camera_roll_property_;  // Amount to roll the camera from default position
};

} // end namespace rviz

#endif // RVIZ_TF_ALIGNED_VIEW_CONTROLLER_H
