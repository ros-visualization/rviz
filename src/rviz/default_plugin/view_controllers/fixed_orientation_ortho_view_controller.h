/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_FIXED_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
#define RVIZ_FIXED_ORIENTATION_ORTHO_VIEW_CONTROLLER_H

#include "rviz/frame_position_tracking_view_controller.h"

#include <OgreQuaternion.h>

namespace rviz
{
class FloatProperty;
class SceneNode;
class Shape;

class FixedOrientationOrthoViewController : public FramePositionTrackingViewController
{
  Q_OBJECT
public:
  FixedOrientationOrthoViewController();
  ~FixedOrientationOrthoViewController() override;

  void onInitialize() override;

  void handleMouseEvent(ViewportMouseEvent& evt) override;

  void lookAt(const Ogre::Vector3& point_rel_world) override;

  void reset() override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  void mimic(ViewController* source_view) override;

  void update(float dt, float ros_dt) override;

protected:
  void onTargetFrameChanged(const Ogre::Vector3& old_reference_position,
                            const Ogre::Quaternion& old_reference_orientation) override;

  /** Set the camera orientation based on angle_. */
  void orientCamera();

  void setPosition(const Ogre::Vector3& pos_rel_target);
  void move(float x, float y);
  void updateCamera();

  FloatProperty* scale_property_;
  FloatProperty* angle_property_;
  FloatProperty* x_property_;
  FloatProperty* y_property_;
  bool dragging_;
};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
