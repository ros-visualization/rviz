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

#ifndef RVIZ_FRAME_POSITION_TRACKING_VIEW_CONTROLLER_H
#define RVIZ_FRAME_POSITION_TRACKING_VIEW_CONTROLLER_H

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "rviz/view_controller.h"
#include "rviz/rviz_export.h"

namespace rviz
{
class TfFrameProperty;

/** @brief Base class of ViewControllers which have a "Target Frame"
 * which is a TF frame whose position they track. */
class RVIZ_EXPORT FramePositionTrackingViewController : public ViewController
{
  Q_OBJECT
public:
  FramePositionTrackingViewController();
  ~FramePositionTrackingViewController() override;

  /** @brief Do subclass-specific initialization.
   *
   * Called by ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set. */
  void onInitialize() override;

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.
   * This version calls updateTargetSceneNode(). */
  void onActivate() override;

  void update(float dt, float ros_dt) override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera().
   *
   * This base class implementation does nothing. */
  void mimic(ViewController* source_view) override;

protected Q_SLOTS:
  /** @brief Called when Target Frame property changes while view is
   * active.  Purpose is to change values in the view controller (like
   * a position offset) such that the actual viewpoint does not
   * change.  Calls updateTargetSceneNode() and
   * onTargetFrameChanged(). */
  virtual void updateTargetFrame();

protected:
  /** @brief Override to implement the change in properties which
   * nullifies the change in target frame.
   * @see updateTargetFrame() */
  virtual void onTargetFrameChanged(const Ogre::Vector3& /*old_reference_position*/,
                                    const Ogre::Quaternion& /*old_reference_orientation*/)
  {
  }

  bool getNewTransform();

  /** @brief Update the position of the target_scene_node_ from the TF
   * frame specified in the Target Frame property. */
  virtual void updateTargetSceneNode();

  TfFrameProperty* target_frame_property_;
  Ogre::SceneNode* target_scene_node_;
  Ogre::Quaternion reference_orientation_;
  Ogre::Vector3 reference_position_;
};

} // end namespace rviz

#endif // RVIZ_FRAME_POSITION_TRACKING_VIEW_CONTROLLER_H
