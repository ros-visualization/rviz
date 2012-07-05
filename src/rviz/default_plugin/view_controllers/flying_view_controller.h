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

#ifndef RVIZ_FPS_VIEW_CONTROLLER_H
#define RVIZ_FPS_VIEW_CONTROLLER_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "rviz/view_controller.h"

namespace rviz
{

class SceneNode;
class Shape;
class FloatProperty;
class VectorProperty;
class QuaternionProperty;

/** @brief An un-constrained "flying" camera, controlled by a position and quaternion. */
class FlyingViewController : public ViewController
{
Q_OBJECT
public:
  FlyingViewController();
  virtual ~FlyingViewController();

  virtual void onInitialize();

  void yaw( float angle );
  void pitch( float angle );
  void move( float x, float y, float z );

  virtual void handleMouseEvent(ViewportMouseEvent& evt);

  virtual void lookAt( const Ogre::Vector3& point );

  virtual void reset();

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera(). */
  virtual void mimic( ViewController* source_view );

protected:
  virtual void onUpdate(float dt, float ros_dt);
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  void setPropertiesFromCamera( Ogre::Camera* source_camera );

  void updateCamera();

  Ogre::Quaternion getOrientation(); ///< Return a Quaternion based on the yaw and pitch properties.

  FloatProperty* yaw_property_;                         ///< The camera's yaw (rotation around the y-axis), in radians
  FloatProperty* pitch_property_;                       ///< The camera's pitch (rotation around the x-axis), in radians
  QuaternionProperty* quaternion_property_;             ///< The quaternion defining the camera rotation matrix
  VectorProperty* position_property_;                   ///< The position of the camera.
  FloatProperty* transitionTime_;
};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
