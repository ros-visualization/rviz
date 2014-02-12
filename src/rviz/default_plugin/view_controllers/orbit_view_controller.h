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

#ifndef RVIZ_ORBIT_VIEW_CONTROLLER_H
#define RVIZ_ORBIT_VIEW_CONTROLLER_H

#include <OgreVector3.h>

#include <QCursor>

#include "rviz/frame_position_tracking_view_controller.h"

namespace rviz
{
class FloatProperty;
class Shape;
class SceneNode;
class VectorProperty;

/**
 * \brief An orbital camera, controlled by yaw, pitch, distance, and focal point
 *
 * This camera is based on the equation of a sphere in spherical coordinates:
 @verbatim
 x = d*cos(theta)sin(phi)
 y = d*cos(phi)
 z = d*sin(theta)sin(phi)
 @endverbatim
 * Where:<br>
 * d = #distance_<br>
 * theta = #yaw_<br>
 * phi = #pitch_
 */
class OrbitViewController: public FramePositionTrackingViewController
{
Q_OBJECT
public:
  OrbitViewController();
  virtual ~OrbitViewController();

  /** @brief Do subclass-specific initialization.  Called by
   * ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set. */
  virtual void onInitialize();

  /**
   * \brief Move in/out from the focal point, ie. adjust #distance_ by amount
   * @param amount The distance to move.  Positive amount moves towards the focal point, negative moves away
   */
  void zoom( float amount );
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
  virtual void update(float dt, float ros_dt);
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  /**
   * \brief Calculates pitch and yaw values given a new position and the current focal point
   * @param position Position to calculate the pitch/yaw for
   */
  void calculatePitchYawFromPosition( const Ogre::Vector3& position );

  virtual void updateCamera();

  FloatProperty* yaw_property_;                         ///< The camera's yaw (rotation around the y-axis), in radians
  FloatProperty* pitch_property_;                       ///< The camera's pitch (rotation around the x-axis), in radians
  FloatProperty* distance_property_;                    ///< The camera's distance from the focal point
  VectorProperty* focal_point_property_; ///< The point around which the camera "orbits".

  Shape* focal_shape_;
  bool dragging_;
};

}

#endif // RVIZ_VIEW_CONTROLLER_H
