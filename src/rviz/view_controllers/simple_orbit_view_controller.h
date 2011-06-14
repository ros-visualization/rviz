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

#ifndef RVIZ_SIMPLE_ORBIT_VIEW_CONTROLLER_H
#define RVIZ_SIMPLE_ORBIT_VIEW_CONTROLLER_H

#include "rviz/view_controller.h"

#include <OGRE/OgreVector3.h>

namespace ogre_tools
{
class Axes;
}

namespace rviz
{

/**
 * \brief Like the orbit view controller, but moves only in the y-z plane
 */
class SimpleOrbitViewController : public ViewController
{
public:

  SimpleOrbitViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node);
  virtual ~SimpleOrbitViewController();

  /**
   * \brief Move in/out from the focal point, ie. adjust #distance_ by amount
   * @param amount The distance to move.  Positive amount moves towards the focal point, negative moves away
   */
  void zoom( float amount );
  void yaw( float angle );
  void pitch( float angle );

  virtual void handleMouseEvent(ViewportMouseEvent& evt);
  virtual void fromString(const std::string& str);
  virtual std::string toString();

  virtual void lookAt( const Ogre::Vector3& point );

  static std::string getClassNameStatic() { return "rviz::SimpleOrbitViewController"; }
  virtual std::string getClassName() { return getClassNameStatic(); }

protected:
  virtual void onActivate();
  virtual void onDeactivate();
  virtual void onUpdate(float dt, float ros_dt);
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  /**
   * \brief Calculates pitch and yaw values given a new position and the current focal point
   * @param position Position to calculate the pitch/yaw for
   */
  void calculatePitchYawFromPosition( const Ogre::Vector3& position );
  /**
   * \brief Normalizes the camera's pitch, preventing it from reaching vertical (or turning upside down)
   */
  void normalizePitch();
  /**
   * \brief Normalizes the camera's yaw in the range [0, 2*pi)
   */
  void normalizeYaw();

  void updateCamera();

  bool intersectGroundPlane( Ogre::Ray mouse_ray, Ogre::Vector3 &intersection_3d );

  Ogre::Vector3 focal_point_;         ///< The camera's focal point
  float yaw_;                         ///< The camera's yaw (rotation around the y-axis), in radians
  float pitch_;                       ///< The camera's pitch (rotation around the x-axis), in radians
  float distance_;                    ///< The camera's distance from the focal point
  ogre_tools::Axes *axes_;
};

}

#endif // RVIZ_VIEW_CONTROLLER_H
