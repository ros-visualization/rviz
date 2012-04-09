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

#ifndef RVIZ_FIXED_ORIENTATION_ORTHO_VIEW_CONTROLLER_H
#define RVIZ_FIXED_ORIENTATION_ORTHO_VIEW_CONTROLLER_H

#include "rviz/view_controller.h"

#include <OGRE/OgreQuaternion.h>

namespace rviz
{
class Shape;
class SceneNode;
}

namespace rviz
{

class FixedOrientationOrthoViewController : public ViewController
{
public:
  FixedOrientationOrthoViewController(VisualizationManager* manager, const std::string& name, Ogre::SceneNode* target_scene_node);
  virtual ~FixedOrientationOrthoViewController();

  virtual void handleMouseEvent(ViewportMouseEvent& evt);
  virtual void fromString(const std::string& str);
  virtual std::string toString();

  virtual void lookAt( const Ogre::Vector3& point_rel_world );

  static std::string getClassNameStatic() { return "rviz::FixedOrientationOrthoViewController"; }
  virtual std::string getClassName() { return getClassNameStatic(); }

  virtual void reset();

protected:
  virtual void onActivate();
  virtual void onDeactivate();
  virtual void onUpdate(float dt, float ros_dt);
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation);

  /** Set the camera orientation based on angle_. */
  void orientCamera();

  void setPosition( const Ogre::Vector3& pos_rel_target );
  void move( float x, float y );
  void updateCamera();

  float scale_;
  float angle_;
  bool dragging_;
};

}

#endif // RVIZ_VIEW_CONTROLLER_H
