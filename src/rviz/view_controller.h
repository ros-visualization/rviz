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

#ifndef RVIZ_VIEW_CONTROLLER_H
#define RVIZ_VIEW_CONTROLLER_H

#include <string>

namespace Ogre
{
class Camera;
class SceneNode;
class Vector3;
class Quaternion;
}

namespace rviz
{

class VisualizationManager;
class ViewportMouseEvent;

class ViewController
{
public:
  ViewController(VisualizationManager* manager, const std::string& name);
  virtual ~ViewController();

  void activate(Ogre::Camera* camera, const std::string& reference_frame);
  void deactivate();
  void update(float dt, float ros_dt);
  void setReferenceFrame(const std::string& reference_frame);
  const std::string& getName() { return name_; }

  virtual void handleMouseEvent(ViewportMouseEvent& evt) {}
  virtual void fromString(const std::string& str) = 0;
  virtual std::string toString() = 0;
  virtual void lookAt( const Ogre::Vector3& point ) = 0;
  virtual std::string getClassName() = 0;

protected:
  virtual void onActivate() = 0;
  virtual void onDeactivate() = 0;
  virtual void onReferenceFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation) = 0;
  virtual void onUpdate(float dt, float ros_dt) {}

  void updateReferenceNode();

  VisualizationManager* manager_;
  Ogre::Camera* camera_;
  std::string reference_frame_;
  Ogre::SceneNode* reference_node_;

  std::string name_;
};

}

#endif // RVIZ_VIEW_CONTROLLER_H
