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

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "rviz/properties/property.h"

namespace Ogre
{
class Camera;
class SceneNode;
class Vector3;
class Quaternion;
}

namespace rviz
{
class DisplayContext;
class ViewportMouseEvent;

class ViewController: public Property
{
Q_OBJECT
public:
  ViewController(DisplayContext* context, const std::string& name, Ogre::SceneNode* target_scene_node);
  virtual ~ViewController();

  Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Called by RenderPanel when this view controller is about to be used.
   *
   * Override to implement view-specific activation.  This base
   * implementation records the given reference frame. */
  virtual void activate( const std::string& reference_frame );

  /** @brief Called by RenderPanel when this view controller is done being used.
   *
   * Override to implement view-specific deactivation.  This base
   * implementation does nothing. */
  virtual void deactivate() {}

  void update(float dt, float ros_dt);
  void setTargetFrame(const std::string& reference_frame);

  virtual void handleMouseEvent(ViewportMouseEvent& evt) {}

  /** Set internal state from a string. */
  virtual void fromString(const std::string& str) = 0;

  /** Return internal state as a string. */
  virtual std::string toString() = 0;

  virtual void lookAt( const Ogre::Vector3& point ) = 0;
  virtual std::string getClassName() = 0;

  /** Reset the view controller to some sane initial state, like
   * looking at 0,0,0 of the target frame. */
  virtual void reset() = 0;

  /** @brief Implement copy() in each subclass to return a deep copy
   * of this view controller. */
  virtual ViewController* copy() const = 0;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera().
   *
   * This base class implementation does nothing. */
  virtual void initializeFrom( ViewController* source_view ) {}

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  Ogre::Camera* getCamera() const { return camera_; }

Q_SIGNALS:
  void configChanged();

protected:
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation) = 0;
  virtual void onUpdate(float dt, float ros_dt) {}

  void updateTargetSceneNode();

  DisplayContext* context_;
  Ogre::Camera* camera_;
  std::string reference_frame_;
  Ogre::SceneNode* target_scene_node_;

  Ogre::Quaternion global_orientation_;

  std::string name_;

  Ogre::Quaternion reference_orientation_;
  Ogre::Vector3 reference_position_;
};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
