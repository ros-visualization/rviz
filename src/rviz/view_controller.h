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
#include "rviz/CameraPlacementTrajectory.h"
#include "ros/subscriber.h"

namespace YAML
{
class Node;
class Emitter;
}

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
class EnumProperty;
class ViewportMouseEvent;

class ViewController: public Property
{
Q_OBJECT
public:
  ViewController();
  virtual ~ViewController();
  void initialize( DisplayContext* context, Ogre::SceneNode* target_scene_node );

  /** @brief Do subclass-specific initialization.  Called by
   * ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set.  Default implementation does nothing. */
  virtual void onInitialize() {}

  virtual QVariant getViewData( int column, int role ) const;

  virtual Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Called by RenderPanel when this view controller is about to be used.
   *
   * There is no deactivate() because ViewControllers leaving
   * "current" are destroyed.  Put any cleanup in the destructor. */
  void activate( const std::string& reference_frame );

  /** @brief called by activate().
   *
   * Override to implement view-specific activation.  This base
   * implementation does nothing. */
  virtual void onActivate() {}

  void update(float dt, float ros_dt);
  void setTargetFrame(const std::string& reference_frame);

  void viewControllerMsgCallback(const rviz::CameraPlacementTrajectoryConstPtr &cptptr);
  //void viewControllerMsgCallback(const rviz::CameraPlacementTrajectory &cpt);

  virtual void updateFromStandardViewControllerMsg(const rviz::CameraPlacementTrajectory &cpt) {}

  void transformCameraPlacementToFixedFrame(rviz::CameraPlacement &cp);

  virtual void handleMouseEvent(ViewportMouseEvent& evt) {}

  virtual void lookAt( const Ogre::Vector3& point ) = 0;

  /** Reset the view controller to some sane initial state, like
   * looking at 0,0,0 of the target frame. */
  virtual void reset() = 0;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @a source_view.
   *
   * @a source_view must return a valid @c Ogre::Camera* from getCamera().
   *
   * This base class implementation does nothing. */
  virtual void mimic( ViewController* source_view ) {}

  /** @brief Called by ViewManager when this ViewController is being made current.
   * @param previous_view is the previous "current" view, and will not be NULL.
   *
   * This gives ViewController subclasses an opportunity to implement
   * a smooth transition from a previous viewpoint to the new
   * viewpoint.
   *
   * This base class implementation does nothing. */
  virtual void transitionFrom( ViewController* previous_view ) {}

  /** @brief Subclasses should call this whenever a change is made which would change the results of toString(). */
  void emitConfigChanged();

  Ogre::Camera* getCamera() const { return camera_; }

  /** @brief Add an enum property to this view which lets the user
   * replace this view with one of a different type. */
  void addTypeSelector( const QStringList& class_ids );

  /** @brief Return the class identifier which was used to create this
   * instance.  This version just returns whatever was set with
   * setClassId(). */
  virtual QString getClassId() const { return class_id_; }

  /** @brief Set the class identifier used to create this instance.
   * Typically this will be set by the factory object which created it. */
  virtual void setClassId( const QString& class_id ) { class_id_ = class_id; }

  virtual void load( const YAML::Node& yaml_node );
  virtual void save( YAML::Emitter& emitter );

  bool isActive() const { return is_active_; }

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
  bool is_active_;

  ros::Subscriber pose_subscriber_;

private:
  EnumProperty* type_property_;
  QString class_id_;
};

} // end namespace rviz

#endif // RVIZ_VIEW_CONTROLLER_H
