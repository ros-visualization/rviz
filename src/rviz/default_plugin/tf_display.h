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

#ifndef RVIZ_TF_DISPLAY_H
#define RVIZ_TF_DISPLAY_H

#include <map>
#include <set>

#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>

#include "rviz/display.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Arrow;
class Axes;
class BoolProperty;
class FloatProperty;
class MovableText;
class QuaternionProperty;
class StringProperty;
class VectorProperty;

struct FrameInfo;
typedef std::set<FrameInfo*> S_FrameInfo;

/** @brief Displays a visual representation of the TF hierarchy. */
class TFDisplay: public Display
{
Q_OBJECT
public:
  TFDisplay();
  virtual ~TFDisplay();

  virtual void update(float wall_dt, float ros_dt);

protected:
  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();

private Q_SLOTS:
  void updateShowAxes();
  void updateShowArrows();
  void updateShowNames();

private:
  void updateFrames();
  FrameInfo* createFrame(const std::string& frame);
  void updateFrame(FrameInfo* frame);
  void deleteFrame(FrameInfo* frame, bool delete_properties);

  FrameInfo* getFrameInfo(const std::string& frame);

  void clear();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* names_node_;
  Ogre::SceneNode* arrows_node_;
  Ogre::SceneNode* axes_node_;

  typedef std::map<std::string, FrameInfo*> M_FrameInfo;
  M_FrameInfo frames_;

  float update_timer_;
  float update_rate_;

  bool show_names_;
  bool show_arrows_;
  bool show_axes_;
  float frame_timeout_;
  bool all_enabled_;

  float scale_;

  Property* show_names_property_;
  Property* show_arrows_property_;
  Property* show_axes_property_;
  FloatProperty* update_rate_property_;
  FloatProperty* frame_timeout_property_;
  Property* all_enabled_property_;

  FloatProperty* scale_property_;

  Property* frames_category_;
  Property* tree_category_;

  friend class FrameInfo;

  class FrameInfo: public QObject
  {
    Q_OBJECT
  public:
    FrameInfo( TFDisplay* display );

    const Ogre::Vector3& getPositionInRobotSpace() { return robot_space_position_; }
    const Ogre::Quaternion& getOrientationInRobotSpace() { return robot_space_orientation_; }
    const std::string& getParent() { return parent_; }

    bool isEnabled() { return enabled_; }

  public Q_SLOTS:
    void updateVisibility();

  public:
    TFDisplay* display_;
    std::string name_;
    std::string parent_;
    Axes* axes_;
    CollObjectHandle axes_coll_;
    Arrow* parent_arrow_;
    MovableText* name_text_;
    Ogre::SceneNode* name_node_;

    Ogre::Vector3 position_;
    Ogre::Quaternion orientation_;
    float distance_to_parent_;
    Ogre::Quaternion arrow_orientation_;

    Ogre::Vector3 robot_space_position_;
    Ogre::Quaternion robot_space_orientation_;

    bool enabled_;

    ros::Time last_update_;
    ros::Time last_time_to_fixed_;

    Property* category_;
    VectorProperty* position_property_;
    QuaternionProperty* orientation_property_;
    StringProperty* parent_property_;
    BoolProperty* enabled_property_;

    Property* tree_property_;
  };
};

} // namespace rviz

 #endif

