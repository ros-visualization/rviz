/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OGRE_VISUALIZER_TF_DISPLAY_H
#define OGRE_VISUALIZER_TF_DISPLAY_H

#include "display.h"

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <map>

namespace ogre_tools
{
class Arrow;
class Axes;
class MovableText;
}

namespace Ogre
{
class SceneNode;
}

namespace ogre_vis
{

class BoolProperty;
class Vector3Property;
class QuaternionProperty;
class FloatProperty;
class CategoryProperty;
class StringProperty;

struct FrameInfo
{
  FrameInfo();

  const Ogre::Vector3& getPositionInRobotSpace() { return robot_space_position_; }
  const Ogre::Quaternion& getOrientationInRobotSpace() { return robot_space_orientation_; }
  const std::string& getParent() { return parent_; }

  std::string name_;
  std::string parent_;
  ogre_tools::Axes* axes_;
  ogre_tools::Arrow* parent_arrow_;
  ogre_tools::MovableText* name_text_;
  Ogre::SceneNode* name_node_;

  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  float distance_to_parent_;
  Ogre::Quaternion arrow_orientation_;

  Ogre::Vector3 robot_space_position_;
  Ogre::Quaternion robot_space_orientation_;

  CategoryProperty* category_;
  Vector3Property* position_property_;
  QuaternionProperty* orientation_property_;
  StringProperty* parent_property_;

  CategoryProperty* tree_property_;
};

/**
 * \class TFDisplay
 * \brief Displays a visual representation of the TF hierarchy
 */
class TFDisplay : public Display
{
public:
  TFDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~TFDisplay();

  bool getShowNames() { return show_names_; }
  void setShowNames( bool show );

  bool getShowAxes() { return show_axes_; }
  void setShowAxes( bool show );

  bool getShowArrows() { return show_arrows_; }
  void setShowArrows( bool show );

  float getUpdateRate() { return update_rate_; }
  void setUpdateRate( float rate );

  // Overrides from Display
  virtual void update( float dt );
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged() {}
  virtual void createProperties();
  virtual bool isObjectPickable( const Ogre::MovableObject* object ) const { return true; }
  virtual void reset();

  static const char* getTypeStatic() { return "TF"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();

protected:
  void updateFrames();
  FrameInfo* createFrame(const std::string& frame);
  void updateFrame(FrameInfo* frame);
  void deleteFrame(FrameInfo* frame);

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

  BoolProperty* show_names_property_;
  BoolProperty* show_arrows_property_;
  BoolProperty* show_axes_property_;
  FloatProperty* update_rate_property_;

  CategoryProperty* frames_category_;
  CategoryProperty* tree_category_;
};

} // namespace ogre_vis

 #endif

