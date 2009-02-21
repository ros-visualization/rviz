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

#ifndef OGRE_VISUALIZER_ROBOT_MODEL_DISPLAY_H
#define OGRE_VISUALIZER_ROBOT_MODEL_DISPLAY_H

#include "display.h"

#include <robot_msgs/MechanismState.h>

#include <OGRE/OgreVector3.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace ogre_tools
{
class Axes;
}

namespace rviz
{

class BoolProperty;
class FloatProperty;
class StringProperty;
class ROSTopicStringProperty;

class Robot;

/**
 * \class RobotModelDisplay
 * \brief Uses a robot xml description to display the pieces of a robot at the transforms broadcast by rosTF
 */
class RobotModelDisplay : public Display
{
public:
  RobotModelDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~RobotModelDisplay();

  /**
   * \brief Set the robot description parameter
   * @param description_param The ROS parameter name which contains the robot xml description
   */
  void setRobotDescription( const std::string& description_param );

  virtual void update( float dt );

  /**
   * \brief Set whether the visual mesh representation should be displayed
   * @param visible
   */
  void setVisualVisible( bool visible );

  /**
   * \brief Set whether the collision representation should be displayed
   * @param visible
   */
  void setCollisionVisible( bool visible );

  /**
   * \brief Set the rate at which we request new transforms from libTF
   * @param rate The rate, in seconds
   */
  void setUpdateRate( float rate );

  const std::string& getRobotDescription() { return description_param_; }
  float getUpdateRate() { return update_rate_; }
  bool isVisualVisible();
  bool isCollisionVisible();

  void setMechanismTopic( const std::string& topic );
  const std::string& getMechanismTopic() { return mechanism_topic_; }

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged() {}
  virtual void createProperties();
  virtual bool isObjectPickable( const Ogre::MovableObject* object ) const { return true; }
  virtual void reset();

  static const char* getTypeStatic() { return "Robot Model"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();

protected:

  /**
   * \brief Loads a URDF from our #description_param_, iterates through the links and loads any necessary models
   */
  void load();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void unsubscribe();
  void subscribe();

  void incomingMechanismState();

  std::string description_param_;             ///< ROS parameter that contains the robot xml description
  std::string mechanism_topic_;               ///< ROS topic we're listening on for mechanism state messages

  Robot* robot_;                              ///< Handles actually drawing the robot

  bool has_new_transforms_;                   ///< Callback sets this to tell our update function it needs to update the transforms
  bool has_new_mechanism_state_;

  float time_since_last_transform_;
  float update_rate_;
  float alpha_;

  BoolProperty* visual_enabled_property_;
  BoolProperty* collision_enabled_property_;
  FloatProperty* update_rate_property_;
  StringProperty* robot_description_property_;
  ROSTopicStringProperty* mechanism_topic_property_;
  FloatProperty* alpha_property_;

  std::string robot_description_;

  robot_msgs::MechanismState mechanism_message_;
};

} // namespace rviz

 #endif

