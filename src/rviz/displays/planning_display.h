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

#ifndef OGRE_VISUALIZER_PLANNING_DISPLAY_H
#define OGRE_VISUALIZER_PLANNING_DISPLAY_H

#include "display.h"
#include <robot_msgs/DisplayKinematicPath.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace planning_models
{
class KinematicModel;
}

namespace ogre_vis
{

class BoolProperty;
class FloatProperty;
class StringProperty;
class ROSTopicStringProperty;

class Robot;

/**
 * \class PlanningDisplay
 * \brief
 */
class PlanningDisplay : public Display
{
public:
  PlanningDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~PlanningDisplay();

  /**
   * \brief Initializes the display.
   * @param description_param The ROS parameter name which contains the robot xml description
   * @param kinematic_path_topic The topic to listen on for a NamedKinematicPath
   */
  void initialize( const std::string& description_param, const std::string& kinematic_path_topic );

  /**
   * \brief Set the robot description parameter
   * @param description_param The ROS parameter name which contains the robot xml description
   */
  void setRobotDescription( const std::string& description_param );

  /**
   * \brief Set the topic to listen on for the DisplayKinematicPath message
   * @param topic The ROS topic
   */
  void setTopic( const std::string& topic );

  /**
   * \brief Set the amount of time each state should display for
   * @param time The length of time, in seconds
   */
  void setStateDisplayTime( float time );

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

  const std::string& getRobotDescription() { return description_param_; }
  const std::string& getTopic() { return kinematic_path_topic_; }
  float getStateDisplayTime() { return state_display_time_; }
  bool isVisualVisible();
  bool isCollisionVisible();

  virtual void update( float dt );

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged() {}
  virtual void createProperties();

  virtual bool isObjectPickable( const Ogre::MovableObject* object ) const { return true; }

  static const char* getTypeStatic() { return "Planning"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();


protected:
  /**
   * \brief Subscribes to any ROS topics we need to subscribe to
   */
  void subscribe();
  /**
   * \brief Unsubscribes from all ROS topics we're currently subscribed to
   */
  void unsubscribe();

  /**
   * \brief Loads a URDF from our #description_param_
   */
  void load();

  /**
   * \brief ROS callback for an incoming kinematic path message
   */
  void incomingKinematicPath();

  /**
   * \brief Uses libTF to set the robot's position, given the target frame and the planning frame
   */
  void calculateRobotPosition();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string description_param_;             ///< ROS parameter that contains the robot xml description

  Robot* robot_;                              ///< Handles actually drawing the robot

  std::string kinematic_path_topic_;
  planning_models::KinematicModel* kinematic_model_;
  robot_msgs::DisplayKinematicPath incoming_kinematic_path_message_;
  robot_msgs::DisplayKinematicPath displaying_kinematic_path_message_;
  bool new_kinematic_path_;
  bool animating_path_;
  int current_state_;
  float state_display_time_;
  float current_state_time_;

  BoolProperty* visual_enabled_property_;
  BoolProperty* collision_enabled_property_;
  FloatProperty* state_display_time_property_;
  StringProperty* robot_description_property_;
  ROSTopicStringProperty* topic_property_;
};

} // namespace ogre_vis

 #endif


