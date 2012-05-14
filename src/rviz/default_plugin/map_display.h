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

#ifndef RVIZ_MAP_DISPLAY_H
#define RVIZ_MAP_DISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/forwards.h"

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <nav_msgs/OccupancyGrid.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace rviz
{

/**
 * \class MapDisplay
 * \brief Displays a map along the XZ plane (XY in robot space)
 *
 */
class MapDisplay : public Display
{
public:
  MapDisplay();
  virtual ~MapDisplay();

  void onInitialize();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

  float getResolution() { return resolution_; }
  int getWidth() { return width_; }
  int getHeight() { return height_; }
  Ogre::Vector3 getPosition() { return position_; }
  Ogre::Quaternion getOrientation() { return orientation_; }

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  bool getDrawUnder() { return draw_under_; }
  void setDrawUnder(bool write);

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  /** @brief Hides all visible parts of this display, so they do not show up when the scene is rendered. */
  virtual void hideVisible();

  /** @brief Restores the display to the state it was in before hideVisible() was called. */
  virtual void restoreVisible();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  void clear();
  void load(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void transformMap();

  void requestThreadFunc();

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;
  std::string frame_;
  nav_msgs::OccupancyGrid::ConstPtr map_;

  float alpha_;
  bool draw_under_;

  ros::Subscriber map_sub_;

  ROSTopicStringPropertyWPtr topic_property_;
  FloatPropertyWPtr resolution_property_;
  IntPropertyWPtr width_property_;
  IntPropertyWPtr height_property_;
  Vector3PropertyWPtr position_property_;
  QuaternionPropertyWPtr orientation_property_;
  FloatPropertyWPtr alpha_property_;
  BoolPropertyWPtr draw_under_property_;

  bool hidden_;
};

} // namespace rviz

 #endif
