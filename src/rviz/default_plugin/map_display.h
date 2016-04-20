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

#ifndef RVIZ_MAP_DISPLAY_H
#define RVIZ_MAP_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>
#endif

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include "rviz/display.h"

namespace Ogre
{
class ManualObject;
}

namespace rviz
{

class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;

/**
 * \class MapDisplay
 * \brief Displays a map along the XY plane.
 */
class MapDisplay: public Display
{
Q_OBJECT
public:
  MapDisplay();
  virtual ~MapDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();

  float getResolution() { return resolution_; }
  int getWidth() { return width_; }
  int getHeight() { return height_; }

  virtual void setTopic( const QString &topic, const QString &datatype );

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updatePalette();
  /** @brief Show current_map_ in the scene. */
  void showMap();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();
  virtual void update( float wall_dt, float ros_dt );

  /** @brief Copy msg into current_map_ and call showMap(). */ 
  void incomingMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  /** @brief Copy update's data into current_map_ and call showMap(). */ 
  void incomingUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& update);

  void clear();

  void transformMap();

  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  std::string frame_;
  nav_msgs::OccupancyGrid current_map_;

  ros::Subscriber map_sub_;
  ros::Subscriber update_sub_;

  RosTopicProperty* topic_property_;
  FloatProperty* resolution_property_;
  IntProperty* width_property_;
  IntProperty* height_property_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  FloatProperty* alpha_property_;
  Property* draw_under_property_;
  EnumProperty* color_scheme_property_;

  BoolProperty* unreliable_property_;
};

} // namespace rviz

 #endif
