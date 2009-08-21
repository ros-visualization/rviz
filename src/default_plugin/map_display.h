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

#include <nav_msgs/GetMap.h>

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
  MapDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~MapDisplay();

  void setService( const std::string& service );
  const std::string& getService() { return service_; }

  float getResolution() { return resolution_; }
  float getWidth() { return width_; }
  float getHeight() { return height_; }
  Ogre::Vector3 getPosition() { return position_; }
  Ogre::Quaternion getOrientation() { return orientation_; }

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  float getMapRequestTime() { return map_request_time_; }
  void setMapRequestTime(float time);

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  void incomingMetaData(const nav_msgs::MapMetaData::ConstPtr& msg);

  void clear();
  void load();
  void transformMap();

  void requestThreadFunc();

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string service_;
  float resolution_;
  float width_;
  float height_;
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  float alpha_;

  float map_request_time_;
  float map_request_timer_;

  ros::Subscriber metadata_sub_;
  ros::Time last_loaded_map_time_;

  boost::thread request_thread_;
  nav_msgs::GetMap map_srv_;
  bool new_map_;
  boost::mutex map_mutex_;
  bool reload_;

  StringPropertyWPtr service_property_;
  FloatPropertyWPtr resolution_property_;
  FloatPropertyWPtr width_property_;
  FloatPropertyWPtr height_property_;
  Vector3PropertyWPtr position_property_;
  QuaternionPropertyWPtr orientation_property_;
  FloatPropertyWPtr alpha_property_;
  FloatPropertyWPtr map_request_time_property_;
};

} // namespace rviz

 #endif
