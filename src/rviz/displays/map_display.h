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

#ifndef OGRE_VISUALIZER_MAP_DISPLAY_H
#define OGRE_VISUALIZER_MAP_DISPLAY_H

#include "display.h"

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>

#include <robot_msgs/MapMetaData.h>
#include <ros/time.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace rviz
{

class StringProperty;
class FloatProperty;

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

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  float getMapRequestTime() { return map_request_time_; }
  void setMapRequestTime(float time);

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update( float dt );
  virtual void reset();

  static const char* getTypeStatic() { return "Map"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  void incomingMetaData();

  void clear();
  void load();
  void transformMap();

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string service_;
  float resolution_;
  float width_;
  float height_;

  float load_timer_;

  float alpha_;

  float map_request_time_;
  float map_request_timer_;

  bool new_metadata_;
  robot_msgs::MapMetaData metadata_message_;
  ros::Time last_loaded_map_time_;

  StringProperty* service_property_;
  FloatProperty* resolution_property_;
  FloatProperty* width_property_;
  FloatProperty* height_property_;
  FloatProperty* alpha_property_;
  FloatProperty* map_request_time_property_;
};

} // namespace rviz

 #endif
