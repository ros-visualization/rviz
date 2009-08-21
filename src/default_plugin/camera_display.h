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

#ifndef RVIZ_CAMERA_DISPLAY_H
#define RVIZ_CAMERA_DISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/forwards.h"
#include "rviz/image/ros_image_texture.h"

#include <sensor_msgs/CameraInfo.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

class wxFrame;

namespace rviz
{

class RenderPanel;

/**
 * \class CameraDisplay
 *
 */
class CameraDisplay : public Display
{
public:
  CameraDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~CameraDisplay();

  float getAlpha() { return alpha_; }
  void setAlpha( float alpha );

  const std::string& getTopic() { return topic_; }
  void setTopic(const std::string& topic);

  // Overrides from Display
  virtual void targetFrameChanged();
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

  void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void updateCamera();

  void clear();

  Ogre::SceneNode* scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::Camera* camera_;
  Ogre::MaterialPtr material_;

  float alpha_;
  std::string topic_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  tf::MessageFilter<sensor_msgs::CameraInfo> caminfo_tf_filter_;

  FloatPropertyWPtr alpha_property_;
  ROSTopicStringPropertyWPtr topic_property_;

  sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
  boost::mutex caminfo_mutex_;

  bool new_caminfo_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;
  wxFrame* frame_; // temp

  class RenderListener : public Ogre::RenderTargetListener
  {
  public:
    RenderListener(CameraDisplay* display);
    virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);
    virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt);

  private:
    CameraDisplay* display_;
  };
  RenderListener render_listener_;
};

} // namespace rviz

 #endif
