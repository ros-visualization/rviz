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

#include "display.h"
#include "properties/forwards.h"
#include "image/ros_image_texture.h"

#include <image_msgs/CamInfo.h>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

namespace tf
{
template<class Message> class MessageNotifier;
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
  virtual void update( float dt );
  virtual void reset();

  static const char* getTypeStatic() { return "Camera"; }
  virtual const char* getType() const { return getTypeStatic(); }
  static const char* getDescription();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  typedef boost::shared_ptr<image_msgs::CamInfo const> CamInfoConstPtr;
  void caminfoCallback(const CamInfoConstPtr& msg);

  void updateCamera();

  void clear();

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* screen_rect_;
  Ogre::Camera* camera_;
  Ogre::MaterialPtr material_;

  float alpha_;
  std::string topic_;

  tf::MessageNotifier<image_msgs::CamInfo>* caminfo_notifier_;

  FloatPropertyWPtr alpha_property_;
  ROSTopicStringPropertyWPtr topic_property_;

  CamInfoConstPtr current_caminfo_;
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
