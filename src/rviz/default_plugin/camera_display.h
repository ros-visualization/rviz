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

#include <memory>

#include <QObject>

#ifndef Q_MOC_RUN
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"
#endif

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
} // namespace Ogre

namespace rviz
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class RenderPanel;
class RosTopicProperty;
class DisplayGroupVisibilityProperty;

/**
 * \class CameraDisplay
 *
 */
class CameraDisplay : public ImageDisplayBase, public Ogre::RenderTargetListener
{
  Q_OBJECT
public:
  CameraDisplay();
  ~CameraDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void fixedFrameChanged() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  // Overrides from Ogre::RenderTargetListener
  void preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;
  void postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt) override;

  static const QString BACKGROUND;
  static const QString OVERLAY;
  static const QString BOTH;

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  ROSImageTexture texture_;
  RenderPanel* render_panel_;

private Q_SLOTS:
  void forceRender();
  void updateAlpha();

  void updateQueueSize() override;

private:
  void subscribe() override;
  void unsubscribe() override;

  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;
  void caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  bool updateCamera();

  void clear();
  void updateStatus();

  Ogre::SceneNode* bg_scene_node_;
  Ogre::SceneNode* fg_scene_node_;

  Ogre::Rectangle2D* bg_screen_rect_;
  Ogre::MaterialPtr bg_material_;

  Ogre::Rectangle2D* fg_screen_rect_;
  Ogre::MaterialPtr fg_material_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::CameraInfo>> caminfo_tf_filter_;

  FloatProperty* alpha_property_;
  EnumProperty* image_position_property_;
  FloatProperty* zoom_property_;
  DisplayGroupVisibilityProperty* visibility_property_;

  sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
  boost::mutex caminfo_mutex_;

  bool caminfo_tf_ok_;

  bool caminfo_ok_;

  bool force_render_;

  uint32_t vis_bit_;
};

} // namespace rviz

#endif
