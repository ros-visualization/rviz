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

#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "rviz/display.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

namespace Ogre
{
class SceneNode;
class ManualObject;
class Rectangle2D;
class Camera;
}

class QWidget;

namespace rviz
{

class EditableEnumProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class PanelDockWidget;
class RenderPanel;
class RosTopicProperty;

/**
 * \class CameraDisplay
 *
 */
class CameraDisplay: public Display, public Ogre::RenderTargetListener
{
Q_OBJECT
public:
  CameraDisplay();
  virtual ~CameraDisplay();

  virtual void onInitialize();

  // Overrides from Display
  virtual void fixedFrameChanged();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  // Overrides from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

  static const QString BACKGROUND;
  static const QString OVERLAY;
  static const QString BOTH;

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void forceRender();
  void updateAlpha();
  void updateQueueSize();
  void updateTopic();
  void updateTransport();
  void fillTransportOptionList( QStringList* options_out );

private:
  void subscribe();
  void unsubscribe();

  void caminfoCallback( const sensor_msgs::CameraInfo::ConstPtr& msg );

  void updateCamera();

  void clear();
  void updateStatus();

  Ogre::SceneNode* bg_scene_node_;
  Ogre::SceneNode* fg_scene_node_;

  Ogre::Rectangle2D* bg_screen_rect_;
  Ogre::MaterialPtr bg_material_;

  Ogre::Rectangle2D* fg_screen_rect_;
  Ogre::MaterialPtr fg_material_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;
  tf::MessageFilter<sensor_msgs::CameraInfo>* caminfo_tf_filter_;

  FloatProperty* alpha_property_;
  RosTopicProperty* topic_property_;
  EditableEnumProperty* transport_property_;
  EnumProperty* image_position_property_;
  FloatProperty* zoom_property_;
  IntProperty* queue_size_property_;

  sensor_msgs::CameraInfo::ConstPtr current_caminfo_;
  boost::mutex caminfo_mutex_;

  bool new_caminfo_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  bool force_render_;

  PanelDockWidget* panel_container_;
};

} // namespace rviz

 #endif
