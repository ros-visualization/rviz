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

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>

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

namespace rviz
{

class EditableEnumProperty;
class IntProperty;
class PanelDockWidget;
class RenderPanel;
class RosTopicProperty;

/**
 * \class ImageDisplay
 *
 */
class ImageDisplay: public Display
{
Q_OBJECT
public:
  ImageDisplay();
  virtual ~ImageDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  /** @brief Overridden from Property to update the view widget's title. */
  virtual void setName( const QString& name );

protected Q_SLOTS:
  void updateTopic();
  void updateQueueSize();
  void updateTransport();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void fillTransportOptionList( QStringList* options_out );

private:
  void subscribe();
  void unsubscribe();

  void clear();
  void updateStatus();

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  std::string topic_;
  std::string transport_;

  RosTopicProperty* topic_property_;
  EditableEnumProperty* transport_property_;
  IntProperty* queue_size_property_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  PanelDockWidget* panel_container_;
};

} // namespace rviz

#endif
