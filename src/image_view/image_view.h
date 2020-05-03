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

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/ogre_helpers/initialization.h"
#include "rviz/image/ros_image_texture.h"

#include "ros/ros.h"
#include <ros/package.h>

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreRectangle2D.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreTextureUnitState.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#endif

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
#endif

using namespace rviz;

class ImageView : public QtOgreRenderWindow
{
  Q_OBJECT
public:
  ImageView(QWidget* parent = nullptr);
  ~ImageView() override;

protected:
  void showEvent(QShowEvent* event) override;

private Q_SLOTS:
  void onTimer();

private:
  void textureCallback(const sensor_msgs::Image::ConstPtr& msg);

  Ogre::SceneManager* scene_manager_;
  Ogre::Camera* camera_;
  ROSImageTexture* texture_;

  ros::NodeHandle nh_;

  image_transport::ImageTransport texture_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> texture_sub_;
};
