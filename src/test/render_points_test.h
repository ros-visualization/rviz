/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef RENDER_POINTS_TEST_H
#define RENDER_POINTS_TEST_H

#include <QWidget>
#include <QTimer>

#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/ogre_helpers/orbit_camera.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/ogre_helpers/render_system.h"

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#include <OgreLight.h>

#include <ros/time.h>

using namespace rviz;

class MyFrame : public QWidget
{
Q_OBJECT

public:
  MyFrame(QWidget* parent = 0);
  virtual ~MyFrame();

private Q_SLOTS:
  void doRender();

private:
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );

  Ogre::Root* root_;
  RenderSystem* render_system_;
  Ogre::SceneManager* scene_manager_;

  QtOgreRenderWindow* render_panel_;

  Grid* grid_;
  CameraBase* camera_;

  // Mouse handling
  bool left_mouse_down_;
  bool middle_mouse_down_;
  bool right_mouse_down_;
  int mouse_x_;
  int mouse_y_;

  QTimer render_timer_;
};

#endif // RENDER_POINTS_TEST_H
