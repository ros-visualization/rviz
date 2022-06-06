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

#include "rviz/ogre_helpers/render_system.h"
#include "rviz/ogre_helpers/render_widget.h"

#include <QApplication>
#include <QTimer>
#include <QVBoxLayout>
#include <QPushButton>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreRenderWindow.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

using namespace rviz;

int main(int argc, char** argv)
{
  // initialize the entire render system
  RenderSystem* render_system = RenderSystem::get();

  QApplication app(argc, argv);


  // make the render window
  RenderWidget* window = new RenderWidget(render_system);
  window->setWindowTitle("I hope this is not all black.");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(window);
  QPushButton* hide_button = new QPushButton("hide");
  layout->addWidget(hide_button);
  QPushButton* show_button = new QPushButton("show");
  layout->addWidget(show_button);

  QWidget container;
  container.setLayout(layout);
  container.resize(900, 600);
  container.show();

  // Make a scene and show it in the window.
  Ogre::SceneManager* scene_manager = render_system->root()->createSceneManager(Ogre::ST_GENERIC);

  Ogre::Entity* thing = scene_manager->createEntity("thing", "rviz_cone.mesh");
  Ogre::SceneNode* node = scene_manager->getRootSceneNode()->createChildSceneNode();
  node->attachObject(thing);

  scene_manager->setAmbientLight(Ogre::ColourValue(.5, .5, .5));
  Ogre::Light* light = scene_manager->createLight("light");
  light->setPosition(20, 80, 50);

  Ogre::Camera* camera = scene_manager->createCamera("SampleCam");
  camera->setPosition(Ogre::Vector3(0, 0, 10));
  camera->lookAt(Ogre::Vector3(0, 0, -300));
  camera->setNearClipDistance(5);

  Ogre::Viewport* viewport = window->getRenderWindow()->addViewport(camera);
  viewport->setBackgroundColour(Ogre::ColourValue(0, 0, 1.0));

  camera->setAspectRatio(Ogre::Real(viewport->getActualWidth()) /
                         Ogre::Real(viewport->getActualHeight()));

  // redraw every 33ms.
  QTimer timer;
  QObject::connect(&timer, SIGNAL(timeout()), window, SLOT(update()));
  timer.start(33);


  RenderWidget window2(render_system);
  window2.resize(400, 400);
  window2.setWindowTitle("I hope this is also not all black.");
  window2.show();

  hide_button->connect(hide_button, SIGNAL(clicked()), &window2, SLOT(hide()));
  show_button->connect(show_button, SIGNAL(clicked()), &window2, SLOT(show()));

  Ogre::Camera* camera2 = scene_manager->createCamera("SampleCam2");
  camera2->setPosition(Ogre::Vector3(0, 10, 0));
  camera2->setFixedYawAxis(false);
  camera2->lookAt(Ogre::Vector3(0, 0, 0));
  camera2->setNearClipDistance(5);

  Ogre::Viewport* viewport2 = window2.getRenderWindow()->addViewport(camera2);
  viewport2->setBackgroundColour(Ogre::ColourValue(0, 1.0, 0));

  camera2->setAspectRatio(Ogre::Real(viewport2->getActualWidth()) /
                          Ogre::Real(viewport2->getActualHeight()));

  // redraw every 33ms.
  QTimer timer2;
  QObject::connect(&timer2, SIGNAL(timeout()), &window2, SLOT(update()));
  timer2.start(33);

  // main loop
  return app.exec();
}
