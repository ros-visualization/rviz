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

#include "test/render_points_test.h"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QApplication>
#include <QVBoxLayout>

#include <OgreCamera.h>
#include <OgreSceneNode.h>

using namespace rviz;

MyFrame::MyFrame(QWidget* parent)
  : QWidget(parent)
  , left_mouse_down_(false)
  , middle_mouse_down_(false)
  , right_mouse_down_(false)
  , mouse_x_(0)
  , mouse_y_(0)
{
  render_system_ = RenderSystem::get();
  root_ = render_system_->root();

  try
  {
    scene_manager_ = root_->createSceneManager(Ogre::ST_GENERIC, "TestSceneManager");

    render_panel_ = new QtOgreRenderWindow();
    render_panel_->resize(this->size());
    render_panel_->setAutoRender(false);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(render_panel_);
    setLayout(layout);

    camera_ = new OrbitCamera(scene_manager_);
    camera_->setPosition(0, 0, 15);
    camera_->getOgreCamera()->setNearClipDistance(0.1);

    render_panel_->setCamera(camera_->getOgreCamera());
    render_panel_->setBackgroundColor(Ogre::ColourValue(0.8, 0.8, 1));

    Ogre::Light* directional_light = scene_manager_->createLight("MainDirectional");
    directional_light->setType(Ogre::Light::LT_DIRECTIONAL);
    directional_light->setDirection(Ogre::Vector3(0, -1, 1));
    directional_light->setDiffuseColour(Ogre::ColourValue(1.0f, 1.0f, 1.0f));

#if 0
    Grid* grid = new Grid( scene_manager_, NULL, Grid::Lines, 10, 1.0f, 0.02, Ogre::ColourValue(1.0f, 1.0f, 1.0f, 0.5f));
    grid->setHeight(4);


    BillboardLine* line = new BillboardLine( scene_manager_, NULL );
    line->setMaxPointsPerLine(105);
    for ( int i = -50; i < 50; ++i )
      {
        line->addPoint( Ogre::Vector3( i*2, 0.0f, -1.0f ) );
      }

    for ( int i = 0; i < 5; ++i )
      {
        line->addPoint( Ogre::Vector3( 4.0f, 0.0f, i ) );
      }

    line->setLineWidth( 0.05 );
    line->setColor( 0.0f, 1.0f, 0.0f, 0.5f );

    Shape* sphere = new Shape(Shape::Sphere, scene_manager_);
    sphere->setPosition(Ogre::Vector3(0.0f, 0.0f, 2.0f));
    sphere->setColor(0.0f, 1.0f, 2.0f, 1.0f);
    Shape* cube = new Shape(Shape::Cube, scene_manager_);
    cube->setPosition(Ogre::Vector3(0.0f, 1.0f, 2.0f));
    cube->setColor(1.0f, 0.0f, 0.0f, 1.0f);
    Shape* cylinder = new Shape(Shape::Cylinder, scene_manager_);
    cylinder->setPosition(Ogre::Vector3(0.0f, 2.0f, 2.0f));
    cylinder->setColor(1.0f, 1.0f, 0.0f, 1.0f);
    Shape* cone = new Shape(Shape::Cone, scene_manager_);
    cone->setPosition(Ogre::Vector3(0.0f, 3.0f, 2.0f));
    cone->setColor(0.0f, 0.0f, 1.0f, 1.0f);

    Axes* axes = new Axes( scene_manager_ );
    //axes->setScale( Ogre::Vector3( 2.0f, 2.0f, 2.0f ) );

    /*Cone* cone = new Cone( scene_manager_, NULL );
      cone->setScale( Ogre::Vector3( 0.3f, 2.0f, 0.3f ) );*/

    Arrow* arrow = new Arrow( scene_manager_ );
    arrow->setHeadColor( 1.0f, 0.0f, 0.0f );
    arrow->setShaftColor( 0.0f, 0.0f, 1.0f );
    arrow->setOrientation( Ogre::Quaternion::IDENTITY );
    //arrow->setOrientation( Ogre::Quaternion( Ogre::Degree( 45 ), Ogre::Vector3::UNIT_X ) );
    //arrow->setScale( Ogre::Vector3( 1.0f, 1.0f, 3.0f ) );
#endif

#if 01
    Ogre::SceneNode* scene_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    PointCloud* pointCloud = new PointCloud();
    pointCloud->setDimensions(0.05f, 0.05f, 0.05f);
    // pointCloud->setColorByIndex(true);
    pointCloud->setRenderMode(PointCloud::RM_SQUARES);
    pointCloud->setCommonDirection(Ogre::Vector3(0.0, 1.0, 0.0));
    pointCloud->setCommonUpVector(Ogre::Vector3(0.0, 0.0, -1.0));
    pointCloud->setAlpha(1.0);
    std::vector<PointCloud::Point> points;
    int32_t xcount = 200;
    int32_t ycount = 100;
    int32_t zcount = 100;
    //        points.resize(xcount * ycount * zcount);
    float factor = 0.1f;
    for (int32_t x = 0; x < xcount; ++x)
    {
      for (int32_t y = 0; y < ycount; ++y)
      {
        for (int32_t z = 0; z < zcount; ++z)
        {
          //    int32_t index = (ycount*zcount*x) + zcount*y + z;
          PointCloud::Point point; // = points[index];
          point.position.x = x * factor;
          point.position.y = y * factor;
          point.position.z = z * factor;

          point.setColor(x * 0.1, y * 0.1, z * 0.1);
          points.push_back(point);
        }
      }
    }

    printf("size: %d\n", (int)points.size());
    pointCloud->addPoints(&points.front(), points.size());
    scene_node->attachObject(pointCloud);
#endif
  }
  catch (Ogre::Exception& e)
  {
    printf("Fatal error: %s\n", e.what());
    exit(1);
  }

  connect(&render_timer_, SIGNAL(timeout()), this, SLOT(doRender()));
  render_timer_.start(33);
}

MyFrame::~MyFrame()
{
}

void MyFrame::doRender()
{
  ros::WallTime start = ros::WallTime::now();
  root_->renderOneFrame();
  ros::WallTime end = ros::WallTime::now();
  printf("Render took [%f] msec\n", (end - start).toSec() * 1000.0f);
}

void MyFrame::mousePressEvent(QMouseEvent* event)
{
  left_mouse_down_ = false;
  middle_mouse_down_ = false;
  right_mouse_down_ = false;

  switch (event->button())
  {
  case Qt::LeftButton:
    left_mouse_down_ = true;
    break;
  case Qt::MidButton:
    middle_mouse_down_ = true;
    break;
  case Qt::RightButton:
    right_mouse_down_ = true;
    break;
  default:
    break;
  }
}

void MyFrame::mouseReleaseEvent(QMouseEvent* event)
{
  switch (event->button())
  {
  case Qt::LeftButton:
    left_mouse_down_ = false;
    break;
  case Qt::MidButton:
    middle_mouse_down_ = false;
    break;
  case Qt::RightButton:
    right_mouse_down_ = false;
    break;
  default:
    break;
  }
}

void MyFrame::mouseMoveEvent(QMouseEvent* event)
{
  int32_t diff_x = event->x() - mouse_x_;
  int32_t diff_y = event->y() - mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  bool cmd = event->modifiers() & Qt::ControlModifier;
  bool shift = event->modifiers() & Qt::ShiftModifier;
  bool alt = event->modifiers() & Qt::AltModifier;

  if (left_mouse_down_)
  {
    camera_->mouseLeftDrag(diff_x, diff_y, cmd, alt, shift);
  }
  else if (middle_mouse_down_)
  {
    camera_->mouseMiddleDrag(diff_x, diff_y, cmd, alt, shift);
  }
  else if (right_mouse_down_)
  {
    camera_->mouseRightDrag(diff_x, diff_y, cmd, alt, shift);
  }
}

void MyFrame::wheelEvent(QWheelEvent* event)
{
  if (event->delta() != 0)
  {
    bool cmd = event->modifiers() & Qt::ControlModifier;
    bool shift = event->modifiers() & Qt::ShiftModifier;
    bool alt = event->modifiers() & Qt::AltModifier;

    camera_->scrollWheel(event->delta(), cmd, alt, shift);
  }
}

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  MyFrame frame;
  frame.resize(800, 600);
  frame.setWindowTitle("I hope this is not all black.");
  frame.show();

  return app.exec();
}
