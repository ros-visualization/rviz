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

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlEngine>
#include <QQmlContext>
#include <QString>


#include <ros/ros.h>
#include <ros/package.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/displays_panel.h"
#include "rviz/quick_visualization_frame.h"

#include "simplegrid.h"
#include "displayconfig.h"

using namespace rviz;

int main(int argc, char **argv)
{
  QApplication app( argc, argv );
  ros::init( argc, argv, "quick_render_panel_test" );

  rviz::QuickVisualizationFrame::registerTypes();
  qmlRegisterType<SimpleGrid>("MyModule", 1, 0, "SimpleGrid");
  qmlRegisterType<DisplayConfig>("MyModule", 1, 0, "DisplayConfig");

  QQmlApplicationEngine engine;
  engine.rootContext()->setContextProperty("rvizPath", QString::fromStdString(ros::package::getPath("rviz")));
  engine.load(QUrl("qrc:/qml/quick_render_panel_test.qml"));

  return app.exec();
}
