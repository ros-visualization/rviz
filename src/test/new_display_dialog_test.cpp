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

#include <string>

#include <QApplication>

#include <pluginlib/class_loader.hpp>

#include <rviz/display.h>
#include <rviz/pluginlib_factory.h>
#include <rviz/new_object_dialog.h>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QString lookup_name;
  QString display_name;
  rviz::PluginlibFactory<rviz::Display>* factory =
      new rviz::PluginlibFactory<rviz::Display>("rviz", "rviz::Display");
  QStringList current_names;
  current_names << "Chub"
                << "Town";
  QStringList empty;
  rviz::NewObjectDialog* dialog = new rviz::NewObjectDialog(factory, QString("Display"), current_names,
                                                            empty, &lookup_name, &display_name, nullptr);
  if (dialog->exec() == QDialog::Accepted)
  {
    printf("lookup_name='%s', display_name='%s'\n", lookup_name.toStdString().c_str(),
           display_name.toStdString().c_str());
  }
  else
  {
    printf("cancelled\n");
  }
}
