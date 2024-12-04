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

// Apparently OSX #defines 'check' to be an empty string somewhere.
// That was fun to figure out.
#ifdef check
#undef check
#endif

#include <QTimer>

#include <ros/ros.h>

#include <rviz/noetic_eol_dialog.h>

namespace rviz
{
NoeticEOLDialog::NoeticEOLDialog(QWidget* parent) : QMessageBox(parent)
{
  setIcon(QMessageBox::Critical);
  std::stringstream ss;
  ss << "ROS 1 goes end-of-life 2025-05-31\n\n";
  ss << "Users are encouraged to migrate to ROS 2 as soon as possible!\n\n";
  ss << "For more information see: \n";
  ss << "<some URL TBD>\n\n";
  ss << "To disable this dialog set the DISABLE_ROS1_EOL_WARNINGS environment variable.\n";
  setText(QString::fromStdString(ss.str()));
  setWindowTitle("ROS 1 End-of-Life is May 31st, 2025");
  setStandardButtons(QMessageBox::Ok);
}
} // end namespace rviz
