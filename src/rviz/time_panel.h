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

#ifndef RVIZ_TIME_PANEL_H
#define RVIZ_TIME_PANEL_H

#include <QWidget>

class QLineEdit;

namespace rviz
{

class VisualizationManager;

/**
 * \class TimePanel
 *
 */
class TimePanel: public QWidget
{
Q_OBJECT
public:
  TimePanel( QWidget* parent = 0 );

  void initialize( VisualizationManager* manager );

  VisualizationManager* getManager() { return manager_; }

protected Q_SLOTS:
  /** Reset elapsed timers to 0. */
  void reset();

  /** Read time values from VisualizationManager and update displays. */
  void update();

protected:
  /** Create, configure, and return a single label for showing a time value. */
  QLineEdit* makeTimeLabel();

  /** Fill a single time label with the given time value (in seconds). */
  void fillTimeLabel( QLineEdit* label, double time );

  VisualizationManager* manager_;

  QLineEdit* wall_time_label_;
  QLineEdit* wall_elapsed_label_;
  QLineEdit* ros_time_label_;
  QLineEdit* ros_elapsed_label_;
};

} // namespace rviz

#endif


