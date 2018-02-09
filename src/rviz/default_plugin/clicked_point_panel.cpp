/*
 * Copyright (c) 2018, Synapticon GmbH.
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

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>

#include "clicked_point_panel.h"

namespace rviz
{
ClickedPointPanel::ClickedPointPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* x_layout = new QHBoxLayout;
  x_layout->addWidget(new QLabel("X:"));
  x_line_edit_ = new QLineEdit;
  x_layout->addWidget(x_line_edit_);

  QHBoxLayout* y_layout = new QHBoxLayout;
  y_layout->addWidget(new QLabel("Y:"));
  y_line_edit_ = new QLineEdit;
  y_layout->addWidget(y_line_edit_);

  QHBoxLayout* z_layout = new QHBoxLayout;
  z_layout->addWidget(new QLabel("Z:"));
  z_line_edit_ = new QLineEdit;
  z_layout->addWidget(z_line_edit_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(x_layout);
  layout->addLayout(y_layout);
  layout->addLayout(z_layout);
  setLayout(layout);
  clicked_point_subscriber_ = nh_.subscribe("/clicked_point", 10, &ClickedPointPanel::clickedPointCallback, this);
}

void ClickedPointPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void ClickedPointPanel::clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  x_line_edit_->setText(QString::number(msg->point.x));
  y_line_edit_->setText(QString::number(msg->point.y));
  z_line_edit_->setText(QString::number(msg->point.y));
}

void ClickedPointPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::ClickedPointPanel, rviz::Panel)
