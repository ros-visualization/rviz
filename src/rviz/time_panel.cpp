
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

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QHBoxLayout>
#include <QButtonGroup>
#include <QCheckBox>
#include <QSlider>
#include <QComboBox>

#include "visualization_manager.h"
#include "frame_manager.h"

#include "time_panel.h"

namespace rviz
{

TimePanel::TimePanel( QWidget* parent )
  : Panel( parent )
{
  ros_time_label_ = makeTimeLabel();

  QPushButton* pause_button = new QPushButton( "Pause" );
  pause_button->setToolTip("Freeze ROS time.");
  pause_button->setCheckable(true);

  QCheckBox* sync_checkbox = new QCheckBox( "Sync" );
  sync_checkbox->setToolTip("Sync ROS time to the given time signal.");

  // choose time sync signal
  sync_selector_ = new QComboBox(this);

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget( new QLabel( "Time Mode:" ));
  layout->addWidget( pause_button );
  layout->addWidget( sync_checkbox );
  layout->addWidget( sync_selector_ );
  layout->addSpacing(20);
  layout->addWidget( new QLabel( "ROS Time:" ));
  layout->addWidget( ros_time_label_ );
  layout->addStretch();
  layout->setContentsMargins( 11, 5, 11, 5 );
  setLayout( layout );

  connect( pause_button, SIGNAL( toggled( bool )), this, SLOT( pauseToggled( bool ) ));
  connect( sync_checkbox, SIGNAL( toggled( bool )), this, SLOT( syncToggled( bool ) ));
}

QLineEdit* TimePanel::makeTimeLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly( true );
  return label;
}

void TimePanel::onInitialize()
{
  connect( vis_manager_, SIGNAL( timeChanged() ), this, SLOT( update() ));
}

void TimePanel::fillTimeLabel( QLineEdit* label, double time )
{
  label->setText( QString::number( time, 'f', 2 ));
}

void TimePanel::update()
{
  fillTimeLabel( ros_time_label_, vis_manager_->getROSTime() );
}

void TimePanel::pauseToggled( bool checked )
{
  vis_manager_->overrideROSTime( checked, ros::Time::now() );
}

void TimePanel::asyncToggled( bool checked )
{
}

void TimePanel::syncToggled( bool checked )
{

}

} // namespace rviz

