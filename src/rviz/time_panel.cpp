
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

#include "display_group.h"

#include "time_panel.h"

namespace rviz
{

TimePanel::TimePanel( QWidget* parent )
  : Panel( parent )
{
  ros_time_label_ = makeTimeLabel();

  pause_button_ = new QPushButton( "Pause" );
  pause_button_->setToolTip("Freeze ROS time.");
  pause_button_->setCheckable(true);

  sync_mode_selector_ = new QComboBox(this);
  sync_mode_selector_->addItem( "Off" );
  sync_mode_selector_->addItem( "Exact" );
  sync_mode_selector_->addItem( "Approximate" );
  sync_mode_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_mode_selector_->setToolTip("Allows you to synchronize the ROS time and Tf transforms to a given source.");

  // choose time sync signal
  sync_source_selector_ = new QComboBox(this);
  sync_source_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_source_selector_->setToolTip("Time source to use for synchronization.");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget( pause_button_ );
  layout->addWidget( new QLabel( "Synchronization:" ));
  layout->addWidget( sync_mode_selector_ );
  layout->addWidget( new QLabel( "Source:" ));
  layout->addWidget( sync_source_selector_ );
  layout->addSpacing(20);
  layout->addWidget( new QLabel( "ROS Time:" ));
  layout->addWidget( ros_time_label_ );
  layout->addStretch();
  layout->setContentsMargins( 11, 5, 11, 5 );
  setLayout( layout );

  connect( pause_button_, SIGNAL( toggled( bool )), this, SLOT( pauseToggled( bool ) ));
  connect( sync_mode_selector_, SIGNAL( activated( int )), this, SLOT( syncModeSelected( int ) ));
}

void TimePanel::onInitialize()
{
  connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));

  DisplayGroup *display_group = vis_manager_->getRootDisplayGroup();
  onDisplayAdded(display_group);
}

void TimePanel::onDisplayAdded( Display* display )
{
  DisplayGroup* display_group = qobject_cast<DisplayGroup*>( display );
  if( display_group )
  {
    connect( display_group, SIGNAL( displayAdded( rviz::Display* ) ), this, SLOT( onDisplayAdded( rviz::Display* ) ));
    connect( display_group, SIGNAL( displayRemoved( rviz::Display* ) ), this, SLOT( onDisplayRemoved( rviz::Display* ) ));

    for( int i = 0; i < display_group->numDisplays(); i++ )
    {
      rviz::Display* display = display_group->getDisplayAt( i );
      onDisplayAdded( display );
    }
  }
  else
  {
    connect( display, SIGNAL( timeSignal( rviz::Display*, ros::Time ) ), this, SLOT( onTimeSignal( rviz::Display*, ros::Time ) ));
  }
}

void TimePanel::onDisplayRemoved( Display* display )
{
  QString name = display->getName();
  int index = sync_source_selector_->findData( QVariant( (qulonglong)display ) );
  if ( index >= 0 )
  {
    sync_source_selector_->removeItem( index );
  }
}

void TimePanel::onTimeSignal( Display* display, ros::Time time )
{
  QString name = display->getName();
  int index = sync_source_selector_->findData( QVariant( (qulonglong)display ) );
  if ( index < 0 )
  {
    sync_source_selector_->addItem( name, QVariant( (qulonglong)display ) );
  }
  else
  {
    sync_source_selector_->setItemText( index, name );
    if ( !pause_button_->isChecked() &&
        sync_source_selector_->currentIndex() == index )
    {
      switch ( sync_mode_selector_->currentIndex() )
      {
        case SyncOff:
          break;
        case SyncExact:
            vis_manager_->overrideROSTime( true, time, false );
          break;
        case SyncApprox:
          // store time offset for use in update()
          last_sync_delta_ = (ros::Time::now() - time).toSec();
          break;
      }
  }
  }
}

QLineEdit* TimePanel::makeTimeLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly( true );
  return label;
}

void TimePanel::fillTimeLabel( QLineEdit* label, double time )
{
  label->setText( QString::number( time, 'f', 2 ));
}

void TimePanel::update()
{
  fillTimeLabel( ros_time_label_, vis_manager_->getROSTime() );
  if ( !pause_button_->isChecked() &&
       sync_mode_selector_->currentIndex() == SyncApprox )
  {
    // adjust current time offset to sync source with exponential decay
    current_delta_ = 0.7*current_delta_ + 0.3*last_sync_delta_;
    vis_manager_->overrideROSTime( true, ros::Time::now()+ros::Duration(current_delta_), true );
  }
}

void TimePanel::pauseToggled( bool checked )
{
  vis_manager_->overrideROSTime( checked, ros::Time(vis_manager_->getROSTime()) );
}

void TimePanel::syncModeSelected( int mode )
{
  switch ( mode )
  {
    case SyncOff:
      vis_manager_->overrideROSTime( pause_button_->isChecked(), ros::Time(vis_manager_->getROSTime()) );
      break;
    case SyncExact:
    case SyncApprox:
      current_delta_ = 0;
      last_sync_delta_ = 0;
      break;
  }
}

} // namespace rviz

