
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
TimePanel::TimePanel(QWidget* parent) : Panel(parent)
{
  wall_time_label_ = makeTimeLabel();
  wall_elapsed_label_ = makeTimeLabel();
  ros_time_label_ = makeTimeLabel();
  ros_elapsed_label_ = makeTimeLabel();

  experimental_cb_ = new QCheckBox("Experimental");
  experimental_cb_->setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));

  pause_button_ = new QPushButton("Pause");
  pause_button_->setToolTip("Freeze ROS time.");
  pause_button_->setCheckable(true);

  sync_mode_selector_ = new QComboBox(this);
  sync_mode_selector_->addItem("Off");
  sync_mode_selector_->addItem("Exact");
  sync_mode_selector_->addItem("Approximate");
  sync_mode_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_mode_selector_->setToolTip(
      "Allows you to synchronize the ROS time and Tf transforms to a given source.");

  // choose time sync signal
  sync_source_selector_ = new QComboBox(this);
  sync_source_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_source_selector_->setToolTip("Time source to use for synchronization.");

  experimental_widget_ = new QWidget(this);
  QHBoxLayout* experimental_layout = new QHBoxLayout(this);
  experimental_layout->addWidget(pause_button_);
  experimental_layout->addWidget(new QLabel("Synchronization:"));
  experimental_layout->addWidget(sync_mode_selector_);
  experimental_layout->addWidget(new QLabel("Source:"));
  experimental_layout->addWidget(sync_source_selector_);
  experimental_layout->addSpacing(20);
  experimental_layout->setContentsMargins(0, 0, 20, 0);
  experimental_widget_->setLayout(experimental_layout);

  old_widget_ = new QWidget(this);
  QHBoxLayout* old_layout = new QHBoxLayout(this);
  old_layout->addWidget(new QLabel("ROS Elapsed:"));
  old_layout->addWidget(ros_elapsed_label_);
  old_layout->addWidget(new QLabel("Wall Time:"));
  old_layout->addWidget(wall_time_label_);
  old_layout->addWidget(new QLabel("Wall Elapsed:"));
  old_layout->addWidget(wall_elapsed_label_);
  old_layout->setContentsMargins(0, 0, 20, 0);
  old_widget_->setLayout(old_layout);

  QHBoxLayout* layout = new QHBoxLayout(this);

  layout->addWidget(experimental_widget_);
  layout->addWidget(new QLabel("ROS Time:"));
  layout->addWidget(ros_time_label_);
  layout->addWidget(old_widget_);
  layout->addStretch(100);
  layout->addWidget(experimental_cb_);

  layout->addStretch();
  layout->setContentsMargins(11, 5, 11, 5);

  connect(experimental_cb_, SIGNAL(toggled(bool)), this, SLOT(experimentalToggled(bool)));
  connect(pause_button_, SIGNAL(toggled(bool)), this, SLOT(pauseToggled(bool)));
  connect(sync_mode_selector_, SIGNAL(activated(int)), this, SLOT(syncModeSelected(int)));
  connect(sync_source_selector_, SIGNAL(activated(int)), this, SLOT(syncSourceSelected(int)));
}

void TimePanel::onInitialize()
{
  connect(vis_manager_, SIGNAL(preUpdate()), this, SLOT(update()));

  DisplayGroup* display_group = vis_manager_->getRootDisplayGroup();
  onDisplayAdded(display_group);

  syncModeSelected(0);
  pauseToggled(false);
}

void TimePanel::load(const Config& config)
{
  Panel::load(config);
  int sync_mode;
  if (config.mapGetInt("SyncMode", &sync_mode))
  {
    sync_mode_selector_->setCurrentIndex(sync_mode);
    syncModeSelected(sync_mode);
  }
  config.mapGetString("SyncSource", &config_sync_source_);
  bool experimental = false;
  config.mapGetBool("Experimental", &experimental);
  experimental_cb_->setChecked(experimental);
  experimentalToggled(experimental);
}

void TimePanel::save(Config config) const
{
  Panel::save(config);
  config.mapSetValue("SyncMode", sync_mode_selector_->currentIndex());
  config.mapSetValue("SyncSource", sync_source_selector_->currentText());
  config.mapSetValue("Experimental", experimental_cb_->checkState() == Qt::Checked);
}

void TimePanel::onDisplayAdded(Display* display)
{
  DisplayGroup* display_group = qobject_cast<DisplayGroup*>(display);
  if (display_group)
  {
    connect(display_group, SIGNAL(displayAdded(rviz::Display*)), this,
            SLOT(onDisplayAdded(rviz::Display*)));
    connect(display_group, SIGNAL(displayRemoved(rviz::Display*)), this,
            SLOT(onDisplayRemoved(rviz::Display*)));

    for (int i = 0; i < display_group->numDisplays(); i++)
    {
      rviz::Display* display = display_group->getDisplayAt(i);
      onDisplayAdded(display);
    }
  }
  else
  {
    connect(display, SIGNAL(timeSignal(rviz::Display*, ros::Time)), this,
            SLOT(onTimeSignal(rviz::Display*, ros::Time)));
  }
}

void TimePanel::onDisplayRemoved(Display* display)
{
  QString name = display->getName();
  int index = sync_source_selector_->findData(QVariant((qulonglong)display));
  if (index >= 0)
  {
    sync_source_selector_->removeItem(index);
  }
}

void TimePanel::onTimeSignal(Display* display, ros::Time time)
{
  QString name = display->getName();
  int index = sync_source_selector_->findData(QVariant((qulonglong)display));

  // if we loaded the sync source name from the config, we need to
  // switch to it as soon as we get a signal
  if (index < 0 && name == config_sync_source_)
  {
    sync_source_selector_->addItem(name, QVariant((qulonglong)display));
    index = sync_source_selector_->findData(QVariant((qulonglong)display));
    sync_source_selector_->setCurrentIndex(index);
    config_sync_source_.clear();
  }

  if (index < 0)
  {
    sync_source_selector_->addItem(name, QVariant((qulonglong)display));
  }
  else
  {
    sync_source_selector_->setItemText(index, name);
    if (sync_source_selector_->currentIndex() == index)
    {
      vis_manager_->getFrameManager()->syncTime(time);
    }
  }
}

QLineEdit* TimePanel::makeTimeLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly(true);
  return label;
}

void TimePanel::fillTimeLabel(QLineEdit* label, double time)
{
  label->setText(QString::number(time, 'f', 2));
}

void TimePanel::update()
{
  fillTimeLabel(ros_time_label_, vis_manager_->getROSTime());
  fillTimeLabel(ros_elapsed_label_, vis_manager_->getROSTimeElapsed());
  fillTimeLabel(wall_time_label_, vis_manager_->getWallClock());
  fillTimeLabel(wall_elapsed_label_, vis_manager_->getWallClockElapsed());
}

void TimePanel::pauseToggled(bool checked)
{
  vis_manager_->getFrameManager()->setPause(checked);
}

void TimePanel::experimentalToggled(bool checked)
{
  old_widget_->setVisible(!checked);
  experimental_widget_->setVisible(checked);
  if (vis_manager_ && vis_manager_->getFrameManager())
  {
    if (!checked)
    {
      pauseToggled(false);
      syncModeSelected(0);
    }
    else
    {
      pauseToggled(pause_button_->isChecked());
      syncModeSelected(sync_mode_selector_->currentIndex());
    }
  }
}

void TimePanel::syncSourceSelected(int /*index*/)
{
  // clear whatever was loaded from the config
  config_sync_source_.clear();
  vis_manager_->notifyConfigChanged();
}

void TimePanel::syncModeSelected(int mode)
{
  vis_manager_->getFrameManager()->setSyncMode((FrameManager::SyncMode)mode);
  sync_source_selector_->setEnabled(mode != FrameManager::SyncOff);
  vis_manager_->notifyConfigChanged();
}

} // namespace rviz
