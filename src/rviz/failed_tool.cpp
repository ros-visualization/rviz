/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <QMessageBox>

#include <rviz/display_context.h>
#include <rviz/window_manager_interface.h>

#include <rviz/failed_tool.h>

namespace rviz
{
FailedTool::FailedTool(const QString& desired_class_id, const QString& error_message)
  : error_message_(error_message)
{
  setClassId(desired_class_id);
}

QString FailedTool::getDescription() const
{
  return "The class required for this tool, '" + getClassId() +
         "', could not be loaded.<br><b>Error:</b><br>" + error_message_;
}

void FailedTool::load(const Config& config)
{
  saved_config_ = config;
}

void FailedTool::save(Config config) const
{
  if (saved_config_.isValid())
  {
    config.copy(saved_config_);
  }
}

void FailedTool::activate()
{
  QWidget* parent = nullptr;
  if (context_->getWindowManager())
  {
    parent = context_->getWindowManager()->getParentWindow();
  }
  QMessageBox::critical(parent, "Tool '" + getName() + "'unavailable.", getDescription());
}

} // end namespace rviz
