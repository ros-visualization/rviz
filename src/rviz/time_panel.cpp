
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

#include "time_panel.h"
#include "visualization_manager.h"

#include <boost/bind.hpp>

#include <wx/wx.h>

namespace rviz
{

TimePanel::TimePanel( wxWindow* parent )
: TimePanelGenerated( parent )
, manager_(NULL)
{
}

TimePanel::~TimePanel()
{
}

void TimePanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  manager_->getTimeChangedSignal().connect(boost::bind(&TimePanel::onTimeChanged, this));
}

void TimePanel::onTimeChanged()
{
  wxString str;
  str.Printf(wxT("%f"), manager_->getWallClock());
  wall_time_->SetValue(str);

  str.Printf(wxT("%f"), manager_->getWallClockElapsed());
  wall_elapsed_->SetValue(str);

  str.Printf(wxT("%f"), manager_->getROSTime());
  ros_time_->SetValue(str);

  str.Printf(wxT("%f"), manager_->getROSTimeElapsed());
  ros_elapsed_->SetValue(str);
}

void TimePanel::onReset(wxCommandEvent& event)
{
  manager_->resetTime();
}


} // namespace rviz

