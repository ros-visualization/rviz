/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "splash_screen.h"

#include <wx/dcclient.h>

#define TEXT_AREA_HEIGHT 16

namespace rviz
{

SplashScreen::SplashScreen(wxWindow* parent, const wxBitmap& background)
: wxFrame(0, wxID_ANY, wxT(""), wxDefaultPosition, wxDefaultSize, wxFRAME_NO_TASKBAR|wxFRAME_FLOAT_ON_PARENT)
, background_(background)
{
  Connect(wxEVT_PAINT, wxPaintEventHandler(SplashScreen::onPaint), 0, this);

  wxSize size = wxSize(background_.GetWidth(), background_.GetHeight());
  size.SetHeight(size.GetHeight() + TEXT_AREA_HEIGHT);
  SetSize(size);

  wxSize display_size = wxGetDisplaySize();
  SetPosition(wxPoint(display_size.GetWidth()/2 - size.GetWidth()/2, display_size.GetHeight()/2 - size.GetHeight()/2));
}

SplashScreen::~SplashScreen()
{

}

void SplashScreen::setState(const std::string& state)
{
  state_ = state;
  Refresh();

  wxSafeYield(this, true);
}

void SplashScreen::onPaint(wxPaintEvent& evt)
{
  wxPaintDC dc(this);

  wxSize text_size = dc.GetTextExtent(wxString::FromAscii(state_.c_str()));

  dc.DrawBitmap(background_, 0, 0);
  dc.SetBrush(*wxWHITE_BRUSH);
  dc.DrawRectangle(0, background_.GetHeight(), background_.GetWidth(), TEXT_AREA_HEIGHT);
  dc.DrawText(wxString::FromAscii(state_.c_str()), 4, background_.GetHeight() + (TEXT_AREA_HEIGHT/2) - (text_size.GetHeight()/2));
}

}
