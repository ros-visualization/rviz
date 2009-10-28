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

#include "loading_dialog.h"

#include <wx/dcclient.h>

namespace rviz
{

LoadingDialog::LoadingDialog(wxWindow* parent)
: wxDialog(0, wxID_ANY, wxT(""), wxDefaultPosition, wxSize(400, 32), 0)
{
  Connect(wxEVT_PAINT, wxPaintEventHandler(LoadingDialog::onPaint), 0, this);

  wxSize size = GetSize();
  wxSize parent_size = parent->GetSize();
  wxPoint parent_pos = parent->GetPosition();
  SetPosition(wxPoint(parent_pos.x + parent_size.GetWidth()/2 - size.GetWidth()/2, parent_pos.y + parent_size.GetHeight()/2 - size.GetHeight()/2));
}

LoadingDialog::~LoadingDialog()
{

}

void LoadingDialog::setState(const std::string& state)
{
  state_ = state;
  Refresh();

  wxSafeYield(this, true);
}

void LoadingDialog::onPaint(wxPaintEvent& evt)
{
  wxPaintDC dc(this);

  wxSize text_size = dc.GetTextExtent(wxString::FromAscii(state_.c_str()));
  wxSize size = GetSize();

  dc.SetBrush(*wxWHITE_BRUSH);
  dc.DrawRectangle(0, 0, size.GetWidth(), size.GetHeight());
  dc.DrawText(wxString::FromAscii(("Loading... " + state_).c_str()), 4, (size.GetHeight()/2) - (text_size.GetHeight()/2));
}

}
