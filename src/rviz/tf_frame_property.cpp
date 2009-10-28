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

#include "tf_frame_property.h"
#include "properties/forwards.h"
#include "frame_manager.h"

#include <ros/console.h>

#include <tf/transform_listener.h>

namespace rviz
{

IMPLEMENT_DYNAMIC_CLASS(TFFramePGEditor, wxPGComboBoxEditor);
IMPLEMENT_DYNAMIC_CLASS(TFFramePGProperty, wxEditEnumProperty);

TFFramePGEditor::TFFramePGEditor()
{

}

wxPGWindowList TFFramePGEditor::CreateControls(wxPropertyGrid *propgrid, wxPGProperty *property, const wxPoint &pos, const wxSize &size) const
{
  property->GetChoices().Clear();
  property->GetChoices().Add(wxT(FIXED_FRAME_STRING));

  typedef std::vector<std::string> V_string;
  V_string frames;
  FrameManager::instance()->getTFClient()->getFrameStrings( frames );
  std::sort(frames.begin(), frames.end());

  V_string::iterator it = frames.begin();
  V_string::iterator end = frames.end();
  for (; it != end; ++it)
  {
    const std::string& frame = *it;
    if (frame.empty())
    {
      continue;
    }

    property->GetChoices().Add(wxString::FromAscii(frame.c_str()));
  }

  return wxPGComboBoxEditor::CreateControls(propgrid, property, pos, size);
}

TFFramePGProperty::TFFramePGProperty()
{

}

TFFramePGProperty::TFFramePGProperty(const wxString& label, const wxString& name, const wxString& value )
: wxEditEnumProperty( label, name )
{
  SetValue(value);
}

} // namespace rviz

