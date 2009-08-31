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

#include "ros_topic_property.h"
#include <rxtools/topic_display_dialog.h>

#include <ros/node.h>

using namespace rxtools;

namespace rviz
{

IMPLEMENT_DYNAMIC_CLASS(ROSTopicProperty, wxLongStringProperty);

bool ROSTopicDialogAdapter::DoShowDialog( wxPropertyGrid* propGrid, wxPGProperty* property )
{
  TopicDisplayDialog dialog(NULL, ros::Node::instance(), false, message_type_);

  if (dialog.ShowModal() == wxID_OK)
  {
    std::vector<std::string> selection;
    dialog.getSelection(selection);

    if (!selection.empty())
    {
      SetValue( wxString::FromAscii( selection[0].c_str() ) );
      return true;
    }
  }

  return false;
}

ROSTopicProperty::ROSTopicProperty()
{

}

ROSTopicProperty::ROSTopicProperty(const std::string& message_type, const wxString& label, const wxString& name, const wxString& value )
: wxLongStringProperty( label, name, value )
{
  checkForEmptyValue();
}

void ROSTopicProperty::OnSetValue()
{
  checkForEmptyValue();
}

void ROSTopicProperty::checkForEmptyValue()
{
  wxString str = m_value.GetString();

  wxPGCell* cell = GetCell(1);
  if (!cell)
  {
    cell = new wxPGCell(str, wxNullBitmap, wxNullColour, wxNullColour);
    SetCell(1, cell);
  }

  if (str.IsEmpty())
  {
    cell->SetBgCol(wxColour(255, 50, 0));
    cell->SetFgCol(wxColour(255, 255, 255));
    cell->SetText(wxT("Fill in topic here..."));
  }
  else
  {
    cell->SetBgCol(wxNullColour);
    cell->SetFgCol(wxNullColour);
    cell->SetText(str);
  }
}

} // namespace rviz

