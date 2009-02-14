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

#include "new_display_dialog.h"

#include <wx/wx.h>

#include <sstream>

namespace ogre_vis
{

NewDisplayDialog::NewDisplayDialog( wxWindow* parent, const V_string& types, const V_string& descriptions, const S_string& current_display_names )
: NewDisplayDialogGenerated( parent )
, descriptions_( descriptions )
, current_display_names_(current_display_names)
{
  V_string::const_iterator it = types.begin();
  V_string::const_iterator end = types.end();
  for ( ; it != end; ++it )
  {
    types_->Append( wxString::FromAscii( it->c_str() ) );
  }
}

void NewDisplayDialog::onDisplaySelected( wxCommandEvent& event )
{
  type_description_->SetValue( wxString::FromAscii( descriptions_[types_->GetSelection()].c_str() ) );

  int counter = 1;
  std::string name;
  do
  {
    std::stringstream ss;
    ss << (const char*)types_->GetStringSelection().fn_str();

    if (counter > 1)
    {
      ss << counter;
    }

    ++counter;

    name = ss.str();
  } while(current_display_names_.find(name) != current_display_names_.end());

  name_->SetValue(wxString::FromAscii(name.c_str()));
}

void NewDisplayDialog::onDisplayDClick( wxCommandEvent& event )
{
  if ( name_->GetValue().IsEmpty() )
  {
    wxMessageBox( wxT("You must enter a name!"), wxT("No name"), wxICON_ERROR | wxOK, this );
    return;
  }

  EndModal(wxOK);
}

void NewDisplayDialog::onOK( wxCommandEvent& event )
{
  if ( types_->GetSelection() == wxNOT_FOUND )
  {
    wxMessageBox( wxT("You must select a type!"), wxT("No selection"), wxICON_ERROR | wxOK, this );
    return;
  }

  if ( name_->GetValue().IsEmpty() )
  {
    wxMessageBox( wxT("You must enter a name!"), wxT("No name"), wxICON_ERROR | wxOK, this );
    return;
  }

  std::string name = (const char*)name_->GetValue().fn_str();
  if (current_display_names_.find(name) != current_display_names_.end())
  {
    wxMessageBox( wxT("A display with that name already exists!"), wxT("Name conflict"), wxICON_ERROR | wxOK, this );
    return;
  }

  EndModal(wxOK);
}

void NewDisplayDialog::onCancel( wxCommandEvent& event )
{
  EndModal(wxCANCEL);
}

void NewDisplayDialog::onNameEnter( wxCommandEvent& event )
{
  onOK( event );
}

std::string NewDisplayDialog::getTypeName()
{
  return (const char*)types_->GetStringSelection().mb_str();
}

std::string NewDisplayDialog::getDisplayName()
{
  return (const char*)name_->GetValue().mb_str();
}

} // ogre_vis

