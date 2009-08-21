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

#include <sstream>

#include <wx/msgdlg.h>

namespace rviz
{

class NewDisplayDialogTreeItemData : public wxTreeItemData
{
public:
  int32_t index;
};

NewDisplayDialog::NewDisplayDialog( wxWindow* parent, const L_Plugin& plugins, const S_string& current_display_names )
: NewDisplayDialogGenerated( parent )
, current_display_names_(current_display_names)
{
  wxTreeItemId root = types_->AddRoot(wxT(""));

  L_Plugin::const_iterator pit = plugins.begin();
  L_Plugin::const_iterator pend = plugins.end();
  for (; pit != pend; ++pit)
  {
    const PluginPtr& plugin = *pit;

    wxTreeItemId parent = types_->AppendItem(root, wxString::FromAscii(plugin->getName().c_str()));

    L_DisplayTypeInfo::const_iterator it = plugin->getDisplayTypeInfoList().begin();
    L_DisplayTypeInfo::const_iterator end = plugin->getDisplayTypeInfoList().end();
    for ( ; it != end; ++it )
    {
      const DisplayTypeInfoPtr& info = *it;

      NewDisplayDialogTreeItemData* data = new NewDisplayDialogTreeItemData;
      data->index = typeinfo_.size();
      types_->AppendItem( parent, wxString::FromAscii( info->display_name.c_str() ), -1, -1, data );

      DisplayTypeInfoWithPlugin t;
      t.plugin = plugin;
      t.typeinfo = info;
      typeinfo_.push_back(t);
    }
  }

  type_description_->Connect(wxEVT_COMMAND_HTML_LINK_CLICKED, wxHtmlLinkEventHandler(NewDisplayDialog::onLinkClicked), NULL, this);

  types_->ExpandAll();
}

int32_t NewDisplayDialog::getSelectionIndex()
{
  wxTreeItemId sel = types_->GetSelection();
  if (!sel.IsOk())
  {
    return -1;
  }

  NewDisplayDialogTreeItemData* data = (NewDisplayDialogTreeItemData*)types_->GetItemData(sel);
  if (!data)
  {
    return -1;
  }

  int32_t index = data->index;
  return index;
}

void NewDisplayDialog::onDisplaySelected( wxTreeEvent& event )
{
  int32_t index = getSelectionIndex();
  if (index < 0)
  {
    name_->SetValue(wxT(""));
    type_description_->SetPage(wxT(""));
    return;
  }

  const DisplayTypeInfoPtr& info = typeinfo_[index].typeinfo;
  type_description_->SetPage( wxString::FromAscii( info->help_description.c_str() ) );

  int counter = 1;
  std::string name;
  do
  {
    std::stringstream ss;
    ss << info->display_name;

    if (counter > 1)
    {
      ss << counter;
    }

    ++counter;

    name = ss.str();
  } while(current_display_names_.find(name) != current_display_names_.end());

  name_->SetValue(wxString::FromAscii(name.c_str()));

  Layout();
}

void NewDisplayDialog::onLinkClicked(wxHtmlLinkEvent& event)
{
  wxLaunchDefaultBrowser(event.GetLinkInfo().GetHref());
}

void NewDisplayDialog::onDisplayDClick( wxMouseEvent& event )
{
  int32_t index = getSelectionIndex();
  if (index < 0)
  {
    return;
  }

  if ( name_->GetValue().IsEmpty() )
  {
    wxMessageBox( wxT("You must enter a name!"), wxT("No name"), wxICON_ERROR | wxOK, this );
    return;
  }

  EndModal(wxOK);
}

void NewDisplayDialog::onOK( wxCommandEvent& event )
{
  int32_t index = getSelectionIndex();
  if (index < 0)
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

std::string NewDisplayDialog::getClassName()
{
  int32_t index = getSelectionIndex();
  if (index < 0)
  {
    return "";
  }

  return typeinfo_[index].typeinfo->class_name;
}

std::string NewDisplayDialog::getDisplayName()
{
  return (const char*)name_->GetValue().mb_str();
}

std::string NewDisplayDialog::getPackageName()
{
  int32_t index = getSelectionIndex();
  if (index < 0)
  {
    return "";
  }

  return typeinfo_[index].plugin->getPackageName();
}

} // rviz

