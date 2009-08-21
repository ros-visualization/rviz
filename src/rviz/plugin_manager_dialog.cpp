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

#include "plugin_manager_dialog.h"
#include "plugin/plugin_manager.h"
#include "plugin/plugin.h"

#include <ros/console.h>

#include <wx/checkbox.h>
#include <wx/msgdlg.h>

namespace rviz
{

PluginManagerDialog::PluginManagerDialog(wxWindow* parent, PluginManager* manager)
: PluginManagerDialogGenerated(parent)
, manager_(manager)
{
  const L_Plugin& plugins = manager->getPlugins();
  L_Plugin::const_iterator it = plugins.begin();
  L_Plugin::const_iterator end = plugins.end();
  for (; it != end; ++it)
  {
    const PluginPtr& plugin = *it;

    wxPanel* p = new wxPanel(scrolled_window_);
    p->SetForegroundColour(*wxLIGHT_GREY);
    if (plugin->isLoaded())
    {
      p->SetBackgroundColour(wxColour( 32, 116, 38 ));
    }
    else
    {
      p->SetBackgroundColour(wxColour( 151, 24, 41 ));
    }
    plugins_sizer_->Add(p, 0, wxEXPAND);

    uint32_t row_index = rows_.size();

    wxBoxSizer* s = new wxBoxSizer(wxHORIZONTAL);
    wxCheckBox* loaded_cb = new wxCheckBox(p, wxID_ANY, wxT("Loaded"));
    loaded_cb->SetValue(plugin->isLoaded());
    loaded_cb->SetClientData((void*)row_index);
    wxCheckBox* autoload_cb = new wxCheckBox(p, wxID_ANY, wxT("Auto Load"));
    autoload_cb->SetValue(plugin->isAutoLoad());
    autoload_cb->SetClientData((void*)row_index);
    s->Add(loaded_cb, 0, wxALIGN_CENTER);
    s->Add(autoload_cb, 0, wxALIGN_CENTER);
    s->Add(new wxStaticText(p, wxID_ANY, wxString::FromAscii(plugin->getName().c_str()), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT), 1, wxALIGN_CENTER|wxEXPAND);
    p->SetSizer(s);

    loaded_cb->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(PluginManagerDialog::onLoadedChecked), 0, this);
    autoload_cb->Connect(wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(PluginManagerDialog::onAutoLoadChecked), 0, this);

    Row r;
    r.panel = p;
    r.loaded = loaded_cb;
    r.autoload = autoload_cb;
    r.plugin = plugin.get();
    rows_.push_back(r);
  }
}

void PluginManagerDialog::onLoadedChecked(wxCommandEvent& evt)
{
  wxCheckBox* box = (wxCheckBox*)evt.GetEventObject();
  uintptr_t row_index = (uintptr_t)box->GetClientData();
  Plugin* plugin = rows_[row_index].plugin;
  if (evt.IsChecked())
  {
    try
    {
      plugin->load();
      rows_[row_index].panel->SetBackgroundColour(wxColour( 32, 116, 38 ));
    }
    catch (std::runtime_error& e)
    {
      box->SetValue(false);
      wxMessageBox(wxString::FromAscii(e.what()), wxT("Error Loading Plugin"), wxOK|wxICON_ERROR, this);
    }
  }
  else
  {
    plugin->unload();
    rows_[row_index].panel->SetBackgroundColour(wxColour( 151, 24, 41 ));
  }
}

void PluginManagerDialog::onAutoLoadChecked(wxCommandEvent& evt)
{
  wxCheckBox* box = (wxCheckBox*)evt.GetEventObject();
  uintptr_t row_index = (uintptr_t)box->GetClientData();
  Plugin* plugin = rows_[row_index].plugin;

  plugin->setAutoLoad(evt.IsChecked());
}

} // namespace rviz
