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

#include "views_panel.h"
#include "render_panel.h"
#include "visualization_manager.h"
#include "view_controller.h"

#include <wx/textdlg.h>
#include <wx/confbase.h>

#include <boost/bind.hpp>

namespace rviz
{

class StringClientData : public wxClientData
{
public:
  StringClientData(const std::string s)
  : str(s)
  {}

  std::string str;
};

ViewsPanel::ViewsPanel( wxWindow* parent )
: ViewsPanelGenerated( parent )
, manager_(NULL)
{
}

ViewsPanel::~ViewsPanel()
{

}

void ViewsPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  manager_->getGeneralConfigLoadedSignal().connect( boost::bind( &ViewsPanel::onGeneralConfigLoaded, this, _1 ) );
  manager_->getGeneralConfigSavingSignal().connect( boost::bind( &ViewsPanel::onGeneralConfigSaving, this, _1 ) );
  manager_->getViewControllerTypeAddedSignal().connect( boost::bind( &ViewsPanel::onViewControllerTypeAdded, this, _1, _2 ) );
  manager_->getViewControllerTypeChangedSignal().connect( boost::bind( &ViewsPanel::onViewControllerTypeChanged, this, _1 ) );
}

void ViewsPanel::loadSelected()
{
  int index = views_list_->GetSelection();
  if (index != wxNOT_FOUND)
  {
    const View& view = views_[index];
    manager_->setTargetFrame(view.target_frame_);
    manager_->setCurrentViewControllerType(view.controller_class_);
    manager_->getCurrentViewController()->fromString(view.controller_config_);
    manager_->queueRender();
  }
}

void ViewsPanel::addView(const View& view)
{
  views_.push_back(view);

  std::stringstream ss;
  ss << view.name_ << "; Target=[" << view.target_frame_ << "] Type=[" << view.controller_class_ << "] Config=[" << view.controller_config_ << "]";

  views_list_->Append(wxString::FromAscii(ss.str().c_str()));
}

void ViewsPanel::save(const std::string& name)
{
  View view;
  view.target_frame_ = manager_->getTargetFrame();
  view.controller_class_ = manager_->getCurrentViewControllerType();
  view.name_ = name;
  view.controller_config_ = manager_->getCurrentViewController()->toString();

  addView(view);
}

void ViewsPanel::onViewControllerTypeAdded(const std::string& class_name, const std::string& name)
{
  camera_types_->Append( wxString::FromAscii(name.c_str()) );
  camera_types_->SetClientObject(camera_types_->GetCount() - 1, new StringClientData(class_name));

  if (camera_types_->GetCount() == 1)
  {
    camera_types_->SetSelection( 0 );
  }
}

void ViewsPanel::onViewControllerTypeChanged(ViewController* controller)
{
  int count = camera_types_->GetCount();
  for (int i = 0; i < count; ++i)
  {
    StringClientData* data = (StringClientData*)camera_types_->GetClientObject(i);
    if (data->str == controller->getClassName())
    {
      camera_types_->SetSelection(i);
      break;
    }
  }
}

void ViewsPanel::onCameraTypeSelected( wxCommandEvent& event )
{
  StringClientData* data = (StringClientData*)camera_types_->GetClientObject(camera_types_->GetSelection());
  manager_->setCurrentViewControllerType(data->str);
}

void ViewsPanel::onViewsClicked( wxCommandEvent& event )
{
}

void ViewsPanel::onViewsDClicked( wxCommandEvent& event )
{
  loadSelected();
}

void ViewsPanel::onLoadClicked( wxCommandEvent& event )
{
  loadSelected();
}

void ViewsPanel::onSaveClicked( wxCommandEvent& event )
{
  wxString name = wxGetTextFromUser(wxT("Name the View"), wxT("Name"), wxT("My View"), this);

  if (!name.IsEmpty())
  {
    save((const char*)name.fn_str());
  }
}

void ViewsPanel::onDeleteClicked( wxCommandEvent& event )
{
  int index = views_list_->GetSelection();
  if (index != wxNOT_FOUND)
  {
    views_.erase(views_.begin() + index);
    views_list_->Delete(index);
  }
}

void ViewsPanel::onGeneralConfigLoaded(const boost::shared_ptr<wxConfigBase>& config)
{
  int i = 0;
  while (1)
  {
    wxString type, target, cam_config, name;
    type.Printf( wxT("Views/%d/Type"), i );
    target.Printf( wxT("Views/%d/Target"), i );
    cam_config.Printf( wxT("Views/%d/Config"), i );
    name.Printf( wxT("Views/%d/Name"), i );

    wxString view_type, view_name, view_target, view_config;
    if ( !config->Read( type, &view_type ) )
    {
      break;
    }

    if ( !config->Read( name, &view_name ) )
    {
      break;
    }

    if ( !config->Read( target, &view_target ) )
    {
      break;
    }

    if ( !config->Read( cam_config, &view_config ) )
    {
      break;
    }

    View view;
    view.name_ = (const char*)view_name.fn_str();
    view.controller_class_ = (const char*)view_type.fn_str();
    view.target_frame_ = (const char*)view_target.fn_str();
    view.controller_config_ = (const char*)view_config.fn_str();

    addView(view);

    ++i;
  }
}

void ViewsPanel::onGeneralConfigSaving(const boost::shared_ptr<wxConfigBase>& config)
{
  V_View::const_iterator it = views_.begin();
  V_View::const_iterator end = views_.end();
  uint32_t i = 0;
  for (; it != end; ++it, ++i)
  {
    const View& view = *it;

    wxString type, target, cam_config, name;
    type.Printf( wxT("Views/%d/Type"), i );
    target.Printf( wxT("Views/%d/Target"), i );
    cam_config.Printf( wxT("Views/%d/Config"), i );
    name.Printf( wxT("Views/%d/Name"), i );

    config->Write(name, wxString::FromAscii(view.name_.c_str()));
    config->Write(type, wxString::FromAscii(view.controller_class_.c_str()));
    config->Write(target, wxString::FromAscii(view.target_frame_.c_str()));
    config->Write(cam_config, wxString::FromAscii(view.controller_config_.c_str()));
  }
}

} // namespace rviz

