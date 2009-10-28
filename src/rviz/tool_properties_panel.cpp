
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

#include "tool_properties_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "display_wrapper.h"
#include "new_display_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "plugin/type_registry.h"
#include "plugin/plugin_manager.h"
#include "plugin/plugin.h"
#include "tools/tool.h"

#include <wx/propgrid/propgrid.h>
#include <wx/msgdlg.h>
#include <wx/confbase.h>
#include <wx/artprov.h>

#include <boost/bind.hpp>

static const wxString PROPERTY_GRID_CONFIG(wxT("Property Grid State"));

namespace rviz
{

ToolPropertiesPanel::ToolPropertiesPanel( wxWindow* parent )
: wxPanel( parent, wxID_ANY )
, manager_(NULL)
{
  wxBoxSizer* top_sizer = new wxBoxSizer(wxVERTICAL);

  property_grid_ = new wxPropertyGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxPG_DEFAULT_STYLE );
  top_sizer->Add(property_grid_, 1, wxEXPAND, 5);
  SetSizer(top_sizer);

  property_grid_->SetExtraStyle(wxPG_EX_DISABLE_TLP_TRACKING);
  property_grid_->SetCaptionBackgroundColour( wxColour( 4, 89, 127 ) );
  property_grid_->SetCaptionForegroundColour( *wxWHITE );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertySelected ), NULL, this );
}

ToolPropertiesPanel::~ToolPropertiesPanel()
{
  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertyChanged ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( ToolPropertiesPanel::onPropertySelected ), NULL, this );
  property_grid_->Destroy();
}

void ToolPropertiesPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  manager_->getToolPropertyManager()->setPropertyGrid(property_grid_);
  manager_->getToolAddedSignal().connect(boost::bind(&ToolPropertiesPanel::onToolAdded, this, _1));
}

void ToolPropertiesPanel::onToolAdded(Tool* tool)
{
  if (tool->hasProperties())
  {
    std::string name = tool->getName();
    CategoryPropertyWPtr cat = manager_->getToolPropertyManager()->createCategory(name, "", CategoryPropertyWPtr(), tool);
    tool->enumerateProperties(manager_->getToolPropertyManager(), cat);
  }
}

void ToolPropertiesPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getToolPropertyManager()->propertyChanging( event );
}

void ToolPropertiesPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getToolPropertyManager()->propertyChanged( event );
}

void ToolPropertiesPanel::onPropertySelected( wxPropertyGridEvent& event )
{
  event.Skip();
}

void ToolPropertiesPanel::onDisplaysConfigLoaded(const boost::shared_ptr<wxConfigBase>& config)
{
  wxString grid_state;
  if ( config->Read( PROPERTY_GRID_CONFIG, &grid_state ) )
  {
    property_grid_->RestoreEditableState( grid_state );
  }
}

void ToolPropertiesPanel::onDisplaysConfigSaving(const boost::shared_ptr<wxConfigBase>& config)
{
  config->Write( PROPERTY_GRID_CONFIG, property_grid_->SaveEditableState() );
}

} // namespace rviz
