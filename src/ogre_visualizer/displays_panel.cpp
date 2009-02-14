
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

#include "displays_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "new_display_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>
#include <wx/artprov.h>

#include <boost/bind.hpp>

namespace ogre_vis
{

DisplaysPanel::DisplaysPanel( wxWindow* parent )
: DisplaysPanelGenerated( parent )
, manager_(NULL)
, selected_display_( NULL )
{
  property_grid_ = new wxPropertyGrid( properties_panel_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  properties_panel_sizer_->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( DisplaysPanel::onPropertySelected ), NULL, this );

  property_grid_->SetCaptionBackgroundColour( wxColour( 2, 0, 174 ) );
  property_grid_->SetCaptionForegroundColour( *wxLIGHT_GREY );

  up_button_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_UP, wxART_OTHER, wxSize(16,16) ) );
  down_button_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_DOWN, wxART_OTHER, wxSize(16,16) ) );
}

DisplaysPanel::~DisplaysPanel()
{
  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanged ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( DisplaysPanel::onPropertySelected ), NULL, this );
  property_grid_->Destroy();
}

void DisplaysPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;
  manager_->getDisplayStateSignal().connect( boost::bind( &DisplaysPanel::onDisplayStateChanged, this, _1 ) );
}

void DisplaysPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanging( event );
}

void DisplaysPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanged( event );
}

void DisplaysPanel::onPropertySelected( wxPropertyGridEvent& event )
{
  wxPGProperty* pg_property = event.GetProperty();

  selected_display_ = NULL;

  if ( !pg_property )
  {
    return;
  }

  void* client_data = pg_property->GetClientData();
  if ( client_data )
  {
    PropertyBase* property = reinterpret_cast<PropertyBase*>(client_data);

    void* user_data = property->getUserData();
    if ( user_data )
    {
      Display* display = reinterpret_cast<Display*>(user_data);

      if ( manager_->isValidDisplay( display ) )
      {
        selected_display_ = display;
      }
    }
  }
}

void DisplaysPanel::onNewDisplay( wxCommandEvent& event )
{
  V_string types;
  V_string descriptions;
  manager_->getRegisteredTypes( types, descriptions );

  S_string current_display_names;
  manager_->getDisplayNames(current_display_names);

  NewDisplayDialog dialog( this, types, descriptions, current_display_names );
  while (1)
  {
    if ( dialog.ShowModal() == wxOK )
    {
      std::string type = dialog.getTypeName();
      std::string name = dialog.getDisplayName();

      if ( manager_->getDisplay( name ) != NULL )
      {
        wxMessageBox( wxT("A display with that name already exists!"), wxT("Invalid name"), wxICON_ERROR | wxOK, this );
        continue;
      }

      Display* display = manager_->createDisplay( type, name, true );
      ROS_ASSERT(display);
      (void)display;

      break;
    }
    else
    {
      break;
    }
  }
}

void DisplaysPanel::onDeleteDisplay( wxCommandEvent& event )
{
  if ( !selected_display_ )
  {
    return;
  }

  manager_->removeDisplay( selected_display_ );
  selected_display_ = NULL;
}

void DisplaysPanel::onMoveUp( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    manager_->moveDisplayUp( selected_display_ );
  }
}

void DisplaysPanel::onMoveDown( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    manager_->moveDisplayDown( selected_display_ );
  }
}

void DisplaysPanel::onDisplayStateChanged( Display* display )
{
  DisplayInfo* info = manager_->getDisplayInfo( display );
  ROS_ASSERT( info );
  wxPGProperty* property = info->category_->getPGProperty();
  ROS_ASSERT( property );

  wxPGCell* cell = property->GetCell( 0 );
  if ( !cell )
  {
    cell = new wxPGCell( property->GetLabel(), wxNullBitmap, *wxLIGHT_GREY, *wxGREEN );
    property->SetCell( 0, cell );
  }

  if ( display->isEnabled() )
  {
    cell->SetBgCol( wxColour( 32, 116, 38 ) );
  }
  else
  {
    cell->SetBgCol( wxColour( 151, 24, 41 ) );
  }
}

} // namespace ogre_vis
