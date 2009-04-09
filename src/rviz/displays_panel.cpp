
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
#include <wx/msgdlg.h>
#include <wx/confbase.h>
#include <wx/artprov.h>

#include <boost/bind.hpp>

static const wxString PROPERTY_GRID_CONFIG(wxT("Property Grid State"));

namespace rviz
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
  manager_->getDisplayAddingSignal().connect( boost::bind( &DisplaysPanel::onDisplayAdding, this, _1 ) );
  manager_->getDisplayAddedSignal().connect( boost::bind( &DisplaysPanel::onDisplayAdded, this, _1 ) );
  manager_->getDisplayRemovingSignal().connect( boost::bind( &DisplaysPanel::onDisplayRemoving, this, _1 ) );
  manager_->getDisplayRemovedSignal().connect( boost::bind( &DisplaysPanel::onDisplayRemoved, this, _1 ) );
  manager_->getDisplaysRemovingSignal().connect( boost::bind( &DisplaysPanel::onDisplaysRemoving, this, _1 ) );
  manager_->getDisplaysRemovedSignal().connect( boost::bind( &DisplaysPanel::onDisplaysRemoved, this, _1 ) );
  manager_->getDisplaysConfigLoadedSignal().connect( boost::bind( &DisplaysPanel::onDisplaysConfigLoaded, this, _1 ) );
  manager_->getDisplaysConfigSavingSignal().connect( boost::bind( &DisplaysPanel::onDisplaysConfigSaving, this, _1 ) );

  manager_->getPropertyManager()->setPropertyGrid(property_grid_);

  sortDisplays();
}

void DisplaysPanel::sortDisplays()
{
  property_grid_->Freeze();
  property_grid_->Sort(property_grid_->GetRoot());
  property_grid_->Refresh();
  property_grid_->Thaw();
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

void setDisplayCategoryLabel(const Display* display, int index)
{
  char buf[1024];
  snprintf( buf, 1024, "%02d. %s (%s)", index + 1, display->getName().c_str(), display->getType() );
  display->getCategory().lock()->setLabel(buf);
}

void DisplaysPanel::onMoveUp( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    M_U32ToDisplay::iterator it = display_map_.find(selected_display_);
    ROS_ASSERT(it != display_map_.end());

    if (it->second == 0)
    {
      return;
    }

    --it->second;
    setDisplayCategoryLabel(selected_display_, it->second);

    uint32_t old_index = it->second;

    it = display_map_.begin();
    M_U32ToDisplay::iterator end = display_map_.end();
    for (;it != end; ++it)
    {
      if (it->second == old_index && it->first != selected_display_)
      {
        ++it->second;
        setDisplayCategoryLabel(it->first, it->second);
      }
    }

    sortDisplays();

    manager_->moveDisplayUp(selected_display_);
  }
}

void DisplaysPanel::onMoveDown( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    M_U32ToDisplay::iterator it = display_map_.find(selected_display_);
    ROS_ASSERT(it != display_map_.end());

    if (it->second == display_map_.size() - 1)
    {
      return;
    }

    ++it->second;
    setDisplayCategoryLabel(selected_display_, it->second);

    uint32_t old_index = it->second;

    it = display_map_.begin();
    M_U32ToDisplay::iterator end = display_map_.end();
    for (;it != end; ++it)
    {
      if (it->second == old_index && it->first != selected_display_)
      {
        --it->second;
        setDisplayCategoryLabel(it->first, it->second);
      }
    }

    sortDisplays();

    manager_->moveDisplayDown(selected_display_);
  }
}

void DisplaysPanel::onDisplayStateChanged( Display* display )
{
  wxPGProperty* property = display->getCategory().lock()->getPGProperty();
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

void DisplaysPanel::onDisplayAdding( Display* display )
{
  property_grid_->Freeze();
}

void DisplaysPanel::onDisplayAdded( Display* display )
{
  int index = display_map_.size();
  bool inserted = display_map_.insert(std::make_pair(display, index)).second;
  ROS_ASSERT(inserted);
  setDisplayCategoryLabel(display, index);

  property_grid_->Refresh();
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplayRemoving( Display* display )
{
  property_grid_->Freeze();
}

void DisplaysPanel::onDisplayRemoved( Display* display )
{
  M_U32ToDisplay::iterator it = display_map_.find(display);
  ROS_ASSERT(it != display_map_.end());

  uint32_t index = it->second;

  display_map_.erase(it);

  it = display_map_.begin();
  M_U32ToDisplay::iterator end = display_map_.end();
  for (;it != end; ++it)
  {
    if (it->second > index)
    {
      --it->second;
      setDisplayCategoryLabel(it->first, it->second);
    }
  }

  sortDisplays();

  property_grid_->Refresh();
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplaysRemoving( const V_Display& displays )
{
  property_grid_->Freeze();
}

void DisplaysPanel::onDisplaysRemoved( const V_Display& displays )
{
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplaysConfigLoaded(wxConfigBase* config)
{
  wxString grid_state;
  if ( config->Read( PROPERTY_GRID_CONFIG, &grid_state ) )
  {
    property_grid_->RestoreEditableState( grid_state );
  }
}

void DisplaysPanel::onDisplaysConfigSaving(wxConfigBase* config)
{
  config->Write( PROPERTY_GRID_CONFIG, property_grid_->SaveEditableState() );
}

} // namespace rviz
