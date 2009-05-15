
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

#include "selection_panel.h"
#include "visualization_manager.h"
#include "selection/selection_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include <wx/timer.h>

#include <wx/propgrid/propgrid.h>
#include <boost/bind.hpp>

namespace rviz
{

SelectionPanel::SelectionPanel( wxWindow* parent )
: wxPanel( parent, wxID_ANY )
, manager_(NULL)
, setting_(false)
{
  wxBoxSizer* top_sizer = new wxBoxSizer(wxVERTICAL);

  property_grid_ = new wxPropertyGrid( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );
  top_sizer->Add(property_grid_, 1, wxEXPAND, 5);
  SetSizer(top_sizer);

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( SelectionPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( SelectionPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( SelectionPanel::onPropertySelected ), NULL, this );

  property_grid_->SetCaptionBackgroundColour( wxColour( 2, 0, 174 ) );
  property_grid_->SetCaptionForegroundColour( *wxLIGHT_GREY );

  property_manager_ = new PropertyManager();
  property_manager_->setPropertyGrid(property_grid_);

  refresh_timer_ = new wxTimer( this );
  refresh_timer_->Start( 200 );
  Connect( refresh_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( SelectionPanel::onUpdate ), NULL, this );
}

SelectionPanel::~SelectionPanel()
{
  Disconnect( refresh_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( SelectionPanel::onUpdate ), NULL, this );
  refresh_timer_->Stop();
  delete refresh_timer_;

  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( SelectionPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( SelectionPanel::onPropertyChanged ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( SelectionPanel::onPropertySelected ), NULL, this );
  property_grid_->Destroy();
}

void SelectionPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  manager_->getSelectionManager()->getSelectionAddedSignal().connect( boost::bind( &SelectionPanel::onSelectionAdded, this, _1 ) );
  manager_->getSelectionManager()->getSelectionRemovedSignal().connect( boost::bind( &SelectionPanel::onSelectionRemoved, this, _1 ) );
  manager_->getSelectionManager()->getSelectionSetSignal().connect( boost::bind( &SelectionPanel::onSelectionSet, this, _1 ) );
  manager_->getSelectionManager()->getSelectionSettingSignal().connect( boost::bind( &SelectionPanel::onSelectionSetting, this, _1 ) );
}

void SelectionPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  property_manager_->propertyChanging( event );
}

void SelectionPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  property_manager_->propertyChanged( event );
}

void SelectionPanel::onPropertySelected( wxPropertyGridEvent& event )
{
}

void SelectionPanel::onSelectionRemoved(const SelectionRemovedArgs& args)
{
  if (setting_)
  {
    return;
  }

  property_grid_->Freeze();

  SelectionManager* sel_manager = manager_->getSelectionManager();

  M_Picked::const_iterator it = args.removed_.begin();
  M_Picked::const_iterator end = args.removed_.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandlerPtr handler = sel_manager->getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->destroyProperties(picked, property_manager_);
  }

  //property_grid_->Sort(property_grid_->GetRoot());

  property_grid_->Thaw();
}

void SelectionPanel::onSelectionAdded(const SelectionAddedArgs& args)
{
  property_grid_->Freeze();

  SelectionManager* sel_manager = manager_->getSelectionManager();

  M_Picked::const_iterator it = args.added_.begin();
  M_Picked::const_iterator end = args.added_.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandlerPtr handler = sel_manager->getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->createProperties(picked, property_manager_);
  }

  property_grid_->Sort(property_grid_->GetRoot());

  property_grid_->Thaw();
}

void SelectionPanel::onSelectionSetting(const SelectionSettingArgs& args)
{
  setting_ = true;

  property_grid_->Freeze();
  property_manager_->clear();
}

void SelectionPanel::onSelectionSet(const SelectionSetArgs& args)
{
  setting_ = false;

  property_grid_->Thaw();
}

void SelectionPanel::onUpdate( wxTimerEvent& event )
{
  property_grid_->Freeze();

  SelectionManager* sel_manager = manager_->getSelectionManager();
  const M_Picked& selection = sel_manager->getSelection();
  M_Picked::const_iterator it = selection.begin();
  M_Picked::const_iterator end = selection.end();
  for (; it != end; ++it)
  {
    CollObjectHandle handle = it->first;
    SelectionHandlerPtr handler = sel_manager->getHandler(handle);

    handler->updateProperties();
  }

  property_manager_->update();
  //property_grid_->Sort(property_grid_->GetRoot());

  property_grid_->Thaw();
}

} // namespace rviz
