
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

#include <QTimer>

#include "visualization_manager.h"
#include "selection/selection_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "selection_panel.h"

namespace rviz
{

SelectionPanel::SelectionPanel( QWidget* parent )
  : PropertyTreeWidget( parent )
  , manager_( NULL )
  , setting_( false )
{
  // Ignore change signals emitted by the tree widget. None of the
  // selection properties are editable, so the only change signals are
  // spurious and should be ignored.
  setIgnoreChanges( true );

  property_manager_ = new PropertyManager();
  property_manager_->setPropertyTreeWidget( this );
}

SelectionPanel::~SelectionPanel()
{
  delete property_manager_;
}

void SelectionPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  SelectionManager* sel_man = manager_->getSelectionManager();

  connect( sel_man, SIGNAL( selectionAdded( const M_Picked& )), this, SLOT( onSelectionAdded( const M_Picked& )));
  connect( sel_man, SIGNAL( selectionRemoved( const M_Picked& )), this, SLOT( onSelectionRemoved( const M_Picked& )));
  connect( sel_man, SIGNAL( selectionSet( const M_Picked&, const M_Picked& )), this, SLOT( onSelectionSet() ));
  connect( sel_man, SIGNAL( selectionSetting() ), this, SLOT( onSelectionSetting() ));

  QTimer* timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( onUpdate() ));
  timer->start( 200 );
}

void SelectionPanel::onSelectionRemoved( const M_Picked& removed )
{
  if (setting_)
  {
    return;
  }

  SelectionManager* sel_manager = manager_->getSelectionManager();

  M_Picked::const_iterator it = removed.begin();
  M_Picked::const_iterator end = removed.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandlerPtr handler = sel_manager->getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->destroyProperties(picked, property_manager_);
  }

  //property_grid_->Sort(property_grid_->GetRoot());
}

void SelectionPanel::onSelectionAdded( const M_Picked& added )
{
  SelectionManager* sel_manager = manager_->getSelectionManager();

  M_Picked::const_iterator it = added.begin();
  M_Picked::const_iterator end = added.end();
  for (; it != end; ++it)
  {
    const Picked& picked = it->second;
    SelectionHandlerPtr handler = sel_manager->getHandler(picked.handle);
    ROS_ASSERT(handler);

    handler->createProperties(picked, property_manager_);
  }
  sortItems( 0, Qt::AscendingOrder );
}

void SelectionPanel::onSelectionSetting()
{
  setting_ = true;

  property_manager_->clear();
}

void SelectionPanel::onSelectionSet()
{
  setting_ = false;
}

void SelectionPanel::onUpdate()
{
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
}

} // namespace rviz
