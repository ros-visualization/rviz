
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
#include "properties/property.h"
#include "properties/property_manager.h"
#include "tools/tool.h"
#include "config.h"

#include <boost/bind.hpp>

static const std::string PROPERTY_GRID_CONFIG("Property Grid State");

namespace rviz
{

ToolPropertiesPanel::ToolPropertiesPanel( QWidget* parent )
  : PropertyTreeWidget( parent )
  , manager_( NULL )
{
}

void ToolPropertiesPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;

  manager_->getToolPropertyManager()->setPropertyTreeWidget( this );

  connect( manager_, SIGNAL( toolAdded( Tool* )), this, SLOT( onToolAdded( Tool* )));
}

void ToolPropertiesPanel::onToolAdded( Tool* tool )
{
  if( tool->hasProperties() )
  {
    std::string name = tool->getName();
    CategoryPropertyWPtr cat = manager_->getToolPropertyManager()->createCategory( name, "", CategoryPropertyWPtr(), tool );
    tool->enumerateProperties( manager_->getToolPropertyManager(), cat );
  }
}

void ToolPropertiesPanel::onDisplaysConfigLoaded(const boost::shared_ptr<Config>& config)
{
/////  std::string grid_state;
/////  if ( config->get( PROPERTY_GRID_CONFIG, &grid_state ) )
/////  {
/////    property_grid_->RestoreEditableState( grid_state );
/////  }
}

void ToolPropertiesPanel::onDisplaysConfigSaving(const boost::shared_ptr<Config>& config)
{
/////  config->set( PROPERTY_GRID_CONFIG, property_grid_->SaveEditableState() );
}

} // namespace rviz
