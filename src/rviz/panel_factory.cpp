/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz/displays_panel.h"
#include "rviz/help_panel.h"
#include "rviz/selection_panel.h"
#include "rviz/time_panel.h"
#include "rviz/tool_properties_panel.h"
#include "rviz/views_panel.h"

#include "rviz/panel_factory.h"

namespace rviz
{

static Panel* newDisplaysPanel()       { return new DisplaysPanel(); }
static Panel* newHelpPanel()           { return new HelpPanel(); }
static Panel* newSelectionPanel()      { return new SelectionPanel(); }
static Panel* newTimePanel()           { return new TimePanel(); }
static Panel* newToolPropertiesPanel() { return new ToolPropertiesPanel(); }
static Panel* newViewsPanel()          { return new ViewsPanel(); }

PanelFactory::PanelFactory()
  : PluginlibFactory<Panel>( "rviz", "rviz::Panel" )
{
  addBuiltInClass( "rviz", "Displays", "Show and edit the list of Displays", &newDisplaysPanel );
  addBuiltInClass( "rviz", "Help", "Show the key and mouse bindings", &newHelpPanel );
  addBuiltInClass( "rviz", "Selection", "Show properties of selected objects", &newSelectionPanel );
  addBuiltInClass( "rviz", "Time", "Show the current time", &newTimePanel );
  addBuiltInClass( "rviz", "Tool Properties", "Show and edit properties of tools", &newToolPropertiesPanel );
  addBuiltInClass( "rviz", "Views", "Show and edit viewpoints", &newViewsPanel );
}

} // end namespace rviz
