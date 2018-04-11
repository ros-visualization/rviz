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

#include "rviz/display_context.h"
#include "rviz/render_panel.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "rviz/load_resource.h"

#include "rviz/default_plugin/tools/move_tool.h"

namespace rviz
{

MoveTool::MoveTool()
{
  shortcut_key_ = 'm';
  // this is needed as the move tool is instantiated by other tools
  setIcon( loadPixmap("package://rviz/icons/classes/MoveCamera.png") );
}

int MoveTool::processMouseEvent( ViewportMouseEvent& event )
{
  if (event.panel->getViewController())
  {
    event.panel->getViewController()->handleMouseEvent(event);
    setCursor( event.panel->getViewController()->getCursor() );
  }
  return 0;
}

int MoveTool::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
{
  if( context_->getViewManager()->getCurrent() )
  {
    context_->getViewManager()->getCurrent()->handleKeyEvent( event, panel );
  }
  return Render;
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::MoveTool, rviz::Tool )
