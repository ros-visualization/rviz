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

#include "move_tool.h"
#include "visualization_manager.h"
#include "render_panel.h"
#include "viewport_mouse_event.h"

#include "ogre_tools/camera_base.h"

#include <wx/event.h>

namespace rviz
{

MoveTool::MoveTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: Tool( name, shortcut_key, manager )
{

}

int MoveTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  ogre_tools::CameraBase* camera = manager_->getCurrentCamera();

  if ( event.event.LeftDown() )
  {
    camera->mouseLeftDown( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.MiddleDown() )
  {
    camera->mouseMiddleDown( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.RightDown() )
  {
    camera->mouseRightDown( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.LeftUp() )
  {
    camera->mouseLeftUp( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.MiddleUp() )
  {
    camera->mouseMiddleUp( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.RightUp() )
  {
    camera->mouseRightUp( event.event.GetX(), event.event.GetY() );
    flags |= Render;
  }
  else if ( event.event.Dragging() )
  {
    int32_t diff_x = event.event.GetX() - event.last_x;
    int32_t diff_y = event.event.GetY() - event.last_y;

    if ( event.event.LeftIsDown() )
    {
      camera->mouseLeftDrag( diff_x, diff_y, event.event.CmdDown(), event.event.AltDown(), event.event.ShiftDown() );

      flags |= Render;
    }
    else if ( event.event.MiddleIsDown() )
    {
      camera->mouseMiddleDrag( diff_x, diff_y, event.event.CmdDown(), event.event.AltDown(), event.event.ShiftDown() );

      flags |= Render;
    }
    else if ( event.event.RightIsDown() )
    {
      camera->mouseRightDrag( diff_x, diff_y, event.event.CmdDown(), event.event.AltDown(), event.event.ShiftDown() );

      flags |= Render;
    }
  }

  if ( event.event.GetWheelRotation() != 0 )
  {
    camera->scrollWheel( event.event.GetWheelRotation(), event.event.CmdDown(), event.event.AltDown(), event.event.ShiftDown() );

    flags |= Render;
  }

  return flags;
}

}

