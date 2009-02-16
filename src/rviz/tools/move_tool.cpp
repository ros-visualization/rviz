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
#include "ogre_tools/camera_base.h"

#include <wx/wx.h>

namespace rviz
{

MoveTool::MoveTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: Tool( name, shortcut_key, manager )
{

}

int MoveTool::processMouseEvent( wxMouseEvent& event, int last_x, int last_y )
{
  int flags = 0;

  ogre_tools::CameraBase* camera = manager_->getRenderPanel()->getCurrentCamera();

  if ( event.LeftDown() )
  {
    camera->mouseLeftDown( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.MiddleDown() )
  {
    camera->mouseMiddleDown( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.RightDown() )
  {
    camera->mouseRightDown( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.LeftUp() )
  {
    camera->mouseLeftUp( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.MiddleUp() )
  {
    camera->mouseMiddleUp( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.RightUp() )
  {
    camera->mouseRightUp( event.GetX(), event.GetY() );
    flags |= Render;
  }
  else if ( event.Dragging() )
  {
    int32_t diff_x = event.GetX() - last_x;
    int32_t diff_y = event.GetY() - last_y;

    if ( event.LeftIsDown() )
    {
      camera->mouseLeftDrag( diff_x, diff_y );

      flags |= Render;
    }
    else if ( event.MiddleIsDown() )
    {
      camera->mouseMiddleDrag( diff_x, diff_y );

      flags |= Render;
    }
    else if ( event.RightIsDown() )
    {
      camera->mouseRightDrag( diff_x, diff_y );

      flags |= Render;
    }
  }

  if ( event.GetWheelRotation() != 0 )
  {
    camera->scrollWheel( event.GetWheelRotation() );

    flags |= Render;
  }

  return flags;
}

}

