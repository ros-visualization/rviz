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

#include "selection_tool.h"
#include "move_tool.h"
#include "selection/selection_manager.h"
#include "visualization_manager.h"
#include "render_panel.h"
#include "display.h"
#include "viewport_mouse_event.h"

#include "ogre_tools/camera_base.h"
#include "ogre_tools/wx_ogre_render_window.h"

#include <wx/event.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTextureManager.h>

#include <btBulletCollisionCommon.h>

#include <ros/time.h>

namespace rviz
{

SelectionTool::SelectionTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: Tool( name, shortcut_key, manager )
, move_tool_(new MoveTool("SelectionTool Fake MoveTool", 0, manager))
, selecting_(false)
, sel_start_x_(0)
, sel_start_y_(0)
, moving_(false)
{

}

SelectionTool::~SelectionTool()
{
  delete move_tool_;
}

void SelectionTool::activate()
{
  selecting_ = false;
  moving_ = false;
}

void SelectionTool::deactivate()
{
  manager_->getSelectionManager()->removeHighlight();
}

void SelectionTool::update(float wall_dt, float ros_dt)
{
  SelectionManager* sel_manager = manager_->getSelectionManager();

  if (!selecting_)
  {
    sel_manager->removeHighlight();
  }
}

int SelectionTool::processMouseEvent( ViewportMouseEvent& event )
{
  SelectionManager* sel_manager = manager_->getSelectionManager();

  event.viewport->setMaterialScheme("test");

  int flags = 0;

  if (event.event.AltDown())
  {
    moving_ = true;
    selecting_ = false;
  }
  else
  {
    moving_ = false;

    if (event.event.LeftDown())
    {
      selecting_ = true;

      sel_start_x_ = event.event.GetX();
      sel_start_y_ = event.event.GetY();
    }
  }

  if (selecting_)
  {
    sel_manager->highlight(event.viewport, sel_start_x_, sel_start_y_, event.event.GetX(), event.event.GetY());

    if (event.event.LeftUp())
    {
      SelectionManager::SelectType type = SelectionManager::Replace;

      M_Picked selection;

      if (event.event.ShiftDown())
      {
        type = SelectionManager::Add;
      }
      else if (event.event.ControlDown())
      {
        type = SelectionManager::Remove;
      }

      sel_manager->select(event.viewport, sel_start_x_, sel_start_y_, event.event.GetX(), event.event.GetY(), type);

      selecting_ = false;
    }

    flags |= Render;
  }
  else if (moving_)
  {
    sel_manager->removeHighlight();

    flags = move_tool_->processMouseEvent(event);

    if (event.event.LeftUp() || event.event.RightUp() || event.event.MiddleUp())
    {
      moving_ = false;
    }
  }
  else
  {
    sel_manager->highlight(event.viewport, event.event.GetX(), event.event.GetY(), event.event.GetX(), event.event.GetY());
  }

  return flags;
}

int SelectionTool::processKeyEvent( wxKeyEvent& event )
{
  SelectionManager* sel_manager = manager_->getSelectionManager();
  char key = event.GetKeyCode();

  switch (key)
  {
  case 'f':
  case 'F':
    sel_manager->focusOnSelection();
    break;
  }

  return Render;
}

}

