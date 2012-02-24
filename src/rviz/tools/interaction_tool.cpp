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

#include "interaction_tool.h"

#include "rviz/visualization_manager.h"
#include "rviz/properties/property_manager.h"
#include "rviz/properties/property.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/render_panel.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/selection/selection_handler.h"
#include "rviz/selection/selection_manager.h"

#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

namespace rviz
{

InteractionTool::InteractionTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
  : MoveTool( name, shortcut_key, manager )
  , last_selection_frame_count_(manager->getFrameCount())
{
  deactivate();
}

InteractionTool::~InteractionTool()
{
}

void InteractionTool::activate()
{
  manager_->getSelectionManager()->enableInteraction(true);
  manager_->getSelectionManager()->setTextureSize(2);
}

void InteractionTool::deactivate()
{
  manager_->getSelectionManager()->enableInteraction(false);
}

void InteractionTool::update(float wall_dt, float ros_dt)
{
}

void InteractionTool::updateFocus( const ViewportMouseEvent& event )
{
  M_Picked results;
  // Pick exactly 1 pixel
  manager_->getSelectionManager()->pick( event.viewport,
                                         event.x, event.y,
                                         event.x + 1, event.y + 1,
                                         results, true );

  last_selection_frame_count_ = manager_->getFrameCount();

  InteractiveObjectPtr new_focused_object;

  // look for a valid handle in the result.
  M_Picked::iterator result_it = results.begin();
  if( result_it != results.end() )
  {
    Picked pick = result_it->second;
    SelectionHandlerPtr handler = manager_->getSelectionManager()->getHandler( pick.handle );
    if ( pick.pixel_count > 0 && handler.get() )
    {
      InteractiveObjectPtr object = handler->getInteractiveObject().lock();
      if( object && object->isInteractive() )
      {
        new_focused_object = object;
      }
    }
  }

  // If the mouse has gone from one object to another, defocus the old
  // and focus the new.
  InteractiveObjectPtr new_obj = new_focused_object;
  InteractiveObjectPtr old_obj = focused_object_.lock();
  if( new_obj != old_obj )
  {
    // Only copy the event contents here, once we know we need to use
    // a modified version of it.
    ViewportMouseEvent event_copy = event;
    if( old_obj )
    {
      event_copy.type = QEvent::FocusOut;
      old_obj->handleMouseEvent( event_copy );
    }

    if( new_obj )
    {
      event_copy.type = QEvent::FocusIn;
      new_obj->handleMouseEvent( event_copy );
    }
  }

  focused_object_ = new_focused_object;
}

int InteractionTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  // make sure we let the vis. manager render at least one frame between selection updates
  bool need_selection_update = manager_->getFrameCount() > last_selection_frame_count_;
  bool dragging = (event.type == QEvent::MouseMove && event.buttons_down != Qt::NoButton);

  // unless we're dragging, check if there's a new object under the mouse
  if( need_selection_update &&
      !dragging &&
      event.type != QEvent::MouseButtonRelease )
  {
    updateFocus( event );
    flags = Render;
  }

  {
    InteractiveObjectPtr focused_object = focused_object_.lock();
    if( focused_object )
    {
      focused_object->handleMouseEvent( event );
    }
    else if( event.panel->getViewController() )
    {
      event.panel->getViewController()->handleMouseEvent( event );
    }
  }

  if( event.type == QEvent::MouseButtonRelease )
  {
    updateFocus( event );
  }

  return flags;
}

}

