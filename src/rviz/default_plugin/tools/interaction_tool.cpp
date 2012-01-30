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

// helper class, which acts as SelectionHandler and forwards all mouse events to the view controller
class ViewControllerHandler: public SelectionHandler
{
  virtual bool isInteractive() { return true; }
  virtual void handleMouseEvent(const Picked& obj,  ViewportMouseEvent& event)
  {
    if ( event.panel->getViewController() )
    {
      event.panel->getViewController()->handleMouseEvent( event );
    }
  }
};


InteractionTool::InteractionTool()
  : MoveTool()
  , focused_object_(0)
  , view_controller_handle_( 0 )
  , view_controller_handler_( new ViewControllerHandler() )
{
  name_ = "Interact";
  shortcut_key_ = 'i';
}

InteractionTool::~InteractionTool()
{
  if( view_controller_handle_ )
  {
    manager_->getSelectionManager()->removeObject( view_controller_handle_ );
  }
}

void InteractionTool::onInitialize()
{
  last_selection_frame_count_ = manager_->getFrameCount();
  view_controller_handle_ = manager_->getSelectionManager()->createHandle();
  manager_->getSelectionManager()->addObject(view_controller_handle_, view_controller_handler_ );

  deactivate();
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

void InteractionTool::updateSelection( SelectionHandlerPtr &focused_handler, ViewportMouseEvent event )
{
  M_Picked results;
  // Pick exactly 1 pixel
  manager_->getSelectionManager()->pick( event.viewport,
                                         event.x, event.y,
                                         event.x + 1, event.y + 1,
                                         results, true );

  last_selection_frame_count_ = manager_->getFrameCount();

  SelectionHandlerPtr new_focused_handler;
  Picked new_focused_object;
  new_focused_object.pixel_count = 0;

  // look for a valid handle in the result.
  M_Picked::iterator result_it = results.begin();
  if( result_it != results.end() )
  {
    Picked object = result_it->second;
    SelectionHandlerPtr handler = manager_->getSelectionManager()->getHandler( object.handle );
    if ( object.pixel_count > 0 && handler.get() && handler->isInteractive() )
    {
      new_focused_object = object;
      new_focused_handler = handler;
    }
  }

  // switch to view controller handler if nothing else is there
  if ( !new_focused_handler.get() )
  {
    new_focused_handler = view_controller_handler_;
    new_focused_object.handle = view_controller_handle_;
  }

  // if the mouse has gone from one object to another,
  // pass on focus
  if ( new_focused_handler.get() != focused_handler.get() )
  {
    if ( focused_handler.get() )
    {
      event.type = QEvent::FocusOut;
      focused_handler->handleMouseEvent( focused_object_, event );
    }

    ROS_DEBUG( "Switch focus to %d", new_focused_object.handle );
    event.type = QEvent::FocusIn;
    new_focused_handler->handleMouseEvent( focused_object_, event );
  }

  focused_handler = new_focused_handler;
  focused_object_ = new_focused_object;
}

int InteractionTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;
  // get the handler which was active last time
  SelectionHandlerPtr focused_handler;
  focused_handler = manager_->getSelectionManager()->getHandler( focused_object_.handle );

  // make sure we let the vis. manager render at least one frame between selection updates
  bool need_selection_update = manager_->getFrameCount() > last_selection_frame_count_;

  // unless we're dragging, check if there's a new object under the mouse
  if( need_selection_update &&
      !(event.type == QEvent::MouseMove && event.buttons_down != Qt::NoButton) &&
      event.type != QEvent::MouseButtonRelease )
  {
    updateSelection( focused_handler, event );
    flags = Render;
  }

  if( focused_handler.get() )
  {
    focused_handler->handleMouseEvent( focused_object_, event );
  }

  if( event.type == QEvent::MouseButtonRelease )
  {
    updateSelection( focused_handler, event );
  }

  return flags;
}

}

