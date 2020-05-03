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

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <rviz/render_panel.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>

#include <rviz/default_plugin/tools/interaction_tool.h>

namespace rviz
{
InteractionTool::InteractionTool()
{
  shortcut_key_ = 'i';
  hide_inactive_property_ =
      new BoolProperty("Hide Inactive Objects", true,
                       "While holding down a mouse button, hide all other Interactive Objects.",
                       getPropertyContainer(), SLOT(hideInactivePropertyChanged()), this);
}

InteractionTool::~InteractionTool()
{
}

void InteractionTool::onInitialize()
{
  move_tool_.initialize(context_);
  last_selection_frame_count_ = context_->getFrameCount();
  deactivate();
}

void InteractionTool::activate()
{
  context_->getSelectionManager()->enableInteraction(true);
  context_->getSelectionManager()->setTextureSize(1);
}

void InteractionTool::deactivate()
{
  context_->getSelectionManager()->enableInteraction(false);
}

void InteractionTool::updateFocus(const ViewportMouseEvent& event)
{
  M_Picked results;
  // Pick exactly 1 pixel
  context_->getSelectionManager()->pick(event.viewport, event.x, event.y, event.x + 1, event.y + 1,
                                        results, true);

  last_selection_frame_count_ = context_->getFrameCount();

  InteractiveObjectPtr new_focused_object;

  // look for a valid handle in the result.
  M_Picked::iterator result_it = results.begin();
  if (result_it != results.end())
  {
    Picked pick = result_it->second;
    SelectionHandler* handler = context_->getSelectionManager()->getHandler(pick.handle);
    if (pick.pixel_count > 0 && handler)
    {
      InteractiveObjectPtr object = handler->getInteractiveObject().lock();
      if (object && object->isInteractive())
      {
        new_focused_object = object;
      }
    }
  }

  // If the mouse has gone from one object to another, defocus the old
  // and focus the new.
  InteractiveObjectPtr new_obj = new_focused_object;
  InteractiveObjectPtr old_obj = focused_object_.lock();
  if (new_obj != old_obj)
  {
    // Only copy the event contents here, once we know we need to use
    // a modified version of it.
    ViewportMouseEvent event_copy = event;
    if (old_obj)
    {
      event_copy.type = QEvent::FocusOut;
      old_obj->handleMouseEvent(event_copy);
    }

    if (new_obj)
    {
      event_copy.type = QEvent::FocusIn;
      new_obj->handleMouseEvent(event_copy);
    }
  }

  focused_object_ = new_focused_object;
}

int InteractionTool::processMouseEvent(ViewportMouseEvent& event)
{
  int flags = 0;

  if (event.panel->contextMenuVisible())
  {
    return flags;
  }

  // make sure we let the vis. manager render at least one frame between selection updates
  bool need_selection_update = context_->getFrameCount() > last_selection_frame_count_;

  // We are dragging if a button was down and is still down
  Qt::MouseButtons buttons = event.buttons_down & (Qt::LeftButton | Qt::RightButton | Qt::MidButton);
  if (event.type == QEvent::MouseButtonPress)
    buttons &= ~event.acting_button;
  bool dragging = buttons != 0;

  // unless we're dragging, check if there's a new object under the mouse
  if (need_selection_update && !dragging && event.type != QEvent::MouseButtonRelease)
  {
    updateFocus(event);
    flags = Render;
  }

  {
    InteractiveObjectPtr focused_object = focused_object_.lock();
    if (focused_object)
    {
      focused_object->handleMouseEvent(event);
      setCursor(focused_object->getCursor());
      // this will disable everything but the current interactive object
      if (hide_inactive_property_->getBool())
      {
        context_->getSelectionManager()->enableInteraction(!dragging);
      }
    }
    else if (event.panel->getViewController())
    {
      move_tool_.processMouseEvent(event);
      setCursor(move_tool_.getCursor());
      if (hide_inactive_property_->getBool())
      {
        context_->getSelectionManager()->enableInteraction(true);
      }
    }
  }

  if (event.type == QEvent::MouseButtonRelease)
  {
    updateFocus(event);
  }

  return flags;
}

int InteractionTool::processKeyEvent(QKeyEvent* event, RenderPanel* panel)
{
  return move_tool_.processKeyEvent(event, panel);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::InteractionTool, rviz::Tool)
