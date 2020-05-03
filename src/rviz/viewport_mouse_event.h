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

#ifndef VIEWPORT_MOUSE_EVENT_H
#define VIEWPORT_MOUSE_EVENT_H

#include <QMouseEvent>
#include <QWheelEvent>

namespace Ogre
{
class Viewport;
}

namespace rviz
{
class RenderPanel;

class ViewportMouseEvent
{
public:
  ViewportMouseEvent()
  {
  }

  /** Constructor for use with a QMouseEvent. */
  ViewportMouseEvent(RenderPanel* p, Ogre::Viewport* vp, QMouseEvent* e, int lx, int ly)
    : panel(p)
    , viewport(vp)
    , type(e->type())
    , x(e->x())
    , y(e->y())
    , wheel_delta(0)
    , acting_button(e->button())
    , buttons_down(e->buttons())
    , modifiers(e->modifiers())
    , last_x(lx)
    , last_y(ly)
  {
  }

  // Qt has a separate QWheelEvent for mousewheel events which is not
  // a subclass of QMouseEvent, but has a lot of overlap with it.

  /** Constructor for use with a QWheelEvent. */
  ViewportMouseEvent(RenderPanel* p, Ogre::Viewport* vp, QWheelEvent* e, int lx, int ly)
    : panel(p)
    , viewport(vp)
    , type(e->type())
    , x(e->x())
    , y(e->y())
    , wheel_delta(e->delta())
    , acting_button(Qt::NoButton)
    , buttons_down(e->buttons())
    , modifiers(e->modifiers())
    , last_x(lx)
    , last_y(ly)
  {
  }

  // Convenience functions for getting the state of the buttons and
  // modifiers at the time of the event.  For the button which caused
  // a press or release event, use acting_button.
  bool left()
  {
    return buttons_down & Qt::LeftButton;
  }
  bool middle()
  {
    return buttons_down & Qt::MidButton;
  }
  bool right()
  {
    return buttons_down & Qt::RightButton;
  }

  bool shift()
  {
    return modifiers & Qt::ShiftModifier;
  }
  bool control()
  {
    return modifiers & Qt::ControlModifier;
  }
  bool alt()
  {
    return modifiers & Qt::AltModifier;
  }

  // Convenience functions to tell if the event is a mouse-down or
  // mouse-up event and which button caused it.
  bool leftUp()
  {
    return type == QEvent::MouseButtonRelease && acting_button == Qt::LeftButton;
  }
  bool middleUp()
  {
    return type == QEvent::MouseButtonRelease && acting_button == Qt::MidButton;
  }
  bool rightUp()
  {
    return type == QEvent::MouseButtonRelease && acting_button == Qt::RightButton;
  }

  bool leftDown()
  {
    return type == QEvent::MouseButtonPress && acting_button == Qt::LeftButton;
  }
  bool middleDown()
  {
    return type == QEvent::MouseButtonPress && acting_button == Qt::MidButton;
  }
  bool rightDown()
  {
    return type == QEvent::MouseButtonPress && acting_button == Qt::RightButton;
  }

  RenderPanel* panel;
  Ogre::Viewport* viewport;
  QEvent::Type type;
  int x;
  int y;
  int wheel_delta;
  Qt::MouseButton
      acting_button; // The button which caused the event.  Can be Qt::NoButton (move or wheel events).
  Qt::MouseButtons buttons_down;
  Qt::KeyboardModifiers modifiers;
  int last_x;
  int last_y;
};

} // namespace rviz

#endif // VIEWPORT_MOUSE_EVENT_H
