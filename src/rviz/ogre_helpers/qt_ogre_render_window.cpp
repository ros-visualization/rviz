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
#include "qt_ogre_render_window.h"

namespace rviz {

QtOgreRenderWindow::QtOgreRenderWindow()
{

}

QtOgreRenderWindow::~QtOgreRenderWindow() {}

void QtOgreRenderWindow::setKeyPressEventCallback(const std::function<void (QKeyEvent *)> &function) {
  key_press_event_callback_ = function;
}

void QtOgreRenderWindow::setWheelEventCallback(const std::function<void (QWheelEvent *)> &function) {
  wheel_event_callback_ = function;
}

void QtOgreRenderWindow::setLeaveEventCallack(const std::function<void (QEvent *)> &function) {
  leave_event_callback_ = function;
}

void QtOgreRenderWindow::setMouseMoveEventCallback(const std::function<void (QMouseEvent *)> &function) {
  mouse_move_event_callback_ = function;
}

void QtOgreRenderWindow::setMousePressEventCallback(const std::function<void (QMouseEvent *)> &mouse_press_event_callback)
{
  mouse_press_event_callback_ = mouse_press_event_callback;
}

void QtOgreRenderWindow::setMouseReleaseEventCallback(const std::function<void (QMouseEvent *)> &mouse_release_event_callback)
{
  mouse_release_event_callback_ = mouse_release_event_callback;
}

void QtOgreRenderWindow::setMouseDoubleClickEventCallback(const std::function<void (QMouseEvent *)> &mouse_double_click_event_callback)
{
  mouse_double_click_event_callback_ = mouse_double_click_event_callback;
}

void QtOgreRenderWindow::emitKeyPressEvent(QKeyEvent *event) {
  if (key_press_event_callback_) {
    key_press_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitWheelEvent(QWheelEvent *event) {
  if (wheel_event_callback_) {
    wheel_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitLeaveEvent(QEvent *event) {
  if (leave_event_callback_) {
    leave_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitMouseMoveEvent(QMouseEvent *event) {
  if (mouse_move_event_callback_) {
    mouse_move_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitMousePressEvent(QMouseEvent *event) {
  if (mouse_press_event_callback_) {
    mouse_press_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitMouseReleaseEvent(QMouseEvent *event) {
  if (mouse_release_event_callback_) {
    mouse_release_event_callback_(event);
  }
}

void QtOgreRenderWindow::emitMouseDoubleClickEvent(QMouseEvent *event) {
  if (mouse_double_click_event_callback_) {
    mouse_double_click_event_callback_(event);
  }
}

} // namespace rviz
