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
#ifndef QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
#define QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_

#include "ogre_viewport_support.h"

#include <QCursor>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QEvent>

#include <OgreColourValue.h>
#include <OgreRenderTargetListener.h>

namespace Ogre
{
class Root;
class RenderWindow;
class Viewport;
class Camera;
}

namespace rviz
{

/**
 * \brief Generic interface for Qt Ogre render windows
 * Qt Ogre render window widget.  Similar in API to
 *  wxOgreRenderWindow from ogre_tools release 1.6, but with much of
 *  the guts replaced by new RenderSystem and RenderWidget classes
 *  inspired by the initialization sequence of Gazebo's renderer.
 */
class QtOgreRenderWindow: public OgreViewportSupport {
public:
  explicit QtOgreRenderWindow();

  virtual ~QtOgreRenderWindow();

  virtual void setFocus(Qt::FocusReason reason) = 0;
  virtual QPoint mapFromGlobal(const QPoint &) const = 0;
  virtual QPoint mapToGlobal(const QPoint &) const = 0;
  virtual void setCursor(const QCursor &) = 0;

  void setKeyPressEventCallback(std::function<void (QKeyEvent*)> function) {
    key_press_event_callback_ = function;
  }

  void setWheelEventCallback(std::function<void (QWheelEvent*)> function) {
    wheel_event_callback_ = function;
  }

  void setLeaveEventCallack(std::function<void (QEvent*)> function) {
    leave_event_callback_ = function;
  }

protected:
  void emitKeyPressEvent(QKeyEvent* event) {
    key_press_event_callback_(event);
  }

  void emitWheelEvent(QWheelEvent* event) {
    wheel_event_callback_(event);
  }

  void emitLeaveEvent(QEvent* event) {
    leave_event_callback_(event);
  }

private:
  std::function<void (QKeyEvent*)> key_press_event_callback_;
  std::function<void (QWheelEvent*)> wheel_event_callback_;
  std::function<void (QEvent*)> leave_event_callback_;
};

} // namespace rviz

#endif // QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
