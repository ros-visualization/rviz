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
#ifndef DISPLAY_CONTEXT_H
#define DISPLAY_CONTEXT_H

#include <cstdint> // for uint64_t
#include <memory>

#include <QObject>
#include <QString>
#include "frame_manager.h"

class QKeyEvent;

namespace Ogre
{
class SceneManager;
}

namespace ros
{
class CallbackQueueInterface;
}

namespace tf
{
class TransformListener;
}

namespace tf2_ros
{
class Buffer;
}

namespace rviz
{
class BitAllocator;
class DisplayFactory;
class DisplayGroup;
class RenderPanel;
class SelectionManager;
class ToolManager;
class ViewController;
class ViewportMouseEvent;
class ViewManager;
class WindowManagerInterface;

/** @brief Pure-virtual base class for objects which give Display
 * subclasses context in which to work.
 *
 * This interface class mainly exists to enable more isolated unit
 * tests by enabling small mock objects to take the place of the large
 * VisualizationManager implementation class.  It also serves to
 * define a narrower, more maintainable API for Display plugins. */
class DisplayContext : public QObject
{
  Q_OBJECT
public:
  /** @brief Returns the Ogre::SceneManager used for the main RenderPanel. */
  virtual Ogre::SceneManager* getSceneManager() const = 0;

  /** @brief Return the window manager, if any. */
  virtual WindowManagerInterface* getWindowManager() const = 0;

  /** @brief Return a pointer to the SelectionManager. */
  virtual SelectionManager* getSelectionManager() const = 0;

  /** @brief Return the FrameManager instance. */
  virtual FrameManager* getFrameManager() const = 0;

  /** @brief Convenience function: returns getFrameManager()->getTF2BufferPtr(). */
  std::shared_ptr<tf2_ros::Buffer> getTF2BufferPtr() const
  {
    return getFrameManager()->getTF2BufferPtr();
  }

  /** @brief Return the fixed frame name. */
  virtual QString getFixedFrame() const = 0;

  /** @brief Return the current value of the frame count.
   *
   * The frame count is just a number which increments each time a
   * frame is rendered.  This lets clients check if a new frame has
   * been rendered since the last time they did something. */
  virtual uint64_t getFrameCount() const = 0;

  /** @brief Return a factory for creating Display subclasses based on a class id string. */
  virtual DisplayFactory* getDisplayFactory() const = 0;

  /** @brief Return the CallbackQueue using the main GUI thread. */
  virtual ros::CallbackQueueInterface* getUpdateQueue() = 0;

  /** @brief Return a CallbackQueue using a different thread than the main GUI one. */
  virtual ros::CallbackQueueInterface* getThreadedQueue() = 0;

  /** @brief Handle a single key event for a given RenderPanel. */
  virtual void handleChar(QKeyEvent* event, RenderPanel* panel) = 0;

  /** @brief Handle a mouse event. */
  virtual void handleMouseEvent(const ViewportMouseEvent& event) = 0;

  /** @brief Return the ToolManager. */
  virtual ToolManager* getToolManager() const = 0;

  /** @brief Return the ViewManager. */
  virtual ViewManager* getViewManager() const = 0;

  virtual DisplayGroup* getRootDisplayGroup() const = 0;

  virtual uint32_t getDefaultVisibilityBit() const = 0;

  virtual BitAllocator* visibilityBits() = 0;

  /** Set the message displayed in the status bar */
  virtual void setStatus(const QString& message) = 0;

public Q_SLOTS:
  /** @brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * @note This function can be called from any thread. */
  virtual void queueRender() = 0;
};

} // end namespace rviz

#endif // DISPLAY_CONTEXT_H
