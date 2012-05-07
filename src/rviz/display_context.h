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

#include <stdint.h> // for uint64_t

#include <QString>

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

namespace rviz
{

class WindowManagerInterface;
class SelectionManager;
class FrameManager;
class DisplayFactory;

/** @brief Pure-virtual base class for objects which give Display
 * subclasses context in which to work. */
class DisplayContext
{
public:
  virtual Ogre::SceneManager* getSceneManager() const = 0;
  virtual WindowManagerInterface* getWindowManager() const = 0;
  virtual SelectionManager* getSelectionManager() const = 0;
  virtual FrameManager* getFrameManager() const = 0;
  virtual tf::TransformListener* getTFClient() const = 0;
  virtual void queueRender() = 0;
  virtual QString getFixedFrame() const = 0;
  virtual uint64_t getFrameCount() const = 0;
  virtual DisplayFactory* getDisplayFactory() const = 0;
  virtual ros::CallbackQueueInterface* getUpdateQueue() = 0;
  virtual ros::CallbackQueueInterface* getThreadedQueue() = 0;
};

} // end namespace rviz

#endif // DISPLAY_CONTEXT_H
