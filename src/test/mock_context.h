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
#ifndef MOCK_CONTEXT_H
#define MOCK_CONTEXT_H

#include <rviz/display_context.h>

namespace rviz
{

class MockContext: public DisplayContext
{
public:
  MockContext();

  virtual Ogre::SceneManager* getSceneManager() const { return 0; }
  virtual WindowManagerInterface* getWindowManager() const { return 0; }
  virtual SelectionManager* getSelectionManager() const { return 0; }
  virtual FrameManager* getFrameManager() const { return 0; }
  virtual tf::TransformListener* getTFClient() const { return 0; }
  virtual QString getFixedFrame() const { return ""; }
  virtual uint64_t getFrameCount() const { return 0; }
  virtual DisplayFactory* getDisplayFactory() const { return display_factory_; }
  virtual ros::CallbackQueueInterface* getUpdateQueue() { return 0; }
  virtual ros::CallbackQueueInterface* getThreadedQueue() { return 0; }
  virtual void handleChar( QKeyEvent* event, RenderPanel* panel ) {}
  virtual void handleMouseEvent( const ViewportMouseEvent& event ) {}
  virtual ToolManager* getToolManager() const { return 0; }
  virtual ViewManager* getViewManager() const { return 0; }
  virtual DisplayGroup* getRootDisplayGroup() const { return 0; }
  virtual uint32_t getDefaultVisibilityBit() const { return 0; }
  virtual BitAllocator* visibilityBits() { return 0; }
  virtual void setStatus( const QString & message ) {}

  virtual void queueRender() {}

private:
  DisplayFactory* display_factory_;
};

} // end namespace rviz

#endif // MOCK_CONTEXT_H
