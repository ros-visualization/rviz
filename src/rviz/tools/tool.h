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

#ifndef RVIZ_TOOL_H
#define RVIZ_TOOL_H

#include <string>

class wxMouseEvent;
class wxKeyEvent;

namespace Ogre
{
class SceneManager;
}

namespace rviz
{

class VisualizationManager;
class ViewportMouseEvent;

class Tool
{
public:
  Tool( const std::string& name, char shortcut_key, VisualizationManager* manager );
  virtual ~Tool() {}

  const std::string& getName() { return name_; }
  char getShortcutKey() { return shortcut_key_; }

  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void update(float wall_dt, float ros_dt) {}

  enum Flags
  {
    Render = 1 << 0,
    Finished = 1 << 1
  };
  virtual int processMouseEvent( ViewportMouseEvent& event ) = 0;
  virtual int processKeyEvent( wxKeyEvent& event ) { return 0; }
protected:
  Ogre::SceneManager* scene_manager_;
  VisualizationManager* manager_;

  std::string name_;
  char shortcut_key_;
};

}

#endif
