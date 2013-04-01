/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef RENDER_SYSTEM_H
#define RENDER_SYSTEM_H

#include <OGRE/OgreRoot.h>
#include <stdint.h>

namespace rviz
{

class RenderSystem
{
public:
  static RenderSystem* get();

  Ogre::RenderWindow* makeRenderWindow( intptr_t window_id, unsigned int width, unsigned int height );

  Ogre::Root* root() { return ogre_root_; }

  // @brief return OpenGl Version as integer, e.g. 320 for OpenGl 3.20
  int getGlVersion() { return gl_version_; }

  // @brief return GLSL Version as integer, e.g. 150 for GLSL 1.50
  int getGlslVersion() { return glsl_version_; }

  // @brief Force to use the provided OpenGL version on startup
  static void forceGlVersion( int version );

private:
  RenderSystem();
  void setupDummyWindowId();
  void loadOgrePlugins();

  // Find and configure the render system.
  void setupRenderSystem();
  void setupResources();
  void detectGlVersion();

  static RenderSystem* instance_;

  // ID for a dummy window of size 1x1, used to keep Ogre happy.
  unsigned long dummy_window_id_;

  Ogre::Root* ogre_root_;

  int gl_version_;
  int glsl_version_;
  static int force_gl_version_;
};

} // end namespace rviz

#endif // RENDER_SYSTEM_H
