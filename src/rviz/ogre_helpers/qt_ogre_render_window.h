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

#include <boost/function.hpp>

#include "render_widget.h"

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
class QtOgreRenderWindow : public Ogre::RenderTargetListener {
public:

  /**
   * Set a callback which is called before each render
   * @param func The callback functor
   */
  virtual void setPreRenderCallback( boost::function<void ()> func ) = 0;
  /**
     * Set a callback which is called after each render
     * @param func The callback functor
     */
  virtual void setPostRenderCallback( boost::function<void ()> func ) = 0;

  /** Gets the associated Ogre viewport.  If this is called before
   * QWidget::show() on this widget, it will fail an assertion.
   * Several functions of Ogre::Viewport are duplicated in this class
   * which can be called before QWidget::show(), and their effects are
   * propagated to the viewport when it is created.
   */
  virtual Ogre::Viewport* getViewport() const = 0;

  virtual Ogre::RenderWindow* getRenderWindow() = 0;

  /** Set the camera associated with this render window's viewport.
   */
  virtual void setCamera( Ogre::Camera* camera ) = 0;

  virtual Ogre::Camera* getCamera() const = 0;

  /**
   * \brief Set the scale of the orthographic window.  Only valid for an orthographic camera.
   * @param scale The scale
   */
  virtual void setOrthoScale( float scale ) = 0;

  /** \brief Enable or disable stereo rendering
   * If stereo is not supported this is ignored.
   * @return the old setting (whether stereo was enabled before)
   */
  virtual bool enableStereo(bool enable) = 0;

  /** \brief Prepare to render in stereo if enabled and supported. */
  virtual void setupStereo() = 0;

  virtual void setAutoRender(bool auto_render) = 0;

  ////// Functions mimicked from Ogre::Viewport to satisfy timing of
  ////// after-constructor creation of Ogre::RenderWindow.
  virtual void setOverlaysEnabled( bool overlays_enabled ) = 0;
  virtual void setBackgroundColor( Ogre::ColourValue color ) = 0;
};

} // namespace rviz

#endif // QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
