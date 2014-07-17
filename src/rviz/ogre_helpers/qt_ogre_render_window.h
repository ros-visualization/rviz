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

/** Qt Ogre render window widget.  Similar in API to
 *  wxOgreRenderWindow from ogre_tools release 1.6, but with much of
 *  the guts replaced by new RenderSystem and RenderWidget classes
 *  inspired by the initialization sequence of Gazebo's renderer.
 */
class QtOgreRenderWindow : public RenderWidget, public Ogre::RenderTargetListener {
public:
  /** Constructor.
  	@param parent The parent wxWindow component.
   */
  QtOgreRenderWindow( QWidget* parent = 0 );

  /** Destructor.  */
  virtual ~QtOgreRenderWindow();

  /**
   * Set a callback which is called before each render
   * @param func The callback functor
   */
  virtual void setPreRenderCallback( boost::function<void ()> func );
  /**
     * Set a callback which is called after each render
     * @param func The callback functor
     */
  virtual void setPostRenderCallback( boost::function<void ()> func );

  /** Overrides the default implementation.
  	This override is here for convenience. Returns a symbolic 320x240px size.
  	@return A size of 320x240 (just a symbolic 4:3 size).
   */
  virtual QSize sizeHint () const { return QSize( 320, 240 ); }

  /** Gets the associated Ogre viewport.  If this is called before
   * QWidget::show() on this widget, it will fail an assertion.
   * Several functions of Ogre::Viewport are duplicated in this class
   * which can be called before QWidget::show(), and their effects are
   * propagated to the viewport when it is created.
   */
  Ogre::Viewport* getViewport() const;

  /** Set the camera associated with this render window's viewport.
   */
  void setCamera( Ogre::Camera* camera );

  Ogre::Camera* getCamera() const { return camera_; }

  /**
   * \brief Set the scale of the orthographic window.  Only valid for an orthographic camera.
   * @param scale The scale
   */
  void setOrthoScale( float scale );

  /** \brief Enable or disable stereo rendering
   * If stereo is not supported this is ignored.
   * @return the old setting (whether stereo was enabled before)
   */
  bool enableStereo(bool enable);

  /** \brief Prepare to render in stereo if enabled and supported. */
  void setupStereo();

  void setAutoRender(bool auto_render) { auto_render_ = auto_render; }

  ////// Functions mimicked from Ogre::Viewport to satisfy timing of
  ////// after-constructor creation of Ogre::RenderWindow.
  void setOverlaysEnabled( bool overlays_enabled );
  void setBackgroundColor( Ogre::ColourValue color );

protected:
  virtual void paintEvent( QPaintEvent* e );
  virtual void resizeEvent( QResizeEvent* event );

  // When stereo is enabled, these are called before/after rendering each
  // viewport.
  virtual void preViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);
  virtual void postViewportUpdate(const Ogre::RenderTargetViewportEvent& evt);

  /**
   * Sets the aspect ratio on the camera
   */
  void setCameraAspectRatio();

  /**
   * prepare a viewport's camera for stereo rendering.
   * This should only be called from StereoRenderTargetListener
   */
  void prepareStereoViewport(Ogre::Viewport*);


  Ogre::Viewport* viewport_;

  Ogre::Root* ogre_root_;

  boost::function<void ()> pre_render_callback_;      ///< Functor which is called before each render
  boost::function<void ()> post_render_callback_;     ///< Functor which is called after each render

  float ortho_scale_;
  bool auto_render_;

  Ogre::Camera* camera_;
  bool overlays_enabled_;
  Ogre::ColourValue background_color_;

  // stereo rendering
  bool stereo_enabled_;				// true if we were asked to render stereo
  bool rendering_stereo_;			// true if we are actually rendering stereo
  Ogre::Camera* left_camera_;
  Ogre::Camera* right_camera_;
  Ogre::Viewport* right_viewport_;
};

} // namespace rviz

#endif // QT_OGRE_RENDER_WINDOW_OGRE_RENDER_WINDOW_H_
