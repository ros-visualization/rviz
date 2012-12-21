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
#include "orthographic.h"
#include "render_system.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreStringConverter.h>
#include <OGRE/OgreGpuProgramManager.h>

#include <ros/console.h>
#include <ros/assert.h>

#if OGRE_PLATFORM == OGRE_PLATFORM_LINUX
#include <stdlib.h>
#endif

namespace rviz
{

QtOgreRenderWindow::QtOgreRenderWindow( QWidget* parent )
  : RenderWidget( RenderSystem::get(), parent )
  , viewport_( 0 )
  , ogre_root_( RenderSystem::get()->root() )
  , ortho_scale_( 1.0f )
  , auto_render_( true )
  , camera_( 0 )
  , overlays_enabled_( true ) // matches the default of Ogre::Viewport.
  , background_color_( Ogre::ColourValue::Black ) // matches the default of Ogre::Viewport.
{
  render_window_->setVisible(true);
  render_window_->setAutoUpdated(true);

  viewport_ = render_window_->addViewport( camera_ );
  viewport_->setOverlaysEnabled( overlays_enabled_ );
  viewport_->setBackgroundColour( background_color_ );

  setCameraAspectRatio();
}

//------------------------------------------------------------------------------
Ogre::Viewport* QtOgreRenderWindow::getViewport () const
{
  return viewport_;
}

void QtOgreRenderWindow::setCamera( Ogre::Camera* camera )
{
  camera_ = camera;
  viewport_->setCamera( camera );

  setCameraAspectRatio();

  update();
}

void QtOgreRenderWindow::setOverlaysEnabled( bool overlays_enabled )
{
  overlays_enabled_ = overlays_enabled;
  viewport_->setOverlaysEnabled( overlays_enabled );
}

void QtOgreRenderWindow::setBackgroundColor( Ogre::ColourValue background_color )
{
  background_color_ = background_color;
  viewport_->setBackgroundColour( background_color );
}

void QtOgreRenderWindow::setCameraAspectRatio()
{
  if ( camera_ )
  {
    camera_->setAspectRatio( Ogre::Real( width() ) / Ogre::Real( height() ) );

    if ( camera_->getProjectionType() == Ogre::PT_ORTHOGRAPHIC )
    {
      Ogre::Matrix4 proj;
      buildScaledOrthoMatrix( proj,
                              -width() / ortho_scale_ / 2, width() / ortho_scale_ / 2,
                              -height() / ortho_scale_ / 2, height() / ortho_scale_ / 2,
                              camera_->getNearClipDistance(), camera_->getFarClipDistance() );
      camera_->setCustomProjectionMatrix(true, proj);
    }
  }
}

void QtOgreRenderWindow::setOrthoScale( float scale )
{
  ortho_scale_ = scale;

  setCameraAspectRatio();
}

void QtOgreRenderWindow::setPreRenderCallback( boost::function<void ()> func )
{
  pre_render_callback_ = func;
}

void QtOgreRenderWindow::setPostRenderCallback( boost::function<void ()> func )
{
  post_render_callback_ = func;
}

//------------------------------------------------------------------------------
void QtOgreRenderWindow::paintEvent( QPaintEvent* e )
{
  if( auto_render_ && render_window_ )
  {
    if( pre_render_callback_ )
    {
      pre_render_callback_();
    }

    if( ogre_root_->_fireFrameStarted() )
    {
#if (OGRE_VERSION_MAJOR >= 1 && OGRE_VERSION_MINOR >= 6)
      ogre_root_->_fireFrameRenderingQueued();
#endif

      render_window_->update();

      ogre_root_->_fireFrameEnded();
    }

    if ( post_render_callback_ )
    {
      post_render_callback_();
    }
  }
}

//------------------------------------------------------------------------------
void QtOgreRenderWindow::resizeEvent( QResizeEvent* event )
{
  RenderWidget::resizeEvent( event );

  if( render_window_ )
  {
    setCameraAspectRatio();

    if( auto_render_ )
    {
      update();
    }
  }
}

} // namespace rviz
