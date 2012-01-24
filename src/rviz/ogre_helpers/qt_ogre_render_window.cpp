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
  assert( viewport_ );
  return viewport_;
}

void QtOgreRenderWindow::setCamera( Ogre::Camera* camera )
{
  camera_ = camera;
  if( viewport_ )
  {
    viewport_->setCamera( camera );
  }

  setCameraAspectRatio();

  update();
}

void QtOgreRenderWindow::setOverlaysEnabled( bool overlays_enabled )
{
  overlays_enabled_ = overlays_enabled;
  if( viewport_ )
  {
    viewport_->setOverlaysEnabled( overlays_enabled );
  }
}

void QtOgreRenderWindow::setBackgroundColor( Ogre::ColourValue background_color )
{
  background_color_ = background_color;
  if( viewport_ )
  {
    viewport_->setBackgroundColour( background_color );
  }
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
