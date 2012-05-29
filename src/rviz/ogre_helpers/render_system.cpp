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

// This is required for QT_MAC_USE_COCOA to be set
#include <QtCore/qglobal.h>

#ifndef Q_OS_MAC
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>
#endif

#include <ros/package.h> // This dependency should be moved out of here, it is just used for a search path.
#include <ros/console.h>

#include <OGRE/OgreRenderWindow.h>

#include "render_system.h"

namespace rviz
{

RenderSystem* RenderSystem::instance_ = 0;

RenderSystem* RenderSystem::get()
{
  if( instance_ == 0 )
  {
    instance_ = new RenderSystem();
  }
  return instance_;
}

RenderSystem::RenderSystem()
{
  setupDummyWindowId();
  ogre_root_ = new Ogre::Root();
  loadOgrePlugins();
  setupRenderSystem();
  ogre_root_->initialise(false);
  setupResources();
  makeRenderWindow( dummy_window_id_, 1, 1 );
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void RenderSystem::setupDummyWindowId()
{
#ifdef Q_OS_MAC
  dummy_window_id_ = 0;
#else
  Display *display = XOpenDisplay(0);
  assert( display );

  int screen = DefaultScreen( display );

  int attribList[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, 
                       GLX_STENCIL_SIZE, 8, None };

  XVisualInfo *visual = glXChooseVisual( display, screen, (int*)attribList );

  dummy_window_id_ = XCreateSimpleWindow( display,
                                          RootWindow( display, screen ),
                                          0, 0, 1, 1, 0, 0, 0 );

  GLXContext context = glXCreateContext( display, visual, NULL, 1 );

  glXMakeCurrent( display, dummy_window_id_, context );
#endif
}

void RenderSystem::loadOgrePlugins()
{
  std::string suffix = "";

  std::string plugin_prefix;
#ifdef OGRE_PLUGIN_PATH
  // OGRE_PLUGIN_PATH is defined in package "ogre"'s manifest.xml in
  // the <export><cpp> entry for OSX.
  plugin_prefix = OGRE_PLUGIN_PATH + std::string("/");
#endif

  ogre_root_->loadPlugin( plugin_prefix + "RenderSystem_GL" + suffix );
  ogre_root_->loadPlugin( plugin_prefix + "Plugin_OctreeSceneManager" + suffix );
  ogre_root_->loadPlugin( plugin_prefix + "Plugin_ParticleFX" + suffix );
  ogre_root_->loadPlugin( plugin_prefix + "Plugin_CgProgramManager" + suffix );
}

void RenderSystem::setupRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  // Get the list of available renderers.
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  rsList = ogre_root_->getAvailableRenderers();
#else
  rsList = &(ogre_root_->getAvailableRenderers());
#endif
   
  // Look for the OpenGL one, which we require.
  renderSys = NULL;
  for( unsigned int i = 0; i < rsList->size(); i++ )
  {
    renderSys = rsList->at( i );
    if( renderSys->getName().compare("OpenGL Rendering Subsystem")== 0 )
    {
      break;
    }
  }

  if( renderSys == NULL )
  {
    throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen","No");

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy. 
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  //  renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  // Set the Full Screen Anti-Aliasing factor.
  renderSys->setConfigOption("FSAA", "2");

  ogre_root_->setRenderSystem(renderSys);
}

void RenderSystem::setupResources()
{
  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/textures", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/fonts", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/models", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/scripts", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/programs", "FileSystem", ROS_PACKAGE_NAME );
}

// On Intel graphics chips under X11, there sometimes comes a
// BadDrawable error during Ogre render window creation.  It is not
// consistent, happens sometimes but not always.  Underlying problem
// seems to be a driver bug.  My workaround here is to notice when
// that specific BadDrawable error happens on request 136 minor 3
// (which is what the problem looks like when it happens) and just try
// over and over again until it works (or until 100 failures, which
// makes it seem like it is a different bug).
static bool x_baddrawable_error = false;
#ifdef Q_WS_X11
static int (*old_error_handler)( Display*, XErrorEvent* );
int checkBadDrawable( Display* display, XErrorEvent* error )
{
  if( error->error_code == BadDrawable &&
      error->request_code == 136 &&
      error->minor_code == 3 )
  {
    x_baddrawable_error = true;
    return 0;
  }
  else
  {
    // If the error does not exactly match the one from the driver bug,
    // handle it the normal way so we see it.
    return old_error_handler( display, error );
  }
}
#endif // Q_WS_X11

Ogre::RenderWindow* RenderSystem::makeRenderWindow( intptr_t window_id, unsigned int width, unsigned int height )
{
  static int windowCounter = 0; // Every RenderWindow needs a unique name, oy.

  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  std::stringstream window_handle_stream;
  window_handle_stream << window_id;

#ifdef Q_OS_MAC
  params["externalWindowHandle"] = window_handle_stream.str();
#else
  params["parentWindowHandle"] = window_handle_stream.str();
#endif

  params["externalGLControl"] = true;

// Set the macAPI for Ogre based on the Qt implementation
#ifdef QT_MAC_USE_COCOA
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#else
  params["macAPI"] = "carbon";
#endif

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

#ifdef Q_WS_X11
  old_error_handler = XSetErrorHandler( &checkBadDrawable );
#endif

  int attempts = 0;
  while (window == NULL && (attempts++) < 100)
  {
    try
    {
      window = ogre_root_->createRenderWindow( stream.str(), width, height, false, &params );

      // If the driver bug happened, tell Ogre we are done with that
      // window and then try again.
      if( x_baddrawable_error )
      {
        ogre_root_->detachRenderTarget( window );
        window = NULL;
        x_baddrawable_error = false;
      }
    }
    catch( std::exception ex )
    {
      std::cerr << "rviz::RenderSystem: error creating render window: "
                << ex.what() << std::endl;
      window = NULL;
    }
  }

#ifdef Q_WS_X11
  XSetErrorHandler( old_error_handler );
#endif

  if( window == NULL )
  {
    ROS_ERROR( "Unable to create the rendering window after 100 tries." );
    assert(false);
  }

  if( attempts > 1 )
  {
    ROS_INFO( "Created render window after %d attempts.", attempts );
  }

  if (window)
  {
    window->setActive(true);
    //window->setVisible(true);
    window->setAutoUpdated(false);
  }

  return window;
}

} // end namespace rviz
