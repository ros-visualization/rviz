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

// X.h #defines CursorShape to be "0".  Qt uses CursorShape in normal
// C++ way.  This wasn't an issue until ogre_logging.h (below)
// introduced a #include of <QString>.
#ifdef CursorShape
#undef CursorShape
#endif

#include <ros/package.h> // This dependency should be moved out of here, it is just used for a search path.
#include <ros/console.h>

#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
#include <OgreOverlaySystem.h>
#endif

#include "rviz/env_config.h"
#include "rviz/ogre_helpers/ogre_logging.h"

#include "rviz/ogre_helpers/render_system.h"

#include <QMessageBox>

namespace rviz
{

RenderSystem* RenderSystem::instance_ = 0;
int RenderSystem::force_gl_version_ = 0;
bool RenderSystem::force_no_stereo_ = false;

RenderSystem* RenderSystem::get()
{
  if( instance_ == 0 )
  {
    instance_ = new RenderSystem();
  }
  return instance_;
}

void RenderSystem::forceGlVersion( int version )
{
  force_gl_version_ = version;
  ROS_INFO_STREAM( "Forcing OpenGl version " << (float)version / 100.0 << "." );
}

void RenderSystem::forceNoStereo()
{
  force_no_stereo_ = true;
  ROS_INFO("Forcing Stereo OFF");
}

RenderSystem::RenderSystem()
: ogre_overlay_system_(NULL)
, stereo_supported_(false)
{
  OgreLogging::configureLogging();

  std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);

  setupDummyWindowId();
  ogre_root_ = new Ogre::Root( rviz_path+"/ogre_media/plugins.cfg" );
#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
  ogre_overlay_system_ = new Ogre::OverlaySystem();
#endif
  loadOgrePlugins();
  setupRenderSystem();
  ogre_root_->initialise(false);
  makeRenderWindow( dummy_window_id_, 1, 1 );
  detectGlVersion();
  setupResources();
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

void RenderSystem::prepareOverlays(Ogre::SceneManager* scene_manager)
{
#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
  if (ogre_overlay_system_)
    scene_manager->addRenderQueueListener(ogre_overlay_system_);
#endif
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
  std::string plugin_prefix = get_ogre_plugin_path() + "/";
#ifdef Q_OS_MAC
  plugin_prefix += "lib";
#endif
  ogre_root_->loadPlugin( plugin_prefix + "RenderSystem_GL" );
  ogre_root_->loadPlugin( plugin_prefix + "Plugin_OctreeSceneManager" );
  ogre_root_->loadPlugin( plugin_prefix + "Plugin_ParticleFX" );
}

void RenderSystem::detectGlVersion()
{
  if ( force_gl_version_ )
  {
    gl_version_ = force_gl_version_;
  }
  else
  {
    Ogre::RenderSystem *renderSys = ogre_root_->getRenderSystem();
    renderSys->createRenderSystemCapabilities();
    const Ogre::RenderSystemCapabilities* caps = renderSys->getCapabilities();
    int major = caps->getDriverVersion().major;
    int minor = caps->getDriverVersion().minor;
    gl_version_ = major * 100 + minor*10;
  }

  switch ( gl_version_ )
  {
    case 200:
      glsl_version_ = 110;
      break;
    case 210:
      glsl_version_ = 120;
      break;
    case 300:
      glsl_version_ = 130;
      break;
    case 310:
      glsl_version_ = 140;
      break;
    case 320:
      glsl_version_ = 150;
      break;
    default:
      if ( gl_version_ > 320 )
      {
        glsl_version_  = gl_version_;
      }
      else
      {
        glsl_version_ = 0;
      }
      break;
  }
  ROS_INFO_STREAM( "OpenGl version: " << (float)gl_version_ / 100.0 << " (GLSL " << (float)glsl_version_ / 100.0 << ")." );
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
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/glsl120", "FileSystem", ROS_PACKAGE_NAME );
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/glsl120/nogp", "FileSystem", ROS_PACKAGE_NAME );
  // Add resources that depend on a specific glsl version.
  // Unfortunately, Ogre doesn't have a notion of glsl versions so we can't go
  // the 'official' way of defining multiple schemes per material and let Ogre decide which one to use.
  if ( getGlslVersion() >= 150  )
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/glsl150", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/scripts150", "FileSystem", ROS_PACKAGE_NAME );
  }
  else if ( getGlslVersion() >= 120  )
  {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media/materials/scripts120", "FileSystem", ROS_PACKAGE_NAME );
  }
  else
  {
    std::string s = "Your graphics driver does not support OpenGL 2.1. Please enable software rendering before running RViz (e.g. type 'export LIBGL_ALWAYS_SOFTWARE=1').";
    QMessageBox msgBox;
    msgBox.setText(s.c_str());
    msgBox.exec();
    throw std::runtime_error( s );
  }

  // Add paths exported to the "media_export" package.
  std::vector<std::string> media_paths;
  ros::package::getPlugins( "media_export", "ogre_media_path", media_paths );
  std::string delim(":");
  for( std::vector<std::string>::iterator iter = media_paths.begin(); iter != media_paths.end(); iter++ )
  {
    if( !iter->empty() )
    {
      std::string path;
      int pos1 = 0;
      int pos2 = iter->find(delim);
      while( pos2 != (int)std::string::npos )
      {
        path = iter->substr( pos1, pos2 - pos1 );
        ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation( path, "FileSystem", ROS_PACKAGE_NAME );
        pos1 = pos2 + 1;
        pos2 = iter->find( delim, pos2 + 1 );
      }
      path = iter->substr( pos1, iter->size() - pos1 );
      ROS_DEBUG("adding resource location: '%s'\n", path.c_str());
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation( path, "FileSystem", ROS_PACKAGE_NAME );
    }
  }
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


  // don't bother trying stereo if Ogre does not support it.
#if !OGRE_STEREO_ENABLE
  force_no_stereo_ = true;
#endif

  // attempt to create a stereo window
  bool is_stereo = false;
  if (!force_no_stereo_)
  {
    params["stereoMode"] = "Frame Sequential";
    window = tryMakeRenderWindow( stream.str(), width, height, &params, 100);
    params.erase("stereoMode");

    if (window)
    {
#if OGRE_STEREO_ENABLE
      is_stereo = window->isStereoEnabled();
#endif
      if (!is_stereo)
      {
        // Created a non-stereo window.  Discard it and try again (below)
        // without the stereo parameter.
        ogre_root_->detachRenderTarget(window);
        window->destroy();
        window = NULL;
        stream << "x";
        is_stereo = false;
      }
    }
  }

  if ( window == NULL )
  {
    window = tryMakeRenderWindow( stream.str(), width, height, &params, 100);
  }

  if( window == NULL )
  {
    ROS_ERROR( "Unable to create the rendering window after 100 tries." );
    assert(false);
  }

  if (window)
  {
    window->setActive(true);
    //window->setVisible(true);
    window->setAutoUpdated(false);
  }

  stereo_supported_ = is_stereo;

  ROS_INFO_ONCE("Stereo is %s", stereo_supported_ ? "SUPPORTED" : "NOT SUPPORTED");

  return window;
}

Ogre::RenderWindow* RenderSystem::tryMakeRenderWindow(
      const std::string& name,
      unsigned int width,
      unsigned int height,
      const Ogre::NameValuePairList* params,
      int max_attempts )
{
  Ogre::RenderWindow *window = NULL;
  int attempts = 0;

#ifdef Q_WS_X11
  old_error_handler = XSetErrorHandler( &checkBadDrawable );
#endif

  while (window == NULL && (attempts++) < max_attempts)
  {
    try
    {
      window = ogre_root_->createRenderWindow( name, width, height, false, params );

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

  if( window && attempts > 1 )
  {
    ROS_INFO( "Created render window after %d attempts.", attempts );
  }

  return window;
}


} // end namespace rviz
