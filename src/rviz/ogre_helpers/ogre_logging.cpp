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

#include <OgreLogManager.h>
#include <OgreLog.h>

#include <ros/ros.h>

#include "rviz/ogre_helpers/ogre_logging.h"

namespace rviz
{

class RosLogListener: public Ogre::LogListener
{
public:
  RosLogListener(): min_lml(Ogre::LML_CRITICAL) {};
  virtual ~RosLogListener() {}

#if OGRE_VERSION >= ((1 << 16) | (8 << 8))
  virtual void messageLogged( const Ogre::String& message, Ogre::LogMessageLevel lml, bool maskDebug, const Ogre::String &logName, bool& skipThisMessage )
  {
    if ( !skipThisMessage )
    {
      if ( lml >= min_lml )
      {
        ROS_LOG((ros::console::levels::Level)(lml-1), ROSCONSOLE_DEFAULT_NAME, "%s", message.c_str() );
      }
    }
   }
#else
  virtual void messageLogged( const Ogre::String& message, Ogre::LogMessageLevel lml, bool maskDebug, const Ogre::String &logName )
  {
    if ( lml >= min_lml )
    {
      ROS_LOG((ros::console::levels::Level)(lml-1), ROSCONSOLE_DEFAULT_NAME, "%s", message.c_str() );
    }
  }
#endif
  Ogre::LogMessageLevel min_lml;
};

OgreLogging::Preference OgreLogging::preference_ = OgreLogging::NoLogging;
QString OgreLogging::filename_;

/** @brief Configure Ogre to write output to standard out. */
void OgreLogging::useRosLog()
{
  preference_ = StandardOut;
}

/** @brief Configure Ogre to write output to the given log file
 * name.  If file name is a relative path, it will be relative to
 * the directory which is current when the program is run.  Default
 * is "Ogre.log". */
void OgreLogging::useLogFile( const QString& filename )
{
  preference_ = FileLogging;
  filename_ = filename;
}

/** @brief Disable Ogre logging entirely.  This is the default. */
void OgreLogging::noLog()
{
  preference_ = NoLogging;
}

/** @brief Configure the Ogre::LogManager to give the behavior
 * selected by the most recent call to enableStandardOut(),
 * setLogFile(), or disable().  This must be called before
 * Ogre::Root is instantiated!  Called inside RenderSystem
 * constructor. */
void OgreLogging::configureLogging()
{
  static RosLogListener ll;
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if( log_manager == NULL )
  {
    log_manager = new Ogre::LogManager();
  }
  Ogre::Log* l = log_manager->createLog( filename_.toStdString(), false, false, preference_==NoLogging );
  l->addListener( &ll );

  // Printing to standard out is what Ogre does if you don't do any LogManager calls.
  if( preference_ == StandardOut )
  {
    ll.min_lml=Ogre::LML_NORMAL;
  }
}

} // end namespace rviz
