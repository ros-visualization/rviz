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
#ifndef OGRE_LOGGING_H
#define OGRE_LOGGING_H

#include <QString>

namespace rviz
{

/** @brief Convenience interface to Ogre logging.
 *
 * This all-static class wraps Ogre::LogManager into 3 easy options:
 * no logging, standard out, or file logging.  The option-selection
 * calls (useStandardOut(), useLogFile(), and noLog() must be called
 * before configureLogging().  configureLogging(), in turn, must be
 * called before any Ogre::Root object is instantiated.
 * configureLogging() is called at the right time by the RenderSystem
 * constructor, so you generally won't need to call it explicitly. */
class OgreLogging
{
public:
  /** @brief Configure Ogre to write output to the ROS logger. */
  static void useRosLog();

  /** @brief Configure Ogre to write output to the given log file
   * name.  If file name is a relative path, it will be relative to
   * the directory which is current when the program is run.  Default
   * is "Ogre.log". */
  static void useLogFile( const QString& filename = "Ogre.log" );

  /** @brief Disable Ogre logging entirely.  This is the default. */
  static void noLog();

  /** @brief Configure the Ogre::LogManager to give the
   * currently selected behavior.
   * This must be called before Ogre::Root is instantiated! */
  static void configureLogging();

private:
  typedef enum { StandardOut, FileLogging, NoLogging } Preference;
  static Preference preference_;
  static QString filename_;
};

} // end namespace rviz

#endif // OGRE_LOGGING_H
