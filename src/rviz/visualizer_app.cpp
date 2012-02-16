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

#include <QApplication>
#include <QTimer>

#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <signal.h>

#include <OGRE/OgreHighLevelGpuProgramManager.h>
#include <OGRE/OgreLogManager.h>
#include <std_srvs/Empty.h>

#ifdef Q_OS_MAC
#include <ApplicationServices/ApplicationServices.h>
// Apparently OSX #defines 'check' to be an empty string somewhere.  
// That was fun to figure out.
#undef check
#endif

#include <ros/ros.h>

#include "visualization_frame.h"
#include "version.h"
#include "wait_for_master_dialog.h"
#include "visualizer_app.h"

#define CATCH_EXCEPTIONS 0

namespace po = boost::program_options;

namespace rviz
{

bool reloadShaders(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  Ogre::ResourceManager::ResourceMapIterator it = Ogre::HighLevelGpuProgramManager::getSingleton().getResourceIterator();
  while (it.hasMoreElements())
  {
    Ogre::ResourcePtr resource = it.getNext();
    resource->reload();
  }
  return true;
}

VisualizerApp::VisualizerApp()
  : timer_( 0 )
  , frame_( 0 )
{
}

void VisualizerApp::onTimer()
{
  if( !continue_ )
  {
    QApplication::closeAllWindows();
  }
}

bool VisualizerApp::init( int argc, char** argv )
{
  ROS_INFO( "rviz revision number %s", get_version().c_str() );
  ROS_INFO( "compiled against OGRE version %d.%d.%d%s (%s)",
            OGRE_VERSION_MAJOR, OGRE_VERSION_MINOR, OGRE_VERSION_PATCH,
            OGRE_VERSION_SUFFIX, OGRE_VERSION_NAME );

#ifdef Q_OS_MAC
  ProcessSerialNumber PSN;
  GetCurrentProcess(&PSN);
  TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
  SetFrontProcess(&PSN);
#endif

#if CATCH_EXCEPTIONS
  try
  {
#endif
    ros::init( argc, argv, "rviz", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler );

    po::options_description options;
    options.add_options()
      ("help,h", "Produce this help message")
      ("splash-screen,s", po::value<std::string>(), "A custom splash-screen image to display")
      ("help-file", po::value<std::string>(), "A custom html file to show as the help screen")
      ("display-config,d", po::value<std::string>(), "A display config file (.vcg) to load")
      ("target-frame,t", po::value<std::string>(), "Set the target frame")
      ("fixed-frame,f", po::value<std::string>(), "Set the fixed frame")
      ("ogre-log,l", "Enable the Ogre.log file (output in cwd)")
      ("in-mc-wrapper", "Signal that this is running inside a master-chooser wrapper")
      ("verbose,v", "Enable debug visualizations");
    po::variables_map vm;
    std::string display_config, target_frame, fixed_frame, splash_path, help_path;
    bool enable_ogre_log = false;
    bool in_mc_wrapper = false;
    bool verbose = false;
    try
    {
      po::store( po::parse_command_line( argc, argv, options ), vm );
      po::notify( vm );

      if( vm.count( "help" ))
      {
        std::cout << "rviz command line options:\n" << options;
        return false;
      }

      if( vm.count( "in-mc-wrapper" ))
      {
        in_mc_wrapper = true;
      }

      if (vm.count("display-config"))
      {
        display_config = vm["display-config"].as<std::string>();
      }

      if (vm.count("splash-screen"))
      {
        splash_path = vm["splash-screen"].as<std::string>();
      }

      if (vm.count("help-file"))
      {
        help_path = vm["help-file"].as<std::string>();
      }

      if (vm.count("target-frame"))
      {
        target_frame = vm["target-frame"].as<std::string>();
      }

      if (vm.count("fixed-frame"))
      {
        fixed_frame = vm["fixed-frame"].as<std::string>();
      }

      if (vm.count("ogre-log"))
      {
        enable_ogre_log = true;
      }

      if (vm.count("verbose"))
      {
        verbose = true;
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Error parsing command line: %s", e.what());
      return false;
    }

    if( !ros::master::check() )
    {
      WaitForMasterDialog* dialog = new WaitForMasterDialog;
      if( dialog->exec() != QDialog::Accepted )
      {
        return false;
      }
    }

    // block kill signals on all threads, since this also disables signals in threads
    // created by this one (the main thread)
    sigset_t sig_set;
    sigemptyset(&sig_set);
    sigaddset(&sig_set, SIGKILL);
    sigaddset(&sig_set, SIGTERM);
    sigaddset(&sig_set, SIGQUIT);
    sigaddset(&sig_set, SIGINT);
    pthread_sigmask(SIG_BLOCK, &sig_set, NULL);

    // Start up our signal handler
    continue_ = true;
    signal_handler_thread_ = boost::thread(boost::bind(&VisualizerApp::signalHandler, this));

    nh_.reset( new ros::NodeHandle );

    Ogre::LogManager* log_manager = new Ogre::LogManager();
    log_manager->createLog( "Ogre.log", false, false, !enable_ogre_log );

    frame_ = new VisualizationFrame;
    frame_->initialize( display_config, fixed_frame, target_frame,
                        splash_path, help_path, verbose, in_mc_wrapper );
    frame_->show();

    timer_ = new QTimer( this );
    connect( timer_, SIGNAL( timeout() ), this, SLOT( onTimer() ));
    timer_->start( 100 );

    ros::NodeHandle private_nh("~");
    reload_shaders_service_ = private_nh.advertiseService("reload_shaders", reloadShaders);
#if CATCH_EXCEPTIONS
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Caught exception while loading: %s", e.what());
    return false;
  }
#endif
  return true;
}

VisualizerApp::~VisualizerApp()
{
  if( timer_ )
  {
    timer_->stop();
  }
  continue_ = false;

  delete frame_;

  raise(SIGQUIT);

  signal_handler_thread_.join();
}

void VisualizerApp::signalHandler()
{
  sigset_t signal_set;
  while(continue_)
  {
    // Wait for any signals
    sigfillset(&signal_set);

#if defined(Q_OS_MAC)
    int sig;
    sigwait(&signal_set, &sig);
#else
    struct timespec ts = {0, 100000000};
    int sig = sigtimedwait(&signal_set, NULL, &ts);
#endif

    switch( sig )
    {
    case SIGKILL:
    case SIGTERM:
    case SIGQUIT:
    {
      exit(1);
    }
    break;

    case SIGINT:
    {
      ros::shutdown();
      continue_ = false;
      return;
    }
    break;

    default:
      break;
    }
  }
}

} // namespace rviz
