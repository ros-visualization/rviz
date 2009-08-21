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

/**
 * \file
 */

#include <wx/app.h>
#include <wx/timer.h>
#include "visualization_frame.h"
#include "wx_log_rosout.h"
#include <ogre_tools/initialization.h>

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <signal.h>

#ifdef __WXMAC__
#include <ApplicationServices/ApplicationServices.h>
#endif

namespace po = boost::program_options;

namespace rviz
{

class VisualizerApp : public wxApp
{
public:
  char** local_argv_;
  VisualizationFrame* frame_;
  volatile bool continue_;
  boost::thread signal_handler_thread_;
  wxTimer timer_;
  ros::NodeHandlePtr nh_;

  VisualizerApp()
  : timer_(this)
  {
  }

  void onTimer(wxTimerEvent&)
  {
    if (!continue_)
    {
      frame_->Close();
    }
  }

  bool OnInit()
  {
#ifdef __WXMAC__
    ProcessSerialNumber PSN;
    GetCurrentProcess(&PSN);
    TransformProcessType(&PSN,kProcessTransformToForegroundApplication);
    SetFrontProcess(&PSN);
#endif

    wxLog::SetActiveTarget(new wxLogRosout());

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

    ogre_tools::initializeOgre();

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_, "rviz", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);

    po::options_description options;
    options.add_options()
             ("help,h", "Produce this help message")
             ("display-config,d", po::value<std::string>(), "A display config file (.vcg) to load")
             ("target-frame,t", po::value<std::string>(), "Set the target frame")
             ("fixed-frame,f", po::value<std::string>(), "Set the fixed frame");
    po::variables_map vm;
    std::string display_config, target_frame, fixed_frame;
    try
    {
      po::store(po::parse_command_line(argc, local_argv_, options), vm);
      po::notify(vm);

      if (vm.count("help"))
      {
        std::cout << "rviz command line options:\n" << options;
        return false;
      }


      if (vm.count("display-config"))
      {
        display_config = vm["display-config"].as<std::string>();
      }

      if (vm.count("target-frame"))
      {
        target_frame = vm["target-frame"].as<std::string>();
      }

      if (vm.count("fixed-frame"))
      {
        fixed_frame = vm["fixed-frame"].as<std::string>();
      }
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Error parsing command line: %s", e.what());
      return false;
    }

    frame_ = new VisualizationFrame(NULL);
    frame_->initialize(display_config, fixed_frame, target_frame);

    SetTopWindow(frame_);
    frame_->Show();

    Connect(timer_.GetId(), wxEVT_TIMER, wxTimerEventHandler(VisualizerApp::onTimer), NULL, this);
    timer_.Start(100);

    return true;
  }

  int OnExit()
  {
    timer_.Stop();
    continue_ = false;

    raise(SIGQUIT);

    signal_handler_thread_.join();

    for ( int i = 0; i < argc; ++i )
    {
      free( local_argv_[ i ] );
    }
    delete [] local_argv_;

    ogre_tools::cleanupOgre();

    return 0;
  }

  void signalHandler()
  {
    sigset_t signal_set;
    while(continue_)
    {
      // Wait for any signals
      sigfillset(&signal_set);

#if defined(__WXMAC__)
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
      case SIGINT:
        {
          continue_ = false;

          ros::shutdown();
          return;
        }
        break;

      default:
        break;
      }
    }
  }
};

DECLARE_APP(VisualizerApp);

} // namespace rviz


IMPLEMENT_APP(rviz::VisualizerApp);
