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

#include <wx/wx.h>
#include <wx/timer.h>
#include "visualization_frame.h"
#include <ogre_tools/initialization.h>

#include <ros/common.h>
#include <ros/node.h>

#include <boost/thread.hpp>
#include <signal.h>


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
    // block all signals on all threads, since this also disables signals in threads
    // created by this one (the main thread)
    ros::disableAllSignalsInThisThread();

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

    ros::init(argc, local_argv_);
    new ros::Node( "rviz", ros::Node::DONT_HANDLE_SIGINT | ros::Node::ANONYMOUS_NAME );

    frame_ = new VisualizationFrame(NULL);
    SetTopWindow(frame_);

    frame_->initialize();
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

    delete ros::Node::instance();

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
      struct timespec ts = {0, 100000000};
      int sig = sigtimedwait(&signal_set, NULL, &ts);

      switch( sig )
      {
      case SIGKILL:
      case SIGTERM:
      case SIGQUIT:
      case SIGINT:
        {
          continue_ = false;
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
