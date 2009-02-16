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
#include "visualization_frame.h"
#include <ogre_tools/initialization.h>

#include <ros/node.h>

namespace ogre_vis
{

class VisualizerApp : public wxApp
{
public:
  char** local_argv_;

  bool OnInit()
  {
    ogre_tools::initializeOgre();

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_);
    new ros::Node( "visualizer", ros::Node::DONT_HANDLE_SIGINT | ros::Node::ANONYMOUS_NAME );

    VisualizationFrame* frame = new VisualizationFrame(NULL);
    SetTopWindow(frame);

    frame->initialize();
    frame->Show();

    return true;
  }

  int OnExit()
  {
    for ( int i = 0; i < argc; ++i )
    {
      free( local_argv_[ i ] );
    }
    delete [] local_argv_;

    delete ros::Node::instance();

    ogre_tools::cleanupOgre();

    return 0;
  }
};

DECLARE_APP(VisualizerApp);

} // namespace ogre_vis


IMPLEMENT_APP(ogre_vis::VisualizerApp);
