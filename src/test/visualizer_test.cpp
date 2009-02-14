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

#include "ros/node.h"
#include "ros/common.h"

#include "ogre_tools/initialization.h"

#include "visualization_panel.h"
#include "visualization_manager.h"
#include "displays/grid_display.h"
#include "displays/axes_display.h"
#include "displays/point_cloud_display.h"
#include "displays/laser_scan_display.h"
#include "displays/robot_model_display.h"
#include "displays/marker_display.h"
#include "displays/planning_display.h"

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>

using namespace ogre_vis;

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Ogre Display Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)
  {
    ogre_tools::initializeOgre();

    visualization_panel_ = new VisualizationPanel( this );

    root_ = Ogre::Root::getSingletonPtr();

    std::string mediaPath = ros::getPackagePath( "gazebo_robot_description" );
    mediaPath += "/world/Media/";
    ogre_tools::V_string paths;
    paths.push_back( mediaPath );
    paths.push_back( mediaPath + "fonts" );
    paths.push_back( mediaPath + "materials" );
    paths.push_back( mediaPath + "materials/scripts" );
    paths.push_back( mediaPath + "materials/programs" );
    paths.push_back( mediaPath + "materials/textures" );
    paths.push_back( mediaPath + "models" );
    paths.push_back( mediaPath + "models/pr2" );

    ogre_tools::initializeResources( paths );

    VisualizationManager* manager = visualization_panel_->getManager();

    manager->createDisplay<GridDisplay>( "Grid", true );
    manager->createDisplay<AxesDisplay>( "Origin Axes", false );

    RobotModelDisplay* model = manager->createDisplay<RobotModelDisplay>( "Robot Model", false );
    model->setRobotDescription( "robotdesc/pr2" );

    PlanningDisplay* planning = manager->createDisplay<PlanningDisplay>( "Planning", false );
    planning->initialize( "robotdesc/pr2", "display_kinematic_path" );

    PointCloudDisplay* pointCloud = manager->createDisplay<PointCloudDisplay>( "Stereo Full Cloud", false );
    pointCloud->setTopic( "videre/cloud" );
    pointCloud->setMaxColor( Color( 1.0, 1.0, 1.0 ) );

    pointCloud = manager->createDisplay<PointCloudDisplay>( "Head Full Cloud", false );
    pointCloud->setTopic( "full_cloud" );
    pointCloud->setMaxColor( Color( 1.0, 1.0, 0.0 ) );

    pointCloud = manager->createDisplay<PointCloudDisplay>( "World 3D Map", false );
    pointCloud->setTopic( "world_3d_map" );
    pointCloud->setMaxColor( Color( 1.0f, 0.0f, 0.0f ) );
    pointCloud->setBillboardSize( 0.01 );

    LaserScanDisplay* laserScan = manager->createDisplay<LaserScanDisplay>( "Head Scan", false );
    laserScan->setTopic( "tilt_scan" );
    laserScan->setMaxColor( Color( 1.0, 0.0, 0.0 ) );
    laserScan->setDecayTime( 30.0f );

    laserScan = manager->createDisplay<LaserScanDisplay>( "Floor Scan", false );
    laserScan->setTopic( "base_scan" );
    laserScan->setMaxColor( Color( 0.0f, 1.0f, 0.0f ) );
    laserScan->setDecayTime( 0.0f );

    manager->createDisplay<MarkerDisplay>( "Visualization Markers", false );
  }

  ~MyFrame()
  {
    visualization_panel_->Destroy();

    delete root_;
  }

private:

  Ogre::Root* root_;
  Ogre::Camera* camera_;
  Ogre::SceneManager* scene_manager_;

  VisualizationPanel* visualization_panel_;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:
  char** localArgv;

  bool OnInit()
  {
    // create our own copy of argv, with regular char*s.
    localArgv =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      localArgv[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, localArgv);
    new ros::Node( "Visualizer Test", ros::Node::DONT_HANDLE_SIGINT );

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {


    for ( int i = 0; i < argc; ++i )
    {
      free( localArgv[ i ] );
    }
    delete [] localArgv;

    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);
