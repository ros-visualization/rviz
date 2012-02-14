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


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

// create an interactive marker server on the topic namespace simple_marker
interactive_markers::InteractiveMarkerServer* server;

visualization_msgs::InteractiveMarker makeMarker( float r, float g, float b )
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "my_marker";
  int_marker.description = "Simple 1-DOF Control";

  // create a box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = r;
  box_marker.color.g = g;
  box_marker.color.b = b;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl linear_control;
  linear_control.name = "move_x";
  linear_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  int_marker.controls.push_back(linear_control);

  return int_marker;
}

bool is_red = false;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );

  bool changed = false;
  visualization_msgs::InteractiveMarker int_marker;

  // red when x < 0, green otherwise.  Update marker color when x
  // crosses boundary.
  if( feedback->pose.position.x < 0 && !is_red )
  {
    printf( "turning red.\n" );
    is_red = true;
    int_marker = makeMarker( 1, 0, 0 );
    changed = true;
  }
  if( feedback->pose.position.x >= 0 && is_red )
  {
    printf( "turning green.\n" );
    is_red = false;
    int_marker = makeMarker( 0, 1, 0 );
    changed = true;
  }

  if( changed )
  {
    printf( "changed.\n" );
    int_marker.pose = feedback->pose;
    server->insert( int_marker );
    server->applyChanges();
  }
}

visualization_msgs::InteractiveMarker makeCrazyMarker( bool linear )
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/base_link";
  int_marker.name = "crazy_marker";
  int_marker.description = "Unusual 1-DOF Control";

  // create a box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 1;
  box_marker.scale.y = .3;
  box_marker.scale.z = .3;
  box_marker.color.r = .3;
  box_marker.color.g = .1;
  box_marker.color.b = 1;
  box_marker.color.a = 1.0;
  box_marker.pose.position.y = -1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );
  box_control.name = "crazy";
  if( linear )
  {
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  }
  else
  {
    box_control.orientation.w = 1;
    box_control.orientation.x = 0;
    box_control.orientation.y = 1;
    box_control.orientation.z = 0;
    box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  }

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );
  int_marker.pose.position.y = 3;

  return int_marker;
}

bool is_linear = true;

void processCrazyFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at pos "
                   << feedback->pose.position.x << ", "
                   << feedback->pose.position.y << ", "
                   << feedback->pose.position.z << "; quat "
                   << feedback->pose.orientation.x << ", "
                   << feedback->pose.orientation.y << ", "
                   << feedback->pose.orientation.z << ", "
                   << feedback->pose.orientation.w );

  bool changed = false;
  visualization_msgs::InteractiveMarker int_marker;

  // linear when x < 0, rotary otherwise.  Update when x
  // crosses boundary.
  if( feedback->pose.orientation.z < 0 && !is_linear )
  {
    printf( "turning linear.\n" );
    is_linear = true;
    int_marker = makeCrazyMarker( true );
    changed = true;
  }
  if( feedback->pose.position.x > 0 && is_linear )
  {
    printf( "turning rotary.\n" );
    is_linear = false;
    int_marker = makeCrazyMarker( false );
    changed = true;
  }

  if( changed )
  {
    printf( "changed.\n" );
    int_marker.pose.position.x = 0;
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = 0;
    int_marker.pose.orientation.w = 1;
    server->insert( int_marker );
    server->applyChanges();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_test");

  // create an interactive marker server on the topic namespace simple_marker
  server = new interactive_markers::InteractiveMarkerServer("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker = makeMarker(0, 1, 0);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker, &processFeedback);

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker crazy_marker = makeCrazyMarker( true );

  // add the interactive marker to our collection &
  // tell the server to call processCrazyFeedback() when feedback arrives for it
  server->insert(crazy_marker, &processCrazyFeedback);

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
