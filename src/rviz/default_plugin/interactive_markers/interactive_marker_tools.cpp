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

#include "interactive_marker_tools.h"

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

#include <math.h>
#include <assert.h>

namespace rviz
{

void autoComplete( visualization_msgs::InteractiveMarker &msg )
{
  // this is a 'delete' message. no need for action.
  if ( msg.controls.empty() )
  {
    return;
  }

  // default size
  if ( msg.size == 0 )
  {
    msg.size = 1;
  }

  // correct empty orientation, normalize
  if ( msg.pose.orientation.w == 0 && msg.pose.orientation.x == 0 &&
      msg.pose.orientation.y == 0 && msg.pose.orientation.z == 0 )
  {
    msg.pose.orientation.w = 1;
  }

  //normalize quaternion
  btQuaternion int_marker_orientation( msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w );
  int_marker_orientation.normalize();
  msg.pose.orientation.x = int_marker_orientation.x();
  msg.pose.orientation.y = int_marker_orientation.y();
  msg.pose.orientation.z = int_marker_orientation.z();
  msg.pose.orientation.w = int_marker_orientation.w();

  // complete the controls
  for ( unsigned c=0; c<msg.controls.size(); c++ )
  {
    autoComplete( msg, msg.controls[c] );
  }
}

void autoComplete( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control )
{
  // correct empty orientation, normalize
  if ( control.orientation.w == 0 && control.orientation.x == 0 &&
      control.orientation.y == 0 && control.orientation.z == 0 )
  {
    control.orientation.w = 1;
  }

  // add default control handles if there are none
  if ( control.markers.empty() )
  {
    switch ( control.mode )
    {
      case visualization_msgs::InteractiveMarkerControl::NONE:
        break;

      case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS:
        control.markers.reserve(2);
        makeArrow( msg, control, 1.0 );
        makeArrow( msg, control, -1.0 );
        break;

      case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE:
      case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
      case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE:
        makeDisc( msg, control );
        break;

      case visualization_msgs::InteractiveMarkerControl::BUTTON:
        break;
    }
  }

  // get interactive marker pose for later
  btQuaternion int_marker_orientation( msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w );
  btVector3 int_marker_position( msg.pose.position.x, msg.pose.position.y, msg.pose.position.z );

  // fill in missing pose information into the markers
  for ( unsigned m=0; m<control.markers.size(); m++ )
  {
    visualization_msgs::Marker &marker = control.markers[m];

    if ( marker.scale.x == 0  && marker.scale.z == 0 && marker.scale.z == 0 )
    {
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;
    }

    // make ns = name (might be useful)
    marker.ns = msg.name;

    // correct empty orientation
    if ( marker.pose.orientation.w == 0 && marker.pose.orientation.x == 0 &&
        marker.pose.orientation.y == 0 && marker.pose.orientation.z == 0 )
    {
      marker.pose.orientation.w = 1;
    }

    //normalize orientation
    btQuaternion marker_orientation( marker.pose.orientation.x, marker.pose.orientation.y,
        marker.pose.orientation.z, marker.pose.orientation.w );
    btVector3 marker_position( marker.pose.position.x, marker.pose.position.y, marker.pose.position.z );

    marker_orientation.normalize();

    // if the header is empty, interpret as local coordinates relative to interacitve marker pose
    if ( marker.header.frame_id.empty() )
    {
      marker.header = msg.header;

      // interpret marker pose as relative to interactive marker pose
      marker_orientation = int_marker_orientation * marker_orientation;
      marker_position = int_marker_position + ( btMatrix3x3(int_marker_orientation) * marker_position);
    }

    // write back corrected pose
    marker.pose.position.x = marker_position.x();
    marker.pose.position.y = marker_position.y();
    marker.pose.position.z = marker_position.z();

    marker.pose.orientation.x = marker_orientation.x();
    marker.pose.orientation.y = marker_orientation.y();
    marker.pose.orientation.z = marker_orientation.z();
    marker.pose.orientation.w = marker_orientation.w();
  }
}

void makeArrow( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control, float pos )
{
  visualization_msgs::Marker marker;

  // rely on the auto-completion for the correct orientation
  marker.pose.orientation = control.orientation;

  ROS_INFO( "%f %f %f %f", control.orientation.w,control.orientation.x,control.orientation.y,control.orientation.z );

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = msg.size * 0.3;
  marker.scale.y = msg.size * 0.5;
  marker.scale.z = msg.size * 0.2;

  assignDefaultColor(marker, control.orientation);

  float dist = fabs(pos);
  float dir = pos > 0 ? 1 : -1;

  float inner = 0.5 * dist;
  float outer = inner + 0.4;

  marker.points.resize(2);
  marker.points[0].x = dir * msg.size * inner;
  marker.points[1].x = dir * msg.size * outer;

  control.markers.push_back( marker );
}

void makeDisc( const visualization_msgs::InteractiveMarker &msg,
    visualization_msgs::InteractiveMarkerControl &control, float width )
{
  visualization_msgs::Marker marker;

  // rely on the auto-completion for the correct orientation
  marker.pose.orientation = control.orientation;

  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.scale.x = msg.size;
  marker.scale.y = msg.size;
  marker.scale.z = msg.size;

  assignDefaultColor(marker, control.orientation);

  // compute points on a circle in the y-z plane
  int steps = 18;
  std::vector<geometry_msgs::Point> circle1, circle2;
  circle1.reserve(steps);
  circle2.reserve(steps);

  geometry_msgs::Point v1,v2;

  for ( int i=0; i<steps; i++ )
  {
    float a = float(i)/float(steps) * M_PI * 2.0;

    v1.y = 0.5 * cos(a);
    v1.z = 0.5 * sin(a);

    v2.y = (1+width) * v1.y;
    v2.z = (1+width) * v1.z;

    circle1.push_back( v1 );
    circle2.push_back( v2 );
  }

  //construct disc from several segments, as otherwise z sorting won't work nicely
  control.markers.reserve( control.markers.size() + steps );

  switch ( control.mode )
  {
    case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
      marker.points.resize(6);
      for ( int i=0; i<steps; i++ )
      {
        int i1 = i;
        int i2 = (i+1) % steps;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i1];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle2[i1];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle1[i2];

        marker.color.a = 0.4 + 0.2 * (i%2);
        control.markers.push_back(marker);
      }
      break;

    case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE:
      marker.points.resize(6);
      for ( int i=0; i<steps; i+=2 )
      {
        int i1 = i;
        int i2 = (i+1) % steps;
        int i3 = (i+2) % steps;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i2];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle1[i2];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle1[i3];

        marker.color.a = 0.4;
        control.markers.push_back(marker);

        marker.points[0] = circle2[i1];
        marker.points[1] = circle2[i2];
        marker.points[2] = circle1[i1];

        marker.points[3] = circle2[i2];
        marker.points[4] = circle2[i3];
        marker.points[5] = circle1[i3];

        marker.color.a = 0.6;
        control.markers.push_back(marker);
      }
      break;

    default:
      marker.points.resize(6);

      marker.colors.push_back( marker.color );
      marker.colors.push_back( marker.color );
      marker.colors.push_back( marker.color );
      marker.colors.push_back( marker.color );
      marker.colors.push_back( marker.color );
      marker.colors.push_back( marker.color );
      marker.color.r=0;
      marker.color.g=0;
      marker.color.b=0;

      for ( int i=0; i<steps; i++ )
      {
        int i1 = i;
        int i2 = (i+1) % steps;

        marker.points[0] = circle1[i1];
        marker.points[1] = circle2[i1];
        marker.points[2] = circle1[i2];

        marker.points[3] = circle2[i1];
        marker.points[4] = circle2[i2];
        marker.points[5] = circle1[i2];

        control.markers.push_back(marker);
      }
  }
}

void assignDefaultColor(visualization_msgs::Marker &marker, const geometry_msgs::Quaternion &quat )
{
  geometry_msgs::Vector3 v;

  btQuaternion bt_quat( quat.x, quat.y, quat.z, quat.w );
  btVector3 bt_x_axis = btMatrix3x3(bt_quat) * btVector3(1,0,0);

  float x,y,z;
  x = fabs(bt_x_axis.x());
  y = fabs(bt_x_axis.y());
  z = fabs(bt_x_axis.z());

  float max_xy = x>y ? x : y;
  float max_yz = y>z ? y : z;
  float max_xyz = max_xy > max_yz ? max_xy : max_yz;

  marker.color.r = x / max_xyz;
  marker.color.g = y / max_xyz;
  marker.color.b = z / max_xyz;
  marker.color.a = 0.5;
}

}
