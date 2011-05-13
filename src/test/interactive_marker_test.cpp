#include "ros/ros.h"

#include <visualization_msgs/InteractiveMarkerArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

InteractiveMarkerArray int_marker_array;
float marker_pos = 0;

////////////////////////////////////////////////////////////////////////////////////

void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  t.setOrigin(tf::Vector3(0.0, 0.0, (counter % 1000) * 0.01));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 10.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, M_PI*0.1));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "move_rotate_frame"));

  ++counter;
}

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}


Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.size * 0.45;
  marker.scale.y = msg.size * 0.45;
  marker.scale.z = msg.size * 0.45;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

void addTitle( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.x = msg.size * 0.15;
  marker.scale.y = msg.size * 0.15;
  marker.scale.z = msg.size * 0.15;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.pose.position.z = 1.5;
  marker.text = msg.name;

  InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = InteractiveMarkerControl::NONE;
  int_marker_control.always_visible = true;
  int_marker_control.marker_orientation = InteractiveMarkerControl::FIXED;
  int_marker_control.markers.push_back( marker );

  msg.controls.push_back( int_marker_control );
}

InteractiveMarkerControl makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl int_marker_control;
  int_marker_control.always_visible = true;
  int_marker_control.markers.push_back( makeBox(msg) );

  return int_marker_control;
}

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{
  std_msgs::Header header;
  header.frame_id = "/base_link";
//  header.stamp = ros::Time::now();

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.y = -2.0 * marker_pos++;
  pose.position.x = -2.0;

  InteractiveMarker int_marker;
  int_marker.header = header;
  int_marker.pose = pose;
  int_marker.size = 1.0;
  int_marker.frame_locked = false;

  return int_marker;
}

void saveMarker( InteractiveMarker int_marker )
{
  addTitle(int_marker);
  int_marker_array.markers.push_back(int_marker);
}

////////////////////////////////////////////////////////////////////////////////////


void make6DofMarker( bool fixed )
{
  InteractiveMarker int_marker = makeEmptyMarker();

  int_marker.name = "Simple 6-DOF Control";

  int_marker.size = 1.0;
  int_marker.header.frame_id = "/move_rotate_frame";

  int_marker.controls.push_back( makeBoxControl(int_marker));

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "\n(fixed orientation)";
    control.marker_orientation = InteractiveMarkerControl::FIXED;
  }

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

void makeRandomDofMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "Axis Move & Rotate\n(Non-Unit Axes)";
  int_marker.header.frame_id = "/move_rotate_frame";

  int_marker.controls.push_back( makeBoxControl(int_marker) );

  InteractiveMarkerControl control;

  for ( int i=0; i<3; i++ )
  {
    control.orientation.w = rand(-1,1);
    control.orientation.x = rand(-1,1);
    control.orientation.y = rand(-1,1);
    control.orientation.z = rand(-1,1);
    control.mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  saveMarker( int_marker );
}

void makeQuadrocopterMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "Quadrocopter\n(Dog Walk + Elevator)";
  int_marker.header.frame_id = "/move_rotate_frame";

  int_marker.controls.push_back( makeBoxControl(int_marker) );

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

void makeChessPieceMarker( )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = "Chess Piece\n(2D move)";
  int_marker.header.frame_id = "/move_rotate_frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_test");
  ros::NodeHandle n;

  ros::Publisher marker_pub;
  marker_pub = n.advertise<InteractiveMarkerArray> ("interactive_marker_array", 0, true);
//  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

//  srand( ros::Time::now().sec );

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  make6DofMarker( false );
  make6DofMarker( true );
  makeRandomDofMarker( );
  makeQuadrocopterMarker( );
  makeChessPieceMarker( );

  ROS_INFO( "Publishing %d markers", (int)int_marker_array.markers.size() );
  marker_pub.publish( int_marker_array );

  ros::spin();
}
