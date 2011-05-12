#include "ros/ros.h"

#include <visualization_msgs/InteractiveMarkerArray.h>
#include <visualization_msgs/interactive_marker_tools.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

visualization_msgs::InteractiveMarkerArray int_marker_array;
float marker_pos = 0;

void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  t.setOrigin(tf::Vector3(0.0, 0.0, (counter % 1000) * 0.01));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "moving_frame"));

  t.setOrigin(tf::Vector3(0.0, 10.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(M_PI*0.25, M_PI*0.25, 0.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "move_rotate_frame"));

  ++counter;
}

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}


visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.size * 0.6;
  marker.scale.y = msg.size * 0.6;
  marker.scale.z = msg.size * 0.6;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.3;

  return marker;
}

visualization_msgs::InteractiveMarkerControl makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.NONE;
  int_marker_control.always_visible = true;
  int_marker_control.markers.push_back( makeBox(msg) );

  return int_marker_control;
}

visualization_msgs::InteractiveMarker getEmptyMarker( bool dummyBox=true )
{
  std_msgs::Header header;
  header.frame_id = "/base_link";
  header.stamp = ros::Time::now();

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.y = -2.0 * marker_pos++;
  pose.position.x = -2.0;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = header;
  int_marker.pose = pose;
  int_marker.size = 1.0;
  int_marker.frame_locked = false;

  return int_marker;
}

void saveMarker( visualization_msgs::InteractiveMarker int_marker )
{
  int_marker.controls.push_back( makeBoxControl(int_marker));

  visualization_msgs::autoComplete(int_marker);

  int_marker_array.markers.push_back(int_marker);
}

void make6DofMarker( )
{
  visualization_msgs::InteractiveMarker int_marker = getEmptyMarker();
  int_marker.name = "6-dof";
  int_marker.header.frame_id = "/move_rotate_frame";

  visualization_msgs::InteractiveMarkerControl control;
  control.mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  saveMarker( int_marker );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_test");
  ros::NodeHandle n;

  ros::Publisher marker_pub;
  marker_pub = n.advertise<visualization_msgs::InteractiveMarkerArray> ("interactive_marker_array", 0, true);
//  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  srand( ros::Time::now().sec );

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  make6DofMarker( );

  ROS_INFO( "Publishing %d markers", (int)int_marker_array.markers.size() );
  marker_pub.publish( int_marker_array );

  ros::spin();
}
