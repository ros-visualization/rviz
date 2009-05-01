#include "ros/node.h"

#include "visualization_msgs/Marker.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::Node* node = new ros::Node( "marker_test", ros::Node::DONT_HANDLE_SIGINT );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  tf::TransformBroadcaster tf_broadcaster(*node);

  usleep( 1000000 );

  ros::Time tm(ros::Time::now());
  tf::Transform t;
  t.setIdentity();
  //  tf_broadcaster.sendTransform(tf::Stamped<tf::Transform>(t, tm, "base", "map"));

#if 1
  for ( int i = -50; i < 50; ++i )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "marker_test";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = (i*2);
    marker.pose.position.z = 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    node->publish( "visualization_marker", marker );
  }

#else

  int count = 40000;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.ns = "marker_test";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  for ( int i = 0; i < count; ++i )
  {
    robot_msgs::Point p1, p2;
    p1.x = -1;
    p1.y = (i - count/2);
    p1.z = 0;
    p2.x = -1;
    p2.y = (i - count/2);
    p2.z = 1;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  node->publish( "visualization_marker", marker );

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "base_link";
  line_marker.header.stamp = ros::Time();
  line_marker.ns = "marker_test";
  line_marker.id = count + 1;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = 0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  for ( int i = -50; i < 50; ++i )
  {
    robot_msgs::Point p;
    p.x = 1;
    p.y = (i*2);
    p.z = 0;
    line_marker.points.push_back(p);
  }

  node->publish( "visualization_marker", line_marker );
#endif

  usleep( 1000000 );

  node->shutdown();
  delete node;


}
