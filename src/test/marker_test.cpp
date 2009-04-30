#include "ros/node.h"

#include "visualization_msgs/VisualizationMarker.h"

#include <tf/transform_broadcaster.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::Node* node = new ros::Node( "MarkerTest", ros::Node::DONT_HANDLE_SIGINT );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<visualization_msgs::VisualizationMarker>( "visualizationMarker", 0 );

  tf::TransformBroadcaster tf_broadcaster(*node);

  usleep( 1000000 );

  ros::Time tm(ros::Time::now());
  tf::Transform t;
  t.setIdentity();
  //  tf_broadcaster.sendTransform(tf::Stamped<tf::Transform>(t, tm, "base", "map"));

#if 1
  for ( int i = -50; i < 50; ++i )
  {
    visualization_msgs::VisualizationMarker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::VisualizationMarker::ARROW;
    marker.action = 0;
    marker.x = 1;
    marker.y = (i*2);
    marker.z = 2;
    marker.yaw = 0.0;
    marker.pitch = 0.5;
    marker.roll = 0.0;
    marker.xScale = 0.2;
    marker.yScale = 0.2;
    marker.zScale = 0.2;
    marker.alpha = 255;
    marker.r = 0;
    marker.g = 255;
    marker.b = 0;
    node->publish( "visualizationMarker", marker );
  }

#else

  int count = 40000;
  visualization_msgs::VisualizationMarker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = visualization_msgs::VisualizationMarker::LINE_LIST;
  marker.action = visualization_msgs::VisualizationMarker::ADD;
  marker.x = 0;
  marker.y = 0;
  marker.z = 0;
  marker.yaw = 0.0;
  marker.pitch = 0.0;
  marker.roll = 0.0;
  marker.xScale = 0.1;
  marker.alpha = 255;
  marker.r = 255;
  marker.g = 0;
  marker.b = 0;
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
  node->publish( "visualizationMarker", marker );

  visualization_msgs::VisualizationMarker line_marker;
  line_marker.header.frame_id = "base_link";
  line_marker.header.stamp = ros::Time();
  line_marker.id = count + 1;
  line_marker.type = visualization_msgs::VisualizationMarker::LINE_STRIP;
  line_marker.action = 0;
  line_marker.x = 0;
  line_marker.y = 0;
  line_marker.z = 0;
  line_marker.yaw = 0.0;
  line_marker.pitch = 0.0;
  line_marker.roll = 0.0;
  line_marker.xScale = 0.05;
  line_marker.alpha = 100;
  line_marker.r = 0;
  line_marker.g = 255;
  line_marker.b = 0;
  for ( int i = -50; i < 50; ++i )
  {
    robot_msgs::Point p;
    p.x = 1;
    p.y = (i*2);
    p.z = 0;
    line_marker.points.push_back(p);
  }

  node->publish( "visualizationMarker", line_marker );
#endif

  usleep( 1000000 );

  node->shutdown();
  delete node;


}
