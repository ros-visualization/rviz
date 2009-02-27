#include "ros/node.h"

#include "robot_msgs/VisualizationMarker.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::Node* node = new ros::Node( "MarkerTest", ros::Node::DONT_HANDLE_SIGINT );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<robot_msgs::VisualizationMarker>( "visualizationMarker", 0 );

  usleep( 1000000 );

#if 0
  for ( int i = -50; i < 50; ++i )
  {
    robot_msgs::VisualizationMarker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = robot_msgs::VisualizationMarker::ARROW;
    marker.action = 0;
    marker.x = 1;
    marker.y = (i*2);
    marker.z = 0;
    marker.yaw = 0.0;
    marker.pitch = 0.0;
    marker.roll = 0.0;
    marker.xScale = 0.5;
    marker.yScale = 0.2;
    marker.zScale = 0.2;
    marker.alpha = 255;
    marker.r = 0;
    marker.g = 255;
    marker.b = 0;
    node->publish( "visualizationMarker", marker );
  }

#endif

  int count = 10000;
  robot_msgs::VisualizationMarker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = robot_msgs::VisualizationMarker::LINE_LIST;
  marker.action = robot_msgs::VisualizationMarker::ADD;
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
    p1.z = 1;
    p2.x = -1;
    p2.y = (i - count/2);
    p2.z = 2;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }
  node->publish( "visualizationMarker", marker );

  robot_msgs::VisualizationMarker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = ros::Time();
  line_marker.id = count + 1;
  line_marker.type = robot_msgs::VisualizationMarker::LINE_STRIP;
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

  usleep( 1000000 );

  node->shutdown();
  delete node;


}
