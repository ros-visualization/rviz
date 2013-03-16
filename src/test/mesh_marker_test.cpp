#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void publishMesh( int id, float x, float y, float r, float g, float b, float a, bool use_embedded_materials, bool mesh )
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mesh";
  marker.type = mesh ? (int) visualization_msgs::Marker::MESH_RESOURCE : (int) visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.1;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.frame_locked = true;
  marker.mesh_resource = "package://rviz/src/test/meshes/pr2-base.dae";
  marker.mesh_use_embedded_materials = use_embedded_materials;
  marker.id = id;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  g_marker_pub.publish(marker);
}

void publishCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  ROS_INFO("Publishing");

  int id = 0;
  float x = 0;
  float y = 0;

  publishMesh( id, x, y, 1, 1, 1, 1, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, .5, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, 0, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 1, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, .5, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 0, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 1, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, .5, true, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 0, true, true);
  id++; x++;

  y++; x = 0;

  publishMesh( id, x, y, 1, 1, 1, 1, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, .5, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, 0, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 1, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, .5, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 0, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 1, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, .5, false, true);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 0, false, true);
  id++; x++;

  y++; x = 0;

  publishMesh( id, x, y, 1, 1, 1, 1, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, .5, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, 1, 1, 0, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 1, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, .5, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, 0, 0, 0, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 1, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, .5, true, false);
  id++; x++;
  publishMesh( id, x, y, 1, .5, .5, 0, true, false);
  id++; x++;

  y++; x = 0;

  publishMesh( id, x, y, 0, 0, 0, 0, true, true);
  id++; x++;

  ++counter;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_marker_test");
  ros::NodeHandle n;

  g_marker_pub = n.advertise<visualization_msgs::Marker> ("mesh_markers", 0);
  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);

  ros::Duration(0.1).sleep();

  ros::spin();
}
