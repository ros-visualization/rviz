#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void
publishText(int id, float x, float y, const std::string & text)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mesh";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.text = text;
  marker.id = id;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  g_marker_pub.publish(marker);
}

void
publishMesh(
  int id, float x, float y,
  float r, float g, float b, float a,
  bool use_embedded_materials, bool mesh)
{
  using visualization_msgs::Marker;

  Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mesh";
  if (mesh) {
    marker.type = Marker::MESH_RESOURCE;
  } else {
    marker.type = Marker::SPHERE;
  }
  marker.action = Marker::ADD;
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

void
publishMeshes()
{
  ROS_INFO("Publishing");

  int id = 0;
  float x = 0;
  float y = 0;

  publishText(id, x - 1, y, "mesh, use_embedded_materials = true");
  id++;
  publishText(id, x, y - 1, "rbg: 0.0 0.0 0.0, a: 0");
  id++;
  publishMesh(id, x, y, 0, 0, 0, 0, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 1.0 1.0, a: 1");
  id++;
  publishMesh(id, x, y, 1, 1, 1, 1, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 1.0 1.0, a: 0.5");
  id++;
  publishMesh(id, x, y, 1, 1, 1, .5, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 1.0 1.0, a: 0");
  id++;
  publishMesh(id, x, y, 1, 1, 1, 0, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.0 0.0, a: 1");
  id++;
  publishMesh(id, x, y, 1, 0, 0, 1, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.0 0.0, a: 0.5");
  id++;
  publishMesh(id, x, y, 1, 0, 0, .5, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.0 0.0, a: 0");
  id++;
  publishMesh(id, x, y, 1, 0, 0, 0, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.5 0.5, a: 1");
  id++;
  publishMesh(id, x, y, 1, .5, .5, 1, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.5 0.5, a: 0.5");
  id++;
  publishMesh(id, x, y, 1, .5, .5, .5, true, true);
  id++; x++;
  publishText(id, x, y - 1, "rbg: 1.0 0.5 0.5, a: 0");
  id++;
  publishMesh(id, x, y, 1, .5, .5, 0, true, true);
  id++; x++;

  y++; x = 0;

  publishText(id, x - 1, y, "mesh, use_embedded_materials = false");
  id++;
  publishMesh(id, x, y, 0, 0, 0, 0, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 1, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, .5, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 0, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 1, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, .5, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 0, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 1, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, .5, false, true);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 0, false, true);
  id++; x++;

  y++; x = 0;

  publishText(id, x - 1, y, "sphere, use_embedded_materials = true");
  id++;
  publishMesh(id, x, y, 0, 0, 0, 0, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 1, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, .5, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 0, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 1, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, .5, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 0, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 1, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, .5, true, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 0, true, false);
  id++; x++;

  y++; x = 0;

  publishText(id, x - 1, y, "sphere, use_embedded_materials = false");
  id++;
  publishMesh(id, x, y, 0, 0, 0, 0, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 1, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, .5, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 1, 1, 0, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 1, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, .5, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, 0, 0, 0, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 1, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, .5, false, false);
  id++; x++;
  publishMesh(id, x, y, 1, .5, .5, 0, false, false);
  id++; x++;
}

void publishCallback(const ros::TimerEvent&)
{
  return publishMeshes();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_marker_test");
  ros::NodeHandle n;

  bool latch_only = false;
  if (argc == 2) {
    if (std::string(argv[1]) == "--latch-only") {
      latch_only = true;
    }
  }

  g_marker_pub = n.advertise<visualization_msgs::Marker>("mesh_markers", 0, /*latch*/ latch_only);


  ros::Timer publish_timer;
  if (latch_only) {
    ros::Duration(1.0).sleep();
    publishMeshes();
  } else {
    publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  }

  ros::spin();
}
