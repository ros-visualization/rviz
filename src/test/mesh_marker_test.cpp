#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void publishText(int& id, float x, float y, const std::string& text)
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
  marker.scale.z = 0.2;
  marker.text = text;
  marker.id = ++id;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  g_marker_pub.publish(marker);
}

void publishMesh(int& id,
                 float x,
                 float y,
                 float r,
                 float g,
                 float b,
                 float a,
                 bool use_embedded_materials,
                 int mesh)
{
  using visualization_msgs::Marker;

  Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mesh";
  switch (mesh)
  {
  case 0:
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://rviz/src/test/meshes/pr2-base.dae";
    marker.mesh_use_embedded_materials = use_embedded_materials;
    break;
  case 1:
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://rviz/src/test/meshes/frame.dae";
    marker.mesh_use_embedded_materials = use_embedded_materials;
    break;
  case 2:
    marker.type = Marker::SPHERE;
    break;
  }
  marker.action = Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0995;
  marker.pose.orientation.w = 0.995;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.frame_locked = true;
  marker.id = ++id;
  marker.pose.position.x = x;
  marker.pose.position.y = y;

  g_marker_pub.publish(marker);
}

void publishMeshes(int& id, float x, float r, float g, float b, float a)
{
  int y = -5;
  char buffer[30];
  snprintf(buffer, sizeof(buffer), "rbg: %.1f %.1f %.1f, a: %.1f", r, g, b, a);
  publishText(id, x, y - 1, buffer);

  publishMesh(id, x, y++, r, g, b, a, true, 0);
  publishMesh(id, x, y++, r, g, b, a, false, 0);

  publishMesh(id, x, y++, r, g, b, a, true, 1);
  publishMesh(id, x, y++, r, g, b, a, false, 1);

  publishMesh(id, x, y++, r, g, b, a, true, 2);
  publishMesh(id, x, y++, r, g, b, a, false, 2);
}

void publishMeshes()
{
  ROS_INFO("Publishing");

  int id = 0;
  float x = -6;
  float y = -5;

  // column headers
  publishText(id, x, y++, "mesh1, use_embedded_materials = true");
  publishText(id, x, y++, "mesh1, use_embedded_materials = false");
  publishText(id, x, y++, "mesh2, use_embedded_materials = true");
  publishText(id, x, y++, "mesh2, use_embedded_materials = false");
  publishText(id, x, y++, "sphere, use_embedded_materials = true");
  publishText(id, x, y++, "sphere, use_embedded_materials = false");

  publishMeshes(id, ++x, 0, 0, 0, 0);

  publishMeshes(id, ++x, 1, 1, 1, 1);
  publishMeshes(id, ++x, 1, 1, 1, 0.5);
  publishMeshes(id, ++x, 1, 1, 1, 0.0);

  publishMeshes(id, ++x, 1, 0, 0, 1);
  publishMeshes(id, ++x, 1, 0, 0, 0.5);
  publishMeshes(id, ++x, 1, 0, 0, 0);

  publishMeshes(id, ++x, 1, .5, .5, 1);
  publishMeshes(id, ++x, 1, .5, .5, 0.5);
  publishMeshes(id, ++x, 1, .5, .5, 0);

  static float rgba[4] = {0, 0, 0, 0};
  publishMeshes(id, ++x, rgba[0], rgba[1], rgba[2], rgba[3]);
  // evolve colors over time
  int index = 3;
  while (index >= 0)
  {
    rgba[index] += 0.2;
    if (rgba[index] > 1.01)
      rgba[index--] = 0.0f;
    else
      break;
  }
}

void publishCallback(const ros::TimerEvent& /*unused*/)
{
  return publishMeshes();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_marker_test");
  ros::NodeHandle n;

  bool latch_only = false;
  if (argc == 2)
  {
    if (std::string(argv[1]) == "--latch-only")
    {
      latch_only = true;
    }
  }

  g_marker_pub = n.advertise<visualization_msgs::Marker>("mesh_markers", 0, /*latch*/ latch_only);


  ros::Timer publish_timer;
  if (latch_only)
  {
    ros::Duration(1.0).sleep();
    publishMeshes();
  }
  else
  {
    publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  }

  ros::spin();
}
