#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "interactive_markers/interactive_marker_server.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

interactive_markers::InteractiveMarkerServer* server;

void makeMarker()
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "rviz_logo";
  int_marker.name = "R";

  int_marker.pose.orientation.x = 0.0;
  int_marker.pose.orientation.y = 0.0;
  int_marker.pose.orientation.z = 1.0;
  int_marker.pose.orientation.w = 1.0;
  int_marker.pose.position.y = -2.0;
  int_marker.scale = 2.3;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;

  marker.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 0.2));

  marker.pose.position.x = 0;
  marker.pose.position.y = -0.22;
  marker.pose.position.z = 0;
  marker.scale.x = marker.scale.y = marker.scale.z = 4;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0;
  marker.mesh_resource = "package://rviz/image_src/R.stl";
  marker.mesh_use_embedded_materials = true;
  marker.id = 0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(marker);

  // add the control to the interactive marker
  int_marker.controls.push_back(control);

  visualization_msgs::InteractiveMarkerControl linear_control;
  linear_control.name = "rotate_z";
  linear_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  linear_control.orientation.y = 1;
  linear_control.orientation.w = 1;

  // add the control to the interactive marker
  int_marker.controls.push_back(linear_control);

  server->insert(int_marker);

  int_marker.name = "Viz";

  marker.mesh_resource = "package://rviz/image_src/Viz.stl";
  marker.pose.position.x = 3.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  control.markers.clear();
  control.markers.push_back(marker);

  // add the control to the interactive marker
  int_marker.controls.clear();
  int_marker.controls.push_back(control);

  server->insert(int_marker);


  server->applyChanges();
}

void publishCallback(const ros::TimerEvent& /*unused*/)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;
  transform.transform.translation = tf2::toMsg(tf2::Vector3(3, 1, 0));
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI * 0.9);
  transform.transform.rotation = tf2::toMsg(q);
  transform.header.frame_id = "base_link";
  transform.header.stamp = ros::Time::now();
  transform.child_frame_id = "rviz_logo";
  br.sendTransform(transform);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_logo_marker");
  ros::NodeHandle n;

  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), boost::bind(&publishCallback, _1));

  server = new interactive_markers::InteractiveMarkerServer("rviz_logo");
  makeMarker();

  ros::spin();
}
