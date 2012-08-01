#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void publishCallback(tf::TransformBroadcaster& tf_broadcaster, const ros::TimerEvent&)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/rviz_logo";
  marker.header.stamp = ros::Time(0);
  marker.ns = "mesh";
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 4;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0;
  marker.frame_locked = true;
  marker.mesh_resource = "package://rviz/image_src/RViz.stl";
  marker.mesh_use_embedded_materials = true;
  marker.id = 0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  g_marker_pub.publish(marker);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(3, 1, 0) );
  transform.setRotation( tf::Quaternion(0, 0, M_PI*0.9) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "rviz_logo"));

  /*
  tf::StampedTransform transform;
  transform.frame_id_="/rviz_logo";
  transform.child_frame_id_="/base_link";
  transform.stamp_=ros::Time::now();
  transform.setRotation( btQuaternion( btVector3(0,0,1), btScalar(90) ) );
  tf_broadcaster.sendTransform( transform );
  ROS_INFO("tf_broadcaster.sendTransform( transform );");
  */
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_logo_marker");
  ros::NodeHandle n;
  tf::TransformBroadcaster tf_broadcaster;

  g_marker_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker", 0);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), boost::bind(&publishCallback,tf_broadcaster,_1));

  ros::spin();
}
