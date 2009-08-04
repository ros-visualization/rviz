#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

void emitRow(const std::string type_name, uint32_t type, int32_t x_pos, float r, float g, float b, ros::Duration lifetime, ros::Publisher& pub)
{
  for ( int i = -5; i < 5; ++i )
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "marker_test_" + type_name;
    marker.id = i;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = (i*2);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = lifetime;
    pub.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init( argc, argv, "marker_test" );
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher array_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  while (n.ok())
  {
    ROS_INFO("Publishing");
    int32_t x_pos = -15;
    emitRow("arrows", visualization_msgs::Marker::ARROW, x_pos, 1.0, 0.0, 0.0, ros::Duration(), marker_pub); x_pos += 3;
    emitRow("cubes", visualization_msgs::Marker::CUBE, x_pos, 0.0, 1.0, 0.0, ros::Duration(), marker_pub); x_pos += 3;
    emitRow("spheres", visualization_msgs::Marker::SPHERE, x_pos, 0.0, 0.0, 1.0, ros::Duration(), marker_pub); x_pos += 3;
    emitRow("cylinder", visualization_msgs::Marker::CYLINDER, x_pos, 1.0, 0.0, 0.0, ros::Duration(), marker_pub); x_pos += 3;
    emitRow("arrows_with_lifetime", visualization_msgs::Marker::ARROW, x_pos, 0.0, 1.0, 0.0, ros::Duration(0.6), marker_pub); x_pos += 3;
    emitRow("cubes_with_lifetime", visualization_msgs::Marker::CUBE, x_pos, 0.0, 0.0, 1.0, ros::Duration(0.7), marker_pub); x_pos += 3;
    emitRow("spheres_with_lifetime", visualization_msgs::Marker::SPHERE, x_pos, 1.0, 0.0, 0.0, ros::Duration(0.8), marker_pub); x_pos += 3;
    emitRow("cylinder_with_lifetime", visualization_msgs::Marker::CYLINDER, x_pos, 0.0, 1.0, 0.0, ros::Duration(0.9), marker_pub); x_pos += 3;

    {
      for ( int i = -5; i < 5; ++i )
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "marker_test_arrow_by_points";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = x_pos;
        marker.pose.position.y = i * 2;
        marker.scale.x = 0.25;
        marker.scale.y = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.points.resize(2);
        marker.points[0].x = 0.0f;
        marker.points[0].y = 0.0f;
        marker.points[0].z = 0.0f;
        marker.points[1].x = 1.0f;
        marker.points[1].y = 0.0f;
        marker.points[1].z = 0.0f;
        marker_pub.publish(marker);
      }
    }

    x_pos += 3;

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "marker_test_cube_list";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CUBE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = x_pos;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      for (int x = 0; x < 10; ++x)
      {
        for (int y = 0; y < 10; ++y)
        {
          for (int z = 0; z < 10; ++z)
          {
            geometry_msgs::Point p;
            p.x = x * 0.1f;
            p.y = y * 0.1f;
            p.z = z * 0.1f;

            marker.points.push_back(p);
          }
        }
      }
      marker_pub.publish(marker);
    }

    x_pos += 3;

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "marker_test_sphere_list";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = x_pos;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      for (int x = 0; x < 10; ++x)
      {
        for (int y = 0; y < 10; ++y)
        {
          for (int z = 0; z < 1; ++z)
          {
            geometry_msgs::Point p;
            p.x = x * 0.1f;
            p.y = y * 0.1f;
            p.z = z * 0.1f;

            marker.points.push_back(p);
          }
        }
      }
      marker_pub.publish(marker);
    }

    x_pos += 3;

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "marker_test_points";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = x_pos;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      for (int x = 0; x < 10; ++x)
      {
        for (int y = 0; y < 10; ++y)
        {
          for (int z = 0; z < 10; ++z)
          {
            geometry_msgs::Point p;
            p.x = x * 0.1f;
            p.y = y * 0.1f;
            p.z = z * 0.1f;

            marker.points.push_back(p);
          }
        }
      }
      marker_pub.publish(marker);
    }

    x_pos += 3;

    {
      int count = 10;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "marker_test_line_list";
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
      marker.pose.position.x = x_pos;
      marker.scale.x = 0.1;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      for ( int i = 0; i < count; ++i )
      {
        geometry_msgs::Point p1, p2;
        p1.x = 0;
        p1.y = (i - count/2)*2;
        p1.z = 0;
        p2.x = 0;
        p2.y = (i - count/2)*2;
        p2.z = 1;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
      marker_pub.publish(marker);
    }

    x_pos += 3;

    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time();
      marker.ns = "marker_test_line_strip";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.pose.position.x = x_pos;
      marker.scale.x = 0.1;
      marker.color.g = 1.0;
      marker.color.a = 1.0;
      for ( int i = -5; i < 5; ++i )
      {
        geometry_msgs::Point p;
        p.x = 1;
        p.y = (i*2);
        p.z = 0;
        marker.points.push_back(p);
      }

      marker_pub.publish(marker);
    }

    ros::Duration(1.0).sleep();
  }
}
