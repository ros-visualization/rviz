#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void emitRow(const std::string type_name,
             uint32_t type,
             int32_t x_pos,
             float r,
             float g,
             float b,
             ros::Duration lifetime,
             ros::Publisher& pub,
             bool frame_locked = true,
             std::string frame_id = std::string("/base_link"),
             float sx = 1.0,
             float sy = 1.0,
             float sz = 1.0)
{
  static uint32_t count = 0;
  for (int i = -5; i <= 5; ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    ros::Time ros_time = ros::Time::now();
    //    ros_time.sec -=1;
    marker.header.stamp = ros_time;
    marker.ns = "marker_test_" + type_name;
    marker.id = i;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_pos;
    marker.pose.position.y = (i * 2);
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = float(i + 5) / 10.0;

    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    if (type == visualization_msgs::Marker::TEXT_VIEW_FACING)
    {
      marker.text = "This is some text\nthis is a new line\nthis is another line\nand another with utf8 "
                    "symbols: äöüÄÖÜ\na really really really really really really really really really "
                    "really long one";
      marker.scale.x = marker.scale.y = 0.0;
    }
    else if (type == visualization_msgs::Marker::POINTS)
    {
      marker.scale.z = 0.0;
    }
    else if (type == visualization_msgs::Marker::MESH_RESOURCE)
    {
      marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
      marker.mesh_use_embedded_materials = (i > int((count / 12) % 5));
    }
    pub.publish(marker);
  }

  ++count;
}

void publishCallback(const ros::TimerEvent& /*unused*/)
{
  static uint32_t counter = 0;

  ROS_INFO("Publishing");
  int32_t x_pos = -15;
  emitRow("arrows", visualization_msgs::Marker::ARROW, x_pos, 1.0, 0.0, 0.0, ros::Duration(),
          g_marker_pub);
  x_pos += 3;
  emitRow("cubes", visualization_msgs::Marker::CUBE, x_pos, 0.0, 1.0, 0.0, ros::Duration(), g_marker_pub);
  x_pos += 3;
  emitRow("cubes_frame_locked", visualization_msgs::Marker::CUBE, x_pos, 1.0, 1.0, 0.0, ros::Duration(),
          g_marker_pub, true, "/my_link");
  x_pos += 3;
  emitRow("spheres", visualization_msgs::Marker::SPHERE, x_pos, 0.0, 0.0, 1.0, ros::Duration(),
          g_marker_pub);
  x_pos += 3;
  emitRow("cylinder", visualization_msgs::Marker::CYLINDER, x_pos, 1.0, 0.0, 0.0, ros::Duration(),
          g_marker_pub);
  x_pos += 3;
  emitRow("arrows_with_lifetime", visualization_msgs::Marker::ARROW, x_pos, 0.0, 1.0, 0.0,
          ros::Duration(0.6), g_marker_pub);
  x_pos += 3;
  emitRow("cubes_with_lifetime", visualization_msgs::Marker::CUBE, x_pos, 0.0, 0.0, 1.0,
          ros::Duration(0.7), g_marker_pub);
  x_pos += 3;
  emitRow("spheres_with_lifetime", visualization_msgs::Marker::SPHERE, x_pos, 1.0, 0.0, 0.0,
          ros::Duration(0.8), g_marker_pub);
  x_pos += 3;
  emitRow("cylinder_with_lifetime", visualization_msgs::Marker::CYLINDER, x_pos, 0.0, 1.0, 0.0,
          ros::Duration(0.9), g_marker_pub);
  x_pos += 3;
  emitRow("text_view_facing", visualization_msgs::Marker::TEXT_VIEW_FACING, x_pos, 1.0, 1.0, 1.0,
          ros::Duration(), g_marker_pub, false, "/base_link", 1.0, 1.0, 0.2);
  x_pos += 3;
  emitRow("mesh_resource", visualization_msgs::Marker::MESH_RESOURCE, x_pos, 0.0, 1.0, 1.0,
          ros::Duration(), g_marker_pub);
  x_pos += 3;

  emitRow("invalid_scales", visualization_msgs::Marker::CUBE, x_pos, 0.0, 1.0, 1.0, ros::Duration(),
          g_marker_pub, false, "/base_link", 0.0, 1.0, 1.0);
  x_pos += 3;

  {
    for (int i = -5; i <= 5; ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/base_link";
      marker.header.stamp = ros::Time::now();
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
      marker.frame_locked = true;

      if (counter % 2 == 0)
      {
        marker.points.resize(0);
      }
      else
      {
        marker.points.resize(2);
        marker.points[0].x = 0.0f;
        marker.points[0].y = 0.0f;
        marker.points[0].z = 0.0f;
        marker.points[1].x = 1.0f;
        marker.points[1].y = 0.0f;
        marker.points[1].z = 0.0f;
      }
      g_marker_pub.publish(marker);
    }
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
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
    marker.frame_locked = true;

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
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_cube_list_color_per";
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
    marker.frame_locked = true;

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

          std_msgs::ColorRGBA c;
          c.r = x * 0.1;
          c.g = y * 0.1;
          c.b = z * 0.1;
          c.a = 1.0;
          marker.colors.push_back(c);
        }
      }
    }
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_point_list_alpha_per";
    marker.id = 0;
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
    marker.frame_locked = true;

    for (int type = visualization_msgs::Marker::CUBE_LIST; type <= visualization_msgs::Marker::POINTS;
         type++)
    {
      marker.id = type;
      marker.pose.position.x += 0.5;
      marker.type = type;
      if (type == visualization_msgs::Marker::POINTS)
        marker.scale.z = 0.0;

      for (int y = 0; y < 10; ++y)
      {
        geometry_msgs::Point p;
        p.x = 0;
        p.y = y * 0.1f;
        p.z = 0;

        marker.points.push_back(p);

        std_msgs::ColorRGBA c;
        c.r = 1;
        c.g = 1;
        c.b = 1;
        c.a = (float)y * 0.1 + 0.1;
        marker.colors.push_back(c);
      }
      g_marker_pub.publish(marker);
    }
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
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
    marker.frame_locked = true;

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

          std_msgs::ColorRGBA c;
          c.r = x * 0.1;
          c.g = y * 0.1;
          c.b = 0.5;
          c.a = 1.0;
          marker.colors.push_back(c);
        }
      }
    }
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = x_pos;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = true;

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
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_points_color_per";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = x_pos;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = true;

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

          std_msgs::ColorRGBA c;
          c.r = x * 0.1;
          c.g = y * 0.1;
          c.b = z * 0.1;
          c.a = 1.0;
          marker.colors.push_back(c);
        }
      }
    }
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    int count = 10;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
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
    marker.frame_locked = true;

    for (int i = 0; i <= count; ++i)
    {
      geometry_msgs::Point p1, p2;
      p1.x = 0;
      p1.y = (i - count / 2) * 2;
      p1.z = 0;
      p2.x = 0;
      p2.y = (i - count / 2) * 2;
      p2.z = 1;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    int count = 10;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_line_list_color_per";
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
    marker.frame_locked = true;

    for (int i = 0; i <= count; ++i)
    {
      geometry_msgs::Point p1, p2;
      p1.x = 0;
      p1.y = (i - count / 2) * 2;
      p1.z = 0;
      p2.x = 0;
      p2.y = (i - count / 2) * 2;
      p2.z = 1;
      marker.points.push_back(p1);
      marker.points.push_back(p2);

      std_msgs::ColorRGBA c;
      float pct = (float)i / (float)count;
      c.r = pct * 1.0 + (1 - pct) * 0.0;
      c.g = pct * 0.0 + (1 - pct) * 0.0;
      c.b = pct * 0.0 + (1 - pct) * 1.0;
      c.a = 1.0;

      marker.colors.push_back(c);
      marker.colors.push_back(c);
    }
    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
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
    marker.frame_locked = true;

    for (int i = -5; i <= 5; ++i)
    {
      geometry_msgs::Point p;
      p.x = 1 + (i % 2);
      p.y = (i * 2);
      p.z = 0;
      marker.points.push_back(p);
    }

    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_line_strip_color_per";
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
    marker.frame_locked = true;

    for (int i = -5; i < 5; ++i)
    {
      geometry_msgs::Point p;
      p.x = 1 + (i % 2);
      p.y = (i * 2);
      p.z = 0;
      marker.points.push_back(p);

      std_msgs::ColorRGBA c;
      float pct = (i + 5) / 10.0;
      c.r = pct * 0.0 + (1 - pct) * 0.0;
      c.g = pct * 1.0 + (1 - pct) * 0.0;
      c.b = pct * 0.0 + (1 - pct) * 1.0;
      c.a = 1.0;

      marker.colors.push_back(c);
    }

    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_triangle_list";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = x_pos;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    marker.frame_locked = true;

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

          geometry_msgs::Point p2 = p;
          p2.x = p.x + 0.05;

          geometry_msgs::Point p3 = p;
          p3.x = p2.x;
          p3.z = p.z + 0.05;

          marker.points.push_back(p);
          marker.points.push_back(p2);
          marker.points.push_back(p3);

          std_msgs::ColorRGBA c;
          c.r = x * 0.1;
          c.g = y * 0.1;
          c.b = z * 0.1;
          c.a = 1.0;
          marker.colors.push_back(c);
          marker.colors.push_back(c);
          marker.colors.push_back(c);
        }
      }
    }

    g_marker_pub.publish(marker);
  }

  x_pos += 3;

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_test_mesh_color_change";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = x_pos;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = float(counter % 255) / 255;
    marker.color.g = float((counter * 3) % 255) / 255;
    marker.color.b = float((counter * 10) % 255) / 255;
    marker.color.a = 1.0;
    marker.frame_locked = true;
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.mesh_use_embedded_materials = false;

    g_marker_pub.publish(marker);
  }

  ++counter;
}

void frameCallback(const ros::TimerEvent& /*unused*/)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;
  tf::Transform t;

  t.setOrigin(tf::Vector3(0.0, 0.0, (counter % 1000) * 0.01));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "my_link"));

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(M_PI * 0.25, M_PI * 0.25, 0.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "rotate_base_link", "base_link"));

  ++counter;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;

  g_marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  ros::spin();
}
