#include "ros/ros.h"

#include <limits>

#include "sensor_msgs/PointCloud.h"

#include <tf/transform_broadcaster.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv, "cloud_test" );

  ros::NodeHandle n;

  ros::Publisher rgb_pub = n.advertise<sensor_msgs::PointCloud>( "rgb_cloud_test", 0 );
  ros::Publisher rgb2_pub = n.advertise<sensor_msgs::PointCloud>( "rgb_cloud_test2", 0 );
  ros::Publisher intensity_pub = n.advertise<sensor_msgs::PointCloud>( "intensity_cloud_test", 0 );
  ros::Publisher million_pub = n.advertise<sensor_msgs::PointCloud>( "million_points_cloud_test", 0 );
  ros::Publisher changing_channels_pub = n.advertise<sensor_msgs::PointCloud>( "changing_channels_test", 0 );

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  int i = 0;
  while (n.ok())
  {
    ros::Time tm(ros::Time::now());
    tf::Transform t;
    t.setIdentity();
    //    tf_broadcaster.sendTransform(tf::Stamped<tf::Transform>(t, tm, "base", "map"));

    ROS_INFO("Publishing");

    sensor_msgs::PointCloud changing_cloud;

    {
      static sensor_msgs::PointCloud cloud;

      if (cloud.channels.empty())
      {
        cloud.header.stamp = tm;
        cloud.header.frame_id = "/base_link";

        cloud.channels.resize(1);
        int32_t xcount = 100;
        int32_t ycount = 100;
        int32_t zcount = 100;
        int32_t total = xcount * ycount * zcount;
        cloud.points.resize(total);
        cloud.channels[0].values.resize(total);
        cloud.channels[0].name = "intensities";
        float factor = 0.1f;
        for (int32_t x = 0; x < xcount; ++x)
        {
          for (int32_t y = 0; y < ycount; ++y)
          {
            for (int32_t z = 0; z < zcount; ++z)
            {
              int32_t index = (ycount*zcount*x) + zcount*y + z;
              geometry_msgs::Point32& point = cloud.points[index];
              point.x = x * factor;
              point.y = y * factor;
              point.z = z * factor;

              cloud.channels[0].values[index] = (index % 4096);
            }
          }
        }
      }

      million_pub.publish( cloud );
    }

    {
      sensor_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "/base_link";

      cloud.points.resize(5);
      cloud.channels.resize(2);
      for ( int j = 0; j < 5; ++j )
      {
        cloud.points[j].x = (float)j;
        cloud.points[j].y = 0.0f;
        cloud.points[j].z = i % 10;

        if (j == 2)
        {
          cloud.points[j].z = std::numeric_limits<float>::quiet_NaN();
        }
      }

      cloud.channels[0].name = "rgb";
      cloud.channels[0].values.resize(5);

      int rgb = (0xff << 16);
      cloud.channels[0].values[0] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8);
      cloud.channels[0].values[1] = *reinterpret_cast<float*>(&rgb);
      rgb = 0xff;
      cloud.channels[0].values[2] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 16) | (0xff << 8);
      cloud.channels[0].values[3] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8) | 0xff;
      cloud.channels[0].values[4] = *reinterpret_cast<float*>(&rgb);

      cloud.channels[1].name = "intensity";
      cloud.channels[1].values.resize(5);
      cloud.channels[1].values[0] = 0;
      cloud.channels[1].values[1] = 100;
      cloud.channels[1].values[2] = 200;
      cloud.channels[1].values[3] = 300;
      cloud.channels[1].values[4] = 400;

      rgb_pub.publish(cloud);
    }

    {
      sensor_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "/base_link";

      cloud.points.resize(5);
      cloud.channels.resize(3);
      for ( int j = 0; j < 5; ++j )
      {
        cloud.points[j].x = (float)j;
        cloud.points[j].y = 1.0f;
        cloud.points[j].z = i % 10;
      }

      cloud.channels[0].name = "r";
      cloud.channels[0].values.resize(5);
      cloud.channels[1].name = "g";
      cloud.channels[1].values.resize(5);
      cloud.channels[2].name = "b";
      cloud.channels[2].values.resize(5);

      cloud.channels[0].values[0] = 1.0f;
      cloud.channels[1].values[0] = 0.0f;
      cloud.channels[2].values[0] = 0.0f;

      cloud.channels[0].values[1] = 0.0f;
      cloud.channels[1].values[1] = 1.0f;
      cloud.channels[2].values[1] = 0.0f;

      cloud.channels[0].values[2] = 0.0f;
      cloud.channels[1].values[2] = 0.0f;
      cloud.channels[2].values[2] = 1.0f;

      cloud.channels[0].values[3] = 1.0f;
      cloud.channels[1].values[3] = 1.0f;
      cloud.channels[2].values[3] = 0.0f;

      cloud.channels[0].values[4] = 0.0f;
      cloud.channels[1].values[4] = 1.0f;
      cloud.channels[2].values[4] = 1.0f;

      rgb2_pub.publish(cloud);

      if ((i % 10) - 5 < 0)
      {
        changing_cloud = cloud;
      }
    }

    {
      sensor_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "/base_link";

      int num_rows = 1;
      int num_cols = 200;
      int total_pts = num_rows * num_cols;
      cloud.points.resize(total_pts);
      cloud.channels.resize(1);
      cloud.channels[0].values.resize(total_pts);
      cloud.channels[0].name = "intensities";

      int j = 0;
      for (int z = 0; z < num_rows; ++z)
      {
        for (int x = 0; x < num_cols; ++x, ++j)
        {
          cloud.points[j].x = x;
          cloud.points[j].y = 0;

          if (num_rows == 1)
          {
            cloud.points[j].z = i % 10;
          }
          else
          {
            cloud.points[j].z = z;
          }

          cloud.channels[0].values[j] = j;
        }
      }

      intensity_pub.publish(cloud);

      if ((i % 10) - 5 >= 0)
      {
        changing_cloud = cloud;
      }
    }

    changing_channels_pub.publish(changing_cloud);

    ++i;

    ros::Duration(1.0).sleep();
  }
}
