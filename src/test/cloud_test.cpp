#include "ros/node.h"

#include "robot_msgs/PointCloud.h"

#include <tf/transform_broadcaster.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv );

  ros::Node* node = new ros::Node( "RGBCloudTest" );

  while ( !node->ok() )
  {
    usleep( 10000 );
  }

  node->advertise<robot_msgs::PointCloud>( "rgb_cloud_test", 0 );
  node->advertise<robot_msgs::PointCloud>( "rgb_cloud_test2", 0 );
  node->advertise<robot_msgs::PointCloud>( "intensity_cloud_test", 0 );
  node->advertise<robot_msgs::PointCloud>( "million_points_cloud_test", 0 );

  tf::TransformBroadcaster tf_broadcaster(*node);

  usleep( 1000000 );

  int i = 0;
  while (node->ok())
  {
    ros::Time tm(ros::Time::now());
    tf::Transform t;
    t.setIdentity();
    //    tf_broadcaster.sendTransform(tf::Stamped<tf::Transform>(t, tm, "base", "map"));

    {
      robot_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "base_link";

      cloud.pts.resize(100000);
      cloud.chan.resize(1);
      cloud.chan[0].name = "intensities";
      cloud.chan[0].vals.resize(100000);
      for ( int j = 0; j < 100000; ++j )
      {
        cloud.pts[j].x = j;
        cloud.pts[j].y = 3.0f;
        cloud.pts[j].z = i % 10;

        cloud.chan[0].vals[j] = 1000.0f;
      }

      node->publish( "million_points_cloud_test", cloud );
    }

    {
      robot_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "base_link";

      cloud.pts.resize(5);
      cloud.chan.resize(1);
      for ( int j = 0; j < 5; ++j )
      {
        cloud.pts[j].x = (float)j;
        cloud.pts[j].y = 0.0f;
        cloud.pts[j].z = i % 100;
      }

      cloud.chan[0].name = "rgb";
      cloud.chan[0].vals.resize(5);

      int rgb = (0xff << 16);
      cloud.chan[0].vals[0] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8);
      cloud.chan[0].vals[1] = *reinterpret_cast<float*>(&rgb);
      rgb = 0xff;
      cloud.chan[0].vals[2] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 16) | (0xff << 8);
      cloud.chan[0].vals[3] = *reinterpret_cast<float*>(&rgb);
      rgb = (0xff << 8) | 0xff;
      cloud.chan[0].vals[4] = *reinterpret_cast<float*>(&rgb);

      node->publish( "rgb_cloud_test", cloud );
    }

    {
      robot_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "base_link";

      cloud.pts.resize(5);
      cloud.chan.resize(3);
      for ( int i = 0; i < 5; ++i )
      {
        cloud.pts[i].x = (float)i;
        cloud.pts[i].y = 1.0f;
        cloud.pts[i].z = 0.0f;
      }

      cloud.chan[0].name = "r";
      cloud.chan[0].vals.resize(5);
      cloud.chan[1].name = "g";
      cloud.chan[1].vals.resize(5);
      cloud.chan[2].name = "b";
      cloud.chan[2].vals.resize(5);

      cloud.chan[0].vals[0] = 1.0f;
      cloud.chan[1].vals[0] = 0.0f;
      cloud.chan[2].vals[0] = 0.0f;

      cloud.chan[0].vals[1] = 0.0f;
      cloud.chan[1].vals[1] = 1.0f;
      cloud.chan[2].vals[1] = 0.0f;

      cloud.chan[0].vals[2] = 0.0f;
      cloud.chan[1].vals[2] = 0.0f;
      cloud.chan[2].vals[2] = 1.0f;

      cloud.chan[0].vals[3] = 1.0f;
      cloud.chan[1].vals[3] = 1.0f;
      cloud.chan[2].vals[3] = 0.0f;

      cloud.chan[0].vals[4] = 0.0f;
      cloud.chan[1].vals[4] = 1.0f;
      cloud.chan[2].vals[4] = 1.0f;

      node->publish( "rgb_cloud_test2", cloud );
    }

    {
      robot_msgs::PointCloud cloud;
      cloud.header.stamp = tm;
      cloud.header.frame_id = "base_link";

      int num_rows = 1;
      int num_cols = 200;
      int total_pts = num_rows * num_cols;
      cloud.pts.resize(total_pts);
      cloud.chan.resize(1);
      cloud.chan[0].vals.resize(total_pts);
      cloud.chan[0].name = "intensities";

      int j = 0;
      for (int z = 0; z < num_rows; ++z)
      {
        for (int x = 0; x < num_cols; ++x, ++j)
        {
          cloud.pts[j].x = x;
          cloud.pts[j].y = 0;

          if (num_rows == 1)
          {
            cloud.pts[j].z = i % 100;
          }
          else
          {
            cloud.pts[j].z = z;
          }

          cloud.chan[0].vals[j] = j;
        }
      }

      node->publish( "intensity_cloud_test", cloud );
    }

    ++i;

    usleep( 100000 );
  }

  node->shutdown();
  delete node;


}
