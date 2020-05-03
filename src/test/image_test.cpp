#include "ros/ros.h"

#include "sensor_msgs/Image.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_test");

  ros::NodeHandle n;

  ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("red_image", 0);

  ros::Duration(0.1).sleep();

  sensor_msgs::Image red_image;
  red_image.header.frame_id = "base_link";
  red_image.header.stamp = ros::Time::now();
  red_image.height = 100;
  red_image.width = 100;
  red_image.encoding = "rgb8";
  red_image.step = 3 * red_image.height;

  red_image.data.resize(3 * red_image.height * red_image.width);
  for (uint32_t i = 0; i < 3 * red_image.height * red_image.width; ++i)
  {
    if (i % 3 == 0)
    {
      red_image.data[i] = 255;
    }
    else
    {
      red_image.data[i] = 0;
    }
  }

  int i = 0;
  while (n.ok())
  {
    ROS_INFO("Publishing");

    rgb_pub.publish(red_image);

    ++i;

    ros::Duration(1.0).sleep();
  }
}
