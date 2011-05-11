#include "ros/ros.h"

#include <visualization_msgs/InteractiveMarkerArray.h>
#include <visualization_msgs/interactive_marker_tools.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

visualization_msgs::InteractiveMarkerArray int_marker_array;

void frameCallback(const ros::TimerEvent&)
{
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;
  tf::Transform t;
  t.setOrigin(tf::Vector3(0.0, 0.0, (counter % 1000) * 0.01));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "my_link"));

  ++counter;
}

double rand( double min, double max )
{
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}


visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::Marker marker;

  marker.header = msg.header;
  marker.pose = msg.pose;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.size * 0.7;
  marker.scale.y = msg.size * 0.7;
  marker.scale.z = msg.size * 0.7;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::InteractiveMarkerControl makeBoxControl( visualization_msgs::InteractiveMarker &msg )
{
  visualization_msgs::InteractiveMarkerControl int_marker_control;
  int_marker_control.mode = int_marker_control.NONE;
  int_marker_control.always_visible = true;
  int_marker_control.markers.push_back( makeBox(msg) );

  return int_marker_control;
}


visualization_msgs::InteractiveMarker getEmptyMarker( bool dummyBox=true )
{
  std_msgs::Header header;
  header.frame_id = "/base_link";
  header.stamp = ros::Time();

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.y = -1.0 * int_marker_array.markers.size();
  pose.position.x = -1.0;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = header;
  int_marker.pose = pose;
  int_marker.size = 1.0;
  int_marker.frame_locked = false;

  int_marker_array.markers.push_back(int_marker);
  return *(int_marker_array.markers.end()-1);
}

void saveMarker( visualization_msgs::InteractiveMarker int_marker )
{
  int_marker.controls.push_back( makeBoxControl(int_marker));
  int_marker_array.markers.push_back(int_marker);

  for ( unsigned i=0; i<int_marker.controls.size()-1; i++ )
  {
    int_marker.controls[i].fixed_orientation = true;
  }

  for ( unsigned i=0; i<int_marker.controls.size(); i++ )
  {
    for ( unsigned j=0; j<int_marker.controls[i].markers.size(); j++ )
    {
      int_marker.controls[i].markers[j].pose.position.x += 2.0;
    }
  }
  int_marker.pose.position.x += 2.0;
  int_marker.name += " (fixed orientation)";

  int_marker_array.markers.push_back(int_marker);
}

void make6DofMarker( )
{
  visualization_msgs::InteractiveMarker int_marker = getEmptyMarker();
  int_marker.name = "6-dof";

  int_marker.controls.push_back( visualization_msgs::makeMoveAxisControl( int_marker,1,0,0) );
  int_marker.controls.push_back( visualization_msgs::makeMoveAxisControl( int_marker,0,1,0) );
  int_marker.controls.push_back( visualization_msgs::makeMoveAxisControl( int_marker,0,0,1) );

  int_marker.controls.push_back( visualization_msgs::makeRotatePlaneControl( int_marker,1,0,0) );
  int_marker.controls.push_back( visualization_msgs::makeRotatePlaneControl( int_marker,0,1,0) );
  int_marker.controls.push_back( visualization_msgs::makeRotatePlaneControl( int_marker,0,0,1) );

  saveMarker( int_marker );
}

void makeRandomMarker( )
{
  visualization_msgs::InteractiveMarker int_marker = getEmptyMarker();
  int_marker.name = "Random axes";

  for ( int i=0; i<5; i++ )
  {
    int_marker.controls.push_back( visualization_msgs::makeMoveAxisControl( int_marker, rand(-1.0,1.0), rand(-1.0,1.0), rand(-1.0,1.0) ) );
    int_marker.controls.push_back( visualization_msgs::makeRotatePlaneControl( int_marker, rand(-1.0,1.0), rand(-1.0,1.0), rand(-1.0,1.0) ) );
  }

  saveMarker( int_marker );
}

void makeMovePlaneMarker()
{
  visualization_msgs::InteractiveMarker int_marker = getEmptyMarker();
  int_marker.name = "move in plane";

  int_marker.controls.push_back( visualization_msgs::makeMovePlaneControl( int_marker,0,0,1) );
  int_marker.controls.push_back( makeBoxControl(int_marker));

  saveMarker( int_marker );
}

void makeMoveRotatePlaneMarker()
{
  visualization_msgs::InteractiveMarker int_marker = getEmptyMarker();

  int_marker.name = "Move & rotate";

  int_marker.controls.push_back( visualization_msgs::makeMoveRotatePlaneControl( int_marker,0,0,1) );

  int_marker.controls.push_back( makeBoxControl(int_marker));

  saveMarker( int_marker );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_test");
  ros::NodeHandle n;

  ros::Publisher marker_pub;
  marker_pub = n.advertise<visualization_msgs::InteractiveMarkerArray> ("interactive_marker_array", 0, true);
//  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  srand( ros::Time::now().sec );

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  make6DofMarker( );
  makeRandomMarker( );
  makeMovePlaneMarker( );
  makeMoveRotatePlaneMarker( );

  ROS_INFO( "Publishing %d markers", (int)int_marker_array.markers.size() );
  marker_pub.publish( int_marker_array );

  ros::spin();
}
