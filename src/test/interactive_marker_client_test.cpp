/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker_client.h>

class MockReceiver: public rviz::InteractiveMarkerReceiver
{
public:
  std::map<std::string, double> marks_;
//  std::map<std::string, std::string> status_;
  std::string calls_;

  virtual void processMarkerChanges( const std::vector<visualization_msgs::InteractiveMarker>* markers = NULL,
                                     const std::vector<visualization_msgs::InteractiveMarkerPose>* poses = NULL,
                                     const std::vector<std::string>* erases = NULL )
    {
      if( markers != NULL )
      {
        std::vector<visualization_msgs::InteractiveMarker>::const_iterator marker_it;
        for( marker_it = markers->begin(); marker_it != markers->end(); marker_it++ )
        {
          marks_[ marker_it->name ] = marker_it->pose.position.x;
        }
      }

      if( poses != NULL )
      {
        std::vector<visualization_msgs::InteractiveMarkerPose>::const_iterator pose_it;
        for( pose_it = poses->begin(); pose_it != poses->end(); pose_it++ )
        {
          marks_[ pose_it->name ] = pose_it->pose.position.x;
        }
      }

      if( erases != NULL )
      {
        std::vector<std::string>::const_iterator erase_it;
        for( erase_it = erases->begin(); erase_it != erases->end(); erase_it++ )
        {
          std::map<std::string, double>::iterator find_it = marks_.find( *erase_it );
          if( find_it != marks_.end() )
          {
            marks_.erase( find_it );
          }
        }
      }

      calls_ += "processMarkerChanges ";
    }

  virtual void clearMarkers()
    {
      marks_.clear();
      calls_ += "clearMarkers ";
    }

  virtual bool subscribeToInit()
    {
      calls_ += "subscribeToInit ";
      return true;
    }

  virtual void unsubscribeFromInit()
    {
      calls_ += "unsubscribeFromInit ";
    }

  virtual void setStatusOk(const std::string& name, const std::string& text)
    {
    }

  virtual void setStatusWarn(const std::string& name, const std::string& text)
    {
    }

  virtual void setStatusError(const std::string& name, const std::string& text)
    {
    }

  void clearCalls()
    {
      calls_ = "";
    }
};

visualization_msgs::InteractiveMarkerInit::Ptr makeInit( const std::string& server_name,
                                                         uint64_t seq_num,
                                                         const std::string& marker_name,
                                                         double x_value )
{
  visualization_msgs::InteractiveMarkerInit::Ptr msg( new visualization_msgs::InteractiveMarkerInit() );
  msg->server_id = server_name;
  msg->seq_num = seq_num;
  msg->markers.resize( 1 );
  msg->markers[0].name = marker_name;
  msg->markers[0].pose.position.x = x_value;
  return msg;
}

visualization_msgs::InteractiveMarkerUpdate::Ptr makeUpdateMarker( const std::string& server_name,
                                                                   uint64_t seq_num,
                                                                   const std::string& marker_name,
                                                                   double x_value )
{
  visualization_msgs::InteractiveMarkerUpdate::Ptr msg( new visualization_msgs::InteractiveMarkerUpdate() );
  msg->server_id = server_name;
  msg->seq_num = seq_num;
  msg->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
  msg->markers.resize( 1 );
  msg->markers[0].name = marker_name;
  msg->markers[0].pose.position.x = x_value;
  return msg;
}

visualization_msgs::InteractiveMarkerUpdate::Ptr makeUpdatePose( const std::string& server_name,
                                                                 uint64_t seq_num,
                                                                 const std::string& marker_name,
                                                                 double x_value )
{
  visualization_msgs::InteractiveMarkerUpdate::Ptr msg( new visualization_msgs::InteractiveMarkerUpdate() );
  msg->server_id = server_name;
  msg->seq_num = seq_num;
  msg->type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;
  msg->poses.resize( 1 );
  msg->poses[0].name = marker_name;
  msg->poses[0].pose.position.x = x_value;
  return msg;
}

visualization_msgs::InteractiveMarkerUpdate::Ptr makeKeepAlive( const std::string& server_name,
                                                                uint64_t seq_num )
{
  visualization_msgs::InteractiveMarkerUpdate::Ptr msg( new visualization_msgs::InteractiveMarkerUpdate() );
  msg->server_id = server_name;
  msg->seq_num = seq_num;
  msg->type = visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE;
  return msg;
}

TEST(InteractiveMarkerClient, normal_startup)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "unsubscribeFromInit processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 3, "m1", 1.3 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.3, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, normal_keepalive)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeKeepAlive( "s1", 1 ));

  EXPECT_EQ( "unsubscribeFromInit ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, update_first)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges unsubscribeFromInit ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, good_keepalive_first)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeKeepAlive( "s1", 1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges unsubscribeFromInit ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, later_keepalive_first)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeKeepAlive( "s1", 2 ));

  EXPECT_EQ( "", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges unsubscribeFromInit ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}

// This tests that updates which arrive before an appropriate init
// message get queued up and replayed, and that init messages which
// are too early to initialize the queued updates are dropped.
TEST(InteractiveMarkerClient, queued_updates_second_init)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 3, "m3", 3.1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m3"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 4, "m4", 4.1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m3"] );
  EXPECT_EQ( 0, rcvr.marks_["m4"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m1"] );
  EXPECT_EQ( 0, rcvr.marks_["m3"] );
  EXPECT_EQ( 0, rcvr.marks_["m4"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 2, "m2", 2.1 ));

  EXPECT_EQ( "processMarkerChanges unsubscribeFromInit processMarkerChanges processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 0, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  EXPECT_EQ( 3.1, rcvr.marks_["m3"] );
  EXPECT_EQ( 4.1, rcvr.marks_["m4"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, normal_two_server)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s2", 1, "m2", 2.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s2", 2, "m2", 2.2 ));

  EXPECT_EQ( "unsubscribeFromInit processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.2, rcvr.marks_["m2"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, two_server_one_slow)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s2", 1, "m2", 2.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  rcvr.clearCalls();

  // Init corresponding to update just received, because we can't
  // unsubscribe from init yet.  Should be ignored.
  client.processMarkerInit( makeInit( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.1, rcvr.marks_["m2"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s2", 2, "m2", 2.2 ));

  EXPECT_EQ( "unsubscribeFromInit processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  EXPECT_EQ( 2.2, rcvr.marks_["m2"] );
  rcvr.clearCalls();
}

TEST(InteractiveMarkerClient, init_init_update)
{
  MockReceiver rcvr;
  rviz::InteractiveMarkerClient client( &rcvr );

  client.clear();
  EXPECT_EQ( "subscribeToInit ", rcvr.calls_ );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 1, "m1", 1.1 ));

  EXPECT_EQ( "processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.1, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerInit( makeInit( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "clearMarkers processMarkerChanges ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();

  client.processMarkerUpdate( makeUpdatePose( "s1", 2, "m1", 1.2 ));

  EXPECT_EQ( "unsubscribeFromInit ", rcvr.calls_ );
  EXPECT_EQ( 1.2, rcvr.marks_["m1"] );
  rcvr.clearCalls();
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init(); //needed for ros::TIme::now()
  return RUN_ALL_TESTS();
}
