/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "rviz/default_plugin/tools/goal_tool.h"

//ADDED
#include "rviz/properties/bool_property.h"
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "rviz/geometry.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

namespace rviz
{

GoalTool::GoalTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );

  // ADDED
    auto_deactivate_property_ = new BoolProperty( "Single click", true,
                                                  "Switch away from this tool after one click.",
                                                  getPropertyContainer(), SLOT( updateAutoDeactivate() ), this );
}


void GoalTool::onInitialize()
{
    // THIS IS FOR ARROW size TUNING
    // DELETED PoseTool::onInitialize();
    arrow_ = new Arrow( scene_manager_, NULL, 0.25f, 0.07f, 0.15f, 0.2f ); // ADDED
    arrow_->setColor( 0.0f, 0.0f, 1.0f, 1.0f ); // ADDED
    arrow_->getSceneNode()->setVisible( false ); // ADDED
    // TILL HERE
  setName( "2D Nav Goal" );
  updateTopic();
}

void GoalTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1 );
}

void GoalTool::updateAutoDeactivate()  // ADDED
{
}

void GoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal;
  tf::poseStampedTFToMsg(p, goal);
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);
  pub_.publish(goal);
}

// ADDED THE WHOLE FUNCTION
int GoalTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  if( event.leftDown() )
  {
    ROS_ASSERT( state_ == Position );

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( getPointOnPlaneFromWindowXY( event.viewport,
                                     ground_plane,
                                     event.x, event.y, intersection ))
    {
      pos_ = intersection;
      arrow_->setPosition( pos_ );

      state_ = Orientation;
      flags |= Render;
    }
  }
  else if( event.type == QEvent::MouseMove && event.left() )
  {
    if( state_ == Orientation )
    {
      //compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
      if( getPointOnPlaneFromWindowXY( event.viewport,
                                       ground_plane,
                                       event.x, event.y, cur_pos ))
      {
        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );

        arrow_->getSceneNode()->setVisible( true );

        //we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion( Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y );

        arrow_->setOrientation( Ogre::Quaternion( Ogre::Radian(angle), Ogre::Vector3::UNIT_Z ) * orient_x );

        flags |= Render;
      }
    }
  }
  else if( event.leftUp() )
  {
    if( state_ == Orientation )
    {
      //compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
      if( getPointOnPlaneFromWindowXY( event.viewport,
                                       ground_plane,
                                       event.x, event.y, cur_pos ))
      {
        double angle = atan2( cur_pos.y - pos_.y, cur_pos.x - pos_.x );

        onPoseSet(pos_.x, pos_.y, angle);


        // ADDED
        if ( auto_deactivate_property_->getBool() )
        {
        flags |= (Finished|Render);
        }
         else if (!auto_deactivate_property_->getBool())
        {
            state_ = Position;
            arrow_->getSceneNode()->setVisible( false );
        } // ADDED TILL HERE

      }
    }
  }

  return flags;
}
// TILL HERE



} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::GoalTool, rviz::Tool )
