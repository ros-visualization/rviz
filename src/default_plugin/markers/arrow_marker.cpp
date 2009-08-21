/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "arrow_marker.h"
#include "rviz/common.h"
#include "marker_selection_handler.h"

#include "rviz/visualization_manager.h"
#include "rviz/selection/selection_manager.h"

#include <ogre_tools/arrow.h>

#include <tf/transform_listener.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

ArrowMarker::ArrowMarker(VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(manager, parent_node)
, arrow_(0)
{
}

ArrowMarker::~ArrowMarker()
{
  delete arrow_;
}

void ArrowMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::ARROW);

  if (!new_message->points.empty() && new_message->points.size() < 2)
  {
    ROS_ERROR("Arrow marker [%s/%d] only specified one point of a point to point arrow.", new_message->ns.c_str(), new_message->id);
    delete arrow_;
    arrow_ = 0;

    return;
  }

  if (!arrow_)
  {
    arrow_ = new ogre_tools::Arrow(vis_manager_->getSceneManager(), parent_node_);
    coll_ = vis_manager_->getSelectionManager()->createCollisionForObject(arrow_, SelectionHandlerPtr(new MarkerSelectionHandler(this, MarkerID(new_message->ns, new_message->id))), coll_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  if (new_message->points.empty())
  {
    arrow_->setPosition(pos);
    arrow_->setOrientation(orient);
    arrow_->setScale(scale);
  }
  else
  {
    const geometry_msgs::Point& start_pos = new_message->pose.position;
    const geometry_msgs::Point& p1 = new_message->points[0];
    const geometry_msgs::Point& p2 = new_message->points[1];

    tf::Stamped<tf::Point> t_p1;
    tf::Stamped<tf::Point> t_p2;
    try
    {
      vis_manager_->getTFClient()->transformPoint(vis_manager_->getFixedFrame(), tf::Stamped<tf::Point>(tf::Point(p1.x, p1.y, p1.z) + tf::Point(start_pos.x, start_pos.y, start_pos.z), new_message->header.stamp, new_message->header.frame_id), t_p1);
      vis_manager_->getTFClient()->transformPoint(vis_manager_->getFixedFrame(), tf::Stamped<tf::Point>(tf::Point(p2.x, p2.y, p2.z) + tf::Point(start_pos.x, start_pos.y, start_pos.z), new_message->header.stamp, new_message->header.frame_id), t_p2);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR( "Error transforming marker [%s/%d] from frame [%s] to frame [%s]: %s\n", new_message->ns.c_str(), new_message->id, new_message->header.frame_id.c_str(), vis_manager_->getFixedFrame().c_str(), e.what() );
      delete arrow_;
      arrow_ = 0;
      return;
    }

    Ogre::Vector3 point1(t_p1.x(), t_p1.y(), t_p1.z());
    Ogre::Vector3 point2(t_p2.x(), t_p2.y(), t_p2.z());
    robotToOgre(point1);
    robotToOgre(point2);

    Ogre::Vector3 direction = point2 - point1;
    float distance = direction.length();
    direction.normalise();
    Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );
    arrow_->setPosition(point1);
    arrow_->setOrientation(orient);
    arrow_->setScale(Ogre::Vector3(1.0f, 1.0f, 1.0f));

    float head_length = 0.1*distance;
    float shaft_length = distance - head_length;
    arrow_->set(shaft_length, new_message->scale.x, head_length, new_message->scale.y);
  }

  arrow_->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);
}

}
