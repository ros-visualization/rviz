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

#include "points_marker.h"
#include "default_plugin/marker_display.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"

#include <ogre_tools/point_cloud.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

PointsMarker::PointsMarker(MarkerDisplay* owner, VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(owner, manager, parent_node)
, points_(0)
{
  if (parent_node)
  {
    scene_node_ = parent_node->createChildSceneNode();
  }
  else
  {
    scene_node_ = vis_manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
  }
}

PointsMarker::~PointsMarker()
{
  vis_manager_->getSceneManager()->destroySceneNode(scene_node_->getName());
  delete points_;
}

void PointsMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::POINTS ||
             new_message->type == visualization_msgs::Marker::CUBE_LIST);

  if (!points_)
  {
    points_ = new ogre_tools::PointCloud();
    scene_node_->attachObject(points_);
  }

  switch (new_message->type)
  {
  case visualization_msgs::Marker::POINTS:
    points_->setRenderMode(ogre_tools::PointCloud::RM_BILLBOARDS);
    break;
  case visualization_msgs::Marker::CUBE_LIST:
    points_->setRenderMode(ogre_tools::PointCloud::RM_BOXES);
    break;
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  points_->setDimensions(scale.x, scale.y, scale.z);

  scene_node_->setPosition(pos);
  scene_node_->setOrientation(orient);

  points_->clear();

  if (new_message->points.empty())
  {
    return;
  }

  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;
  points_->setAlpha(a);

  bool has_per_point_color = new_message->colors.size() == new_message->points.size();

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize(new_message->points.size());
  std::vector<geometry_msgs::Point>::const_iterator it = new_message->points.begin();
  std::vector<geometry_msgs::Point>::const_iterator end = new_message->points.end();
  for (int i = 0; it != end; ++it, ++i)
  {
    const geometry_msgs::Point& p = *it;
    ogre_tools::PointCloud::Point& point = points[i];

    Ogre::Vector3 v(p.x, p.y, p.z);
    robotToOgre(v);

    point.x = v.x;
    point.y = v.y;
    point.z = v.z;

    if (has_per_point_color)
    {
      const std_msgs::ColorRGBA& color = new_message->colors[i];
      r = color.r;
      g = color.g;
      b = color.b;
    }

    point.setColor(r, g, b);
  }

  points_->addPoints(&points.front(), points.size());
}

}
