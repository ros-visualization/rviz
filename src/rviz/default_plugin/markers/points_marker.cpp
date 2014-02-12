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
#include "rviz/default_plugin/marker_display.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "marker_selection_handler.h"

#include <rviz/ogre_helpers/point_cloud.h>

#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace rviz
{

PointsMarker::PointsMarker(MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
: MarkerBase(owner, context, parent_node)
, points_(0)
{
}

PointsMarker::~PointsMarker()
{
  delete points_;
}

void PointsMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::POINTS ||
             new_message->type == visualization_msgs::Marker::CUBE_LIST ||
             new_message->type == visualization_msgs::Marker::SPHERE_LIST);

  if (!points_)
  {
    points_ = new PointCloud();
    scene_node_->attachObject(points_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  switch (new_message->type)
  {
  case visualization_msgs::Marker::POINTS:
    points_->setRenderMode(PointCloud::RM_SQUARES);
    points_->setDimensions(new_message->scale.x, new_message->scale.y, 0.0f);
    break;
  case visualization_msgs::Marker::CUBE_LIST:
    points_->setRenderMode(PointCloud::RM_BOXES);
    points_->setDimensions(scale.x, scale.y, scale.z);
    break;
  case visualization_msgs::Marker::SPHERE_LIST:
    points_->setRenderMode(PointCloud::RM_SPHERES);
    points_->setDimensions(scale.x, scale.y, scale.z);
    break;
  }

  setPosition(pos);
  setOrientation(orient);

  points_->clear();

  if (new_message->points.empty())
  {
    return;
  }

  float r = new_message->color.r;
  float g = new_message->color.g;
  float b = new_message->color.b;
  float a = new_message->color.a;

  bool has_per_point_color = new_message->colors.size() == new_message->points.size();

  bool has_nonzero_alpha = false;
  bool has_per_point_alpha = false;

  typedef std::vector< PointCloud::Point > V_Point;
  V_Point points;
  points.resize(new_message->points.size());
  std::vector<geometry_msgs::Point>::const_iterator it = new_message->points.begin();
  std::vector<geometry_msgs::Point>::const_iterator end = new_message->points.end();
  for (int i = 0; it != end; ++it, ++i)
  {
    const geometry_msgs::Point& p = *it;
    PointCloud::Point& point = points[i];

    Ogre::Vector3 v(p.x, p.y, p.z);

    point.position.x = v.x;
    point.position.y = v.y;
    point.position.z = v.z;

    if (has_per_point_color)
    {
      const std_msgs::ColorRGBA& color = new_message->colors[i];
      r = color.r;
      g = color.g;
      b = color.b;
      a = color.a;
      has_nonzero_alpha = has_nonzero_alpha || a != 0.0;
      has_per_point_alpha = has_per_point_alpha || a != 1.0;
    }

    point.setColor(r, g, b, a);
  }

  if (has_per_point_color)
  {
    if (!has_nonzero_alpha && owner_)
    {
      owner_->setMarkerStatus(getID(), StatusProperty::Warn, "All points have a zero alpha value.");
    }
    points_->setAlpha(1.0, has_per_point_alpha);
  }
  else
  {
    points_->setAlpha(a);
  }

  points_->addPoints(&points.front(), points.size());

  handler_.reset( new MarkerSelectionHandler( this, MarkerID( new_message->ns, new_message->id ), context_ ));
  points_->setPickColor( SelectionManager::handleToColor( handler_->getHandle() ));
}

void PointsMarker::setHighlightColor( float r, float g, float b )
{
  points_->setHighlightColor( r, g, b );
}

} // end namespace rviz
