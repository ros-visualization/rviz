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

#include "line_list_marker.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"

#include <ogre_tools/billboard_line.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace rviz
{

LineListMarker::LineListMarker(VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(manager, parent_node)
, lines_(0)
{
}

LineListMarker::~LineListMarker()
{
  delete lines_;
}

void LineListMarker::onNewMessage(const MarkerConstPtr& old_message, const MarkerConstPtr& new_message)
{
  ROS_ASSERT(new_message->type == visualization_msgs::Marker::LINE_LIST);

  if (!lines_)
  {
    lines_ = new ogre_tools::BillboardLine(vis_manager_->getSceneManager(), parent_node_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  lines_->setPosition(pos);
  lines_->setOrientation(orient);
  lines_->setScale(scale);
  lines_->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);

  if (new_message->points.empty())
  {
    ROS_ERROR("Marker [%s/%d] is a lines_ list with no points!", new_message->ns.c_str(), new_message->id);
    return;
  }

  if (new_message->points.size() % 2 == 0)
  {
    lines_->clear();
    lines_->setLineWidth( new_message->scale.x );
    lines_->setMaxPointsPerLine(2);
    lines_->setNumLines(new_message->points.size() / 2);

    std::vector<geometry_msgs::Point>::const_iterator it = new_message->points.begin();
    std::vector<geometry_msgs::Point>::const_iterator end = new_message->points.end();
    for ( ; it != end; ++it )
    {
      if (it != new_message->points.begin())
      {
        lines_->newLine();
      }

      const geometry_msgs::Point& p = *it;
      ++it;
      const geometry_msgs::Point& p2 = *it;

      Ogre::Vector3 v( p.x, p.y, p.z );
      robotToOgre( v );
      lines_->addPoint( v );

      v = Ogre::Vector3( p2.x, p2.y, p2.z );
      robotToOgre( v );
      lines_->addPoint( v );
    }
  }
  else
  {
    ROS_ERROR("Marker [%s/%d] with type LINE_LIST has an odd number of points", new_message->ns.c_str(), new_message->id);
  }
}

}
