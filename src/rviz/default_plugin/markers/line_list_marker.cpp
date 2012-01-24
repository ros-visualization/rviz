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
#include "rviz/default_plugin/marker_display.h"
#include "rviz/visualization_manager.h"

#include <rviz/ogre_helpers/billboard_line.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>

namespace rviz
{

LineListMarker::LineListMarker(MarkerDisplay* owner, VisualizationManager* manager, Ogre::SceneNode* parent_node)
: MarkerBase(owner, manager, parent_node)
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
    lines_ = new BillboardLine(vis_manager_->getSceneManager(), scene_node_);
  }

  Ogre::Vector3 pos, scale;
  Ogre::Quaternion orient;
  transform(new_message, pos, orient, scale);

  setPosition(pos);
  setOrientation(orient);
  lines_->setScale(scale);
  lines_->setColor(new_message->color.r, new_message->color.g, new_message->color.b, new_message->color.a);

  lines_->clear();

  if (new_message->points.empty())
  {
    return;
  }

  bool has_per_point_color = new_message->colors.size() == new_message->points.size();

  if (new_message->points.size() % 2 == 0)
  {
    lines_->setLineWidth( new_message->scale.x );
    lines_->setMaxPointsPerLine(2);
    lines_->setNumLines(new_message->points.size() / 2);

    size_t i = 0;
    std::vector<geometry_msgs::Point>::const_iterator it = new_message->points.begin();
    std::vector<geometry_msgs::Point>::const_iterator end = new_message->points.end();
    for ( ; it != end; )
    {
      if (it != new_message->points.begin())
      {
        lines_->newLine();
      }

      for (uint32_t j = 0; j < 2; ++j, ++it, ++i)
      {
        const geometry_msgs::Point& p = *it;

        Ogre::ColourValue c;
        if (has_per_point_color)
        {
          const std_msgs::ColorRGBA& color = new_message->colors[i];
          c.r = color.r;
          c.g = color.g;
          c.b = color.b;
          c.a = new_message->color.a;
        }
        else
        {
          c.r = new_message->color.r;
          c.g = new_message->color.g;
          c.b = new_message->color.b;
          c.a = new_message->color.a;
        }

        Ogre::Vector3 v( p.x, p.y, p.z );
        lines_->addPoint( v, c );
      }
    }
  }
  else
  {
    std::stringstream ss;
    ss << "Line list marker [" << getStringID() << "] has an odd number of points.";
    if ( owner_ )
    {
      owner_->setMarkerStatus(getID(), status_levels::Error, ss.str());
    }
    ROS_DEBUG("%s", ss.str().c_str());
  }
}

S_MaterialPtr LineListMarker::getMaterials()
{
  S_MaterialPtr materials;
  materials.insert( lines_->getMaterial() );
  return materials;
}

}
