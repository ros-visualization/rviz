/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include <OgreRay.h>
#include <OgreVector3.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>

#include <rviz/default_plugin/tools/point_tool.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>

#include <geometry_msgs/PointStamped.h>

#include <sstream>

namespace rviz
{
PointTool::PointTool() : Tool()
{
  shortcut_key_ = 'c';

  topic_property_ =
      new StringProperty("Topic", "/clicked_point", "The topic on which to publish points.",
                         getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ =
      new BoolProperty("Single click", true, "Switch away from this tool after one click.",
                       getPropertyContainer(), SLOT(updateAutoDeactivate()), this);

  updateTopic();
}

PointTool::~PointTool()
{
}

void PointTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = getDefaultCursor();
}

void PointTool::activate()
{
}

void PointTool::deactivate()
{
}

void PointTool::updateTopic()
{
  try
  {
    pub_ = nh_.advertise<geometry_msgs::PointStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("PointTool", e.what());
  }
}

void PointTool::updateAutoDeactivate()
{
}

int PointTool::processMouseEvent(ViewportMouseEvent& event)
{
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, pos);
  setCursor(success ? hit_cursor_ : std_cursor_);

  if (success)
  {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus(s.str().c_str());

    if (event.leftUp())
    {
      geometry_msgs::PointStamped ps;
      ps.point.x = pos.x;
      ps.point.y = pos.y;
      ps.point.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      pub_.publish(ps);

      if (auto_deactivate_property_->getBool())
      {
        flags |= Finished;
      }
    }
  }
  else
  {
    setStatus("Move over an object to select the target point.");
  }

  return flags;
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::PointTool, rviz::Tool)
