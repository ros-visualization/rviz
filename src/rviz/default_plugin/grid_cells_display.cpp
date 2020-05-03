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

#include <boost/bind.hpp>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>

#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>
#include <rviz/display_context.h>

#include "grid_cells_display.h"

namespace rviz
{
GridCellsDisplay::GridCellsDisplay() : MFDClass(), last_frame_count_(uint64_t(-1))
{
  color_property_ = new ColorProperty("Color", QColor(25, 255, 0), "Color of the grid cells.", this);

  alpha_property_ = new FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the cells.",
                                      this, SLOT(updateAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
}

void GridCellsDisplay::onInitialize()
{
  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine" << count++;

  cloud_ = new PointCloud();
  cloud_->setRenderMode(PointCloud::RM_TILES);
  cloud_->setCommonDirection(Ogre::Vector3::UNIT_Z);
  cloud_->setCommonUpVector(Ogre::Vector3::UNIT_Y);
  scene_node_->attachObject(cloud_);
  updateAlpha();

  MFDClass::onInitialize();
}

GridCellsDisplay::~GridCellsDisplay()
{
  if (initialized())
  {
    unsubscribe();
    GridCellsDisplay::reset();
    scene_node_->detachObject(cloud_);
    delete cloud_;
  }
}

void GridCellsDisplay::reset()
{
  MFDClass::reset();
  cloud_->clear();
}

void GridCellsDisplay::updateAlpha()
{
  cloud_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

bool validateFloats(const nav_msgs::GridCells& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.cell_width);
  valid = valid && validateFloats(msg.cell_height);
  valid = valid && validateFloats(msg.cells);
  return valid;
}

void GridCellsDisplay::processMessage(const nav_msgs::GridCells::ConstPtr& msg)
{
  if (context_->getFrameCount() == last_frame_count_)
    return;
  last_frame_count_ = context_->getFrameCount();

  cloud_->clear();

  if (!validateFloats(*msg))
  {
    setStatus(StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  if (msg->cell_width == 0)
  {
    setStatus(StatusProperty::Error, "Topic", "Cell width is zero, cells will be invisible.");
  }
  else if (msg->cell_height == 0)
  {
    setStatus(StatusProperty::Error, "Topic", "Cell height is zero, cells will be invisible.");
  }

  cloud_->setDimensions(msg->cell_width, msg->cell_height, 0.0);

  Ogre::ColourValue color_int = qtToOgre(color_property_->getColor());
  uint32_t num_points = msg->cells.size();

  typedef std::vector<PointCloud::Point> V_Point;
  V_Point points;
  points.resize(num_points);
  for (uint32_t i = 0; i < num_points; i++)
  {
    PointCloud::Point& current_point = points[i];
    current_point.position.x = msg->cells[i].x;
    current_point.position.y = msg->cells[i].y;
    current_point.position.z = msg->cells[i].z;
    current_point.color = color_int;
  }

  cloud_->clear();

  if (!points.empty())
  {
    cloud_->addPoints(&points.front(), points.size());
  }
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::GridCellsDisplay, rviz::Display)
