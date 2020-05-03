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


#ifndef RVIZ_GRID_CELLS_DISPLAY_H
#define RVIZ_GRID_CELLS_DISPLAY_H

#include <rviz/message_filter_display.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>

#include <boost/shared_ptr.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class PointCloud;

/**
 * \class GridCellsDisplay
 * \brief Displays a nav_msgs::GridCells message
 */
class GridCellsDisplay : public MessageFilterDisplay<nav_msgs::GridCells>
{
  Q_OBJECT
public:
  GridCellsDisplay();
  ~GridCellsDisplay() override;

  void onInitialize() override;
  void reset() override;

private Q_SLOTS:
  void updateAlpha();

private:
  void processMessage(const nav_msgs::GridCells::ConstPtr& msg) override;

  PointCloud* cloud_;

  ColorProperty* color_property_;
  FloatProperty* alpha_property_;

  uint64_t last_frame_count_;
};

} // namespace rviz

#endif /* RVIZ_GRID_CELLS_DISPLAY_H */
