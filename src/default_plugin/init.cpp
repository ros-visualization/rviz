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

#include "rviz/plugin/type_registry.h"

#include "axes_display.h"
#include "camera_display.h"
#include "grid_display.h"
#include "laser_scan_display.h"
#include "map_display.h"
#include "marker_display.h"
#include "particle_cloud_2d_display.h"
#include "point_cloud_display.h"
#include "path_display.h"
#include "polygon_display.h"
#include "grid_cells_display.h"
#include "robot_base2d_pose_display.h"
#include "robot_model_display.h"
#include "tf_display.h"

using namespace rviz;

extern "C" void rvizPluginInit(rviz::TypeRegistry* reg)
{
  reg->registerDisplay<AxesDisplay>("rviz::AxesDisplay");
  reg->registerDisplay<CameraDisplay>("rviz::CameraDisplay");
  reg->registerDisplay<GridDisplay>("rviz::GridDisplay");
  reg->registerDisplay<LaserScanDisplay>("rviz::LaserScanDisplay");
  reg->registerDisplay<MapDisplay>("rviz::MapDisplay");
  reg->registerDisplay<MarkerDisplay>("rviz::MarkerDisplay");
  reg->registerDisplay<ParticleCloud2DDisplay>("rviz::ParticleCloud2DDisplay");
  reg->registerDisplay<PointCloudDisplay>("rviz::PointCloudDisplay");
  reg->registerDisplay<PathDisplay>("rviz::PathDisplay");
  reg->registerDisplay<PolygonDisplay>("rviz::PolygonDisplay");
  reg->registerDisplay<GridCellsDisplay>("rviz::GridCellsDisplay");
  reg->registerDisplay<RobotBase2DPoseDisplay>("rviz::RobotBase2DPoseDisplay");
  reg->registerDisplay<RobotModelDisplay>("rviz::RobotModelDisplay");
  reg->registerDisplay<TFDisplay>("rviz::TFDisplay");
}
