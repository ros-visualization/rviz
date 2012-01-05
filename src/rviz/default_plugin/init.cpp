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

#include <pluginlib/class_list_macros.h>

#include "axes_display.h"
#include "camera_display.h"
#include "grid_cells_display.h"
#include "grid_display.h"
#include "image_display.h"
#include "interactive_marker_display.h"
#include "laser_scan_display.h"
#include "map_display.h"
#include "marker_array_display.h"
#include "marker_display.h"
#include "odometry_display.h"
#include "path_display.h"
#include "point_cloud2_display.h"
#include "point_cloud_display.h"
#include "point_cloud_transformers.h"
#include "polygon_display.h"
#include "pose_array_display.h"
#include "pose_display.h"
#include "range_display.h"
#include "robot_model_display.h"
#include "tf_display.h"

PLUGINLIB_DECLARE_CLASS( rviz_qt, Axes, rviz::AxesDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Camera, rviz::CameraDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, GridCells, rviz::GridCellsDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Grid, rviz::GridDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Image, rviz::ImageDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, InteractiveMarker, rviz::InteractiveMarkerDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, LaserScan, rviz::LaserScanDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Map, rviz::MapDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, MarkerArray, rviz::MarkerArrayDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Marker, rviz::MarkerDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Odometry, rviz::OdometryDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Path, rviz::PathDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, PointCloud2, rviz::PointCloud2Display, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, PointCloud, rviz::PointCloudDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Polygon, rviz::PolygonDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, PoseArray, rviz::PoseArrayDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Pose, rviz::PoseDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Range, rviz::RangeDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, RobotModel, rviz::RobotModelDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz_qt, TF, rviz::TFDisplay, rviz::Display )

PLUGINLIB_DECLARE_CLASS( rviz_qt, AxisColor, rviz::AxisColorPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz_qt, FlatColor, rviz::FlatColorPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz_qt, Intensity, rviz::IntensityPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz_qt, RGB8, rviz::RGB8PCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz_qt, RGBF32, rviz::RGBF32PCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz_qt, XYZ, rviz::XYZPCTransformer, rviz::PointCloudTransformer )
