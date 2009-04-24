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

#include "factory.h"

#include "visualization_manager.h"
#include "displays/grid_display.h"
#include "displays/axes_display.h"
#include "displays/point_cloud_display.h"
#include "displays/laser_scan_display.h"
#include "displays/robot_model_display.h"
#include "displays/marker_display.h"
#include "displays/planning_display.h"
#include "displays/robot_base2d_pose_display.h"
#include "displays/particle_cloud_2d_display.h"
#include "displays/poly_line_2d_display.h"
#include "displays/polygonal_map_display.h"
#include "displays/collision_map_display.h"
#include "displays/map_display.h"
#include "displays/tf_display.h"
#include "displays/camera_display.h"

namespace rviz
{

void registerFactories(VisualizationManager* manager)
{
  manager->registerFactory( GridDisplay::getTypeStatic(), GridDisplay::getDescription(), new DisplayFactoryImpl<GridDisplay>() );
  manager->registerFactory( AxesDisplay::getTypeStatic(), AxesDisplay::getDescription(), new DisplayFactoryImpl<AxesDisplay>() );
  manager->registerFactory( PointCloudDisplay::getTypeStatic(), PointCloudDisplay::getDescription(), new DisplayFactoryImpl<PointCloudDisplay>() );
  manager->registerFactory( LaserScanDisplay::getTypeStatic(), LaserScanDisplay::getDescription(), new DisplayFactoryImpl<LaserScanDisplay>() );
  manager->registerFactory( RobotModelDisplay::getTypeStatic(), RobotModelDisplay::getDescription(), new DisplayFactoryImpl<RobotModelDisplay>() );
  manager->registerFactory( MarkerDisplay::getTypeStatic(), MarkerDisplay::getDescription(), new DisplayFactoryImpl<MarkerDisplay>() );
  manager->registerFactory( PlanningDisplay::getTypeStatic(), PlanningDisplay::getDescription(), new DisplayFactoryImpl<PlanningDisplay>() );
  manager->registerFactory( RobotBase2DPoseDisplay::getTypeStatic(), RobotBase2DPoseDisplay::getDescription(), new DisplayFactoryImpl<RobotBase2DPoseDisplay>() );
  manager->registerFactory( ParticleCloud2DDisplay::getTypeStatic(), ParticleCloud2DDisplay::getDescription(), new DisplayFactoryImpl<ParticleCloud2DDisplay>() );
  manager->registerFactory( PolyLine2DDisplay::getTypeStatic(), PolyLine2DDisplay::getDescription(), new DisplayFactoryImpl<PolyLine2DDisplay>() );
  manager->registerFactory( PolygonalMapDisplay::getTypeStatic(), PolygonalMapDisplay::getDescription(), new DisplayFactoryImpl<PolygonalMapDisplay>() );
  manager->registerFactory( CollisionMapDisplay::getTypeStatic(), CollisionMapDisplay::getDescription(), new DisplayFactoryImpl<CollisionMapDisplay>() );
  manager->registerFactory( MapDisplay::getTypeStatic(), MapDisplay::getDescription(), new DisplayFactoryImpl<MapDisplay>() );
  manager->registerFactory( TFDisplay::getTypeStatic(), TFDisplay::getDescription(), new DisplayFactoryImpl<TFDisplay>() );
  manager->registerFactory( CameraDisplay::getTypeStatic(), CameraDisplay::getDescription(), new DisplayFactoryImpl<CameraDisplay>() );
}

} //namespace rviz
