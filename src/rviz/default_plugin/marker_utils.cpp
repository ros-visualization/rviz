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

#include "marker_utils.h"
#include "rviz/default_plugin/markers/shape_marker.h"
#include "rviz/default_plugin/markers/arrow_marker.h"
#include "rviz/default_plugin/markers/line_list_marker.h"
#include "rviz/default_plugin/markers/line_strip_marker.h"
#include "rviz/default_plugin/markers/points_marker.h"
#include "rviz/default_plugin/markers/text_view_facing_marker.h"
#include "rviz/default_plugin/markers/mesh_resource_marker.h"
#include "rviz/default_plugin/markers/triangle_list_marker.h"

namespace rviz
{

MarkerBase* createMarker(int marker_type, MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
{
  switch (marker_type) {
  case visualization_msgs::Marker::CUBE:
  case visualization_msgs::Marker::CYLINDER:
  case visualization_msgs::Marker::SPHERE:
     return new rviz::ShapeMarker(owner, context, parent_node);

  case visualization_msgs::Marker::ARROW:
     return new rviz::ArrowMarker(owner, context, parent_node);

  case visualization_msgs::Marker::LINE_STRIP:
     return new rviz::LineStripMarker(owner, context, parent_node);

  case visualization_msgs::Marker::LINE_LIST:
     return new rviz::LineListMarker(owner, context, parent_node);

  case visualization_msgs::Marker::SPHERE_LIST:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::POINTS:
     return new rviz::PointsMarker(owner, context, parent_node);

  case visualization_msgs::Marker::TEXT_VIEW_FACING:
     return new rviz::TextViewFacingMarker(owner, context, parent_node);

  case visualization_msgs::Marker::MESH_RESOURCE:
     return new rviz::MeshResourceMarker(owner, context, parent_node);

  case visualization_msgs::Marker::TRIANGLE_LIST:
     return new rviz::TriangleListMarker(owner, context, parent_node);

  default:
     return nullptr;
  }
}

} // namespace rviz
