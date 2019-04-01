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
#include "rviz/default_plugin/marker_display.h"
#include "rviz/default_plugin/markers/points_marker.h"
#include "rviz/default_plugin/markers/text_view_facing_marker.h"
#include "rviz/default_plugin/markers/mesh_resource_marker.h"
#include "rviz/default_plugin/markers/triangle_list_marker.h"
#include "rviz/display_context.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/validate_quaternions.h"



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

bool checkMarkerMsg(const visualization_msgs::Marker& marker, MarkerDisplay* owner)
{

  if(marker.action != visualization_msgs::Marker::ADD)
    return true;

  std::stringstream ss;

  switch (marker.type) {
  case visualization_msgs::Marker::ARROW:
    ss << checkQuaternion(marker);
    ss << checkScale(marker);
    ss << checkColor(marker);
    ss << checkPointsArrow(marker);
    ss << checkColorsEmpty(marker);
    ss << checkTextEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;

  case visualization_msgs::Marker::CUBE:
  case visualization_msgs::Marker::CYLINDER:
  case visualization_msgs::Marker::SPHERE:
    ss << checkQuaternion(marker);
    ss << checkScale(marker);
    ss << checkColor(marker);
    ss << checkPointsEmpty(marker);
    ss << checkColorsEmpty(marker);
    ss << checkTextEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;


  case visualization_msgs::Marker::LINE_STRIP:
  case visualization_msgs::Marker::LINE_LIST:
    ss << checkQuaternion(marker);
    ss << checkScaleLineStripAndList(marker);
    ss << checkPointsNotEmpty(marker);
    ss << checkColors(marker);
    ss << checkTextEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;

  case visualization_msgs::Marker::SPHERE_LIST:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::TRIANGLE_LIST:
    ss << checkQuaternion(marker);
    ss << checkScale(marker);
    ss << checkPointsNotEmpty(marker);
    ss << checkColors(marker);
    ss << checkTextEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;

  case visualization_msgs::Marker::POINTS:
    ss << checkScalePoints(marker);
    ss << checkPointsNotEmpty(marker);
    ss << checkColors(marker);
    ss << checkTextEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;

  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    ss << checkColor(marker);
    ss << checkScaleText(marker);
    ss << checkTextNotEmptyOrWhitespace(marker);
    ss << checkPointsEmpty(marker);
    ss << checkColorsEmpty(marker);
    ss << checkMeshEmpty(marker);
    break;

  case visualization_msgs::Marker::MESH_RESOURCE:
    ss << checkQuaternion(marker);
    ss << checkColor(marker);
    ss << checkScale(marker);
    ss << checkPointsEmpty(marker);
    ss << checkColorsEmpty(marker);
    ss << checkTextEmpty(marker);
    break;

  default:
    ss << "Unknown marker type: " << marker.type ;
  }

  std::string warning = ss.str();
  if(!warning.empty())
  {
    if(warning.substr(warning.length()-2) == ", ")
      warning = warning.erase(warning.length()-2, 2);

    owner->setMarkerStatus(MarkerID(marker.ns, marker.id), StatusProperty::Warn, warning);
    ROS_WARN("Marker '%s/%d': %s", marker.ns.c_str(), marker.id, warning.c_str());
    return false;
  }

  owner->deleteMarkerStatus(MarkerID(marker.ns, marker.id));
  return true;
}

bool checkMarkerArrayMsg(const visualization_msgs::MarkerArray& array, MarkerDisplay* owner)
{
  std::vector<MarkerID> marker_ids;

  bool add_marker_in_array = false;

  for(int i = 0; i<array.markers.size(); i++)
  {
    if(array.markers[i].action == visualization_msgs::Marker::ADD)
    {
      add_marker_in_array = true;
    }

    if(add_marker_in_array && i != 0 && array.markers[i].action == visualization_msgs::Marker::DELETEALL)
    {
      std::stringstream warning;
      warning << "found a DELETEALL at index " << i << ", previous markers in the MarkerArray will never show";
      ROS_WARN("MarkerArray: %s", warning.str().c_str());
      owner->setStatusStd(StatusProperty::Warn, "marker_array", warning.str());
      return false;
    }
    MarkerID current_id(array.markers[i].ns, array.markers[i].id);
    std::vector<MarkerID>::iterator search = std::lower_bound(marker_ids.begin(), marker_ids.end(), current_id);
    if (search != marker_ids.end())
    {
      std::stringstream ss;
      ss << "found '" <<  array.markers[i].ns.c_str() << "/" << array.markers[i].id << "' multiple times";
      ROS_WARN("MarkerArray: %s", ss.str().c_str());
      owner->setStatusStd(StatusProperty::Warn, "marker_array", ss.str());
      return false;
    }
    else
    {
      marker_ids.insert(search, current_id);
    }
  }
  owner->setStatusStd(StatusProperty::Ok, "marker_array", "OK");
  return true;
}


const char* checkQuaternion(const visualization_msgs::Marker& marker)
{
  if (marker.pose.orientation.x == 0.0
    && marker.pose.orientation.y == 0.0
    && marker.pose.orientation.z == 0.0
    && marker.pose.orientation.w == 0.0)
  {
    return "uninitialized quaternion assuming identity, ";
  }
  if(!validateQuaternions(marker.pose))
    return "unnormalized quaternion in marker message, ";

  return "";
}

const char* checkScale(const visualization_msgs::Marker& marker)
{
  // for ARROW markers, scale.z is the optional head length
  if(marker.type == visualization_msgs::Marker::ARROW && marker.scale.x != 0.0 && marker.scale.y != 0.0)
    return "";

  if(marker.scale.x == 0.0  || marker.scale.y == 0.0  || marker.scale.z == 0.0)
  {
    if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST)
      return "scale contains 0.0 in x, y or z, TRIANGLE_LIST coordinates are scaled, ";
    return "scale contains 0.0 in x, y or z, ";
  }
  return "";
}

const char* checkScaleLineStripAndList(const visualization_msgs::Marker& marker)
{
  if(marker.scale.x == 0.0)
  {
    return "width LINE_LIST or LINE_STRIP is 0.0 (scale.x), ";
  }
  else if(marker.scale.y != 0.0 || marker.scale.z != 0.0)
  {
    return "scale.y and scale.z of LINE_LIST or LINE_STRIP are ignored, ";
  }
  return "";
}

const char* checkScalePoints(const visualization_msgs::Marker& marker)
{
  if(marker.scale.x == 0.0 || marker.scale.y == 0.0)
  {
    return "width and/or height of POINTS is 0.0 (scale.x, scale.y), ";
  }
  else if(marker.scale.z != 0.0)
  {
    return "scale.z of POINTS is ignored, ";
  }
  return "";
}

const char* checkScaleText(const visualization_msgs::Marker& marker)
{
  if(marker.scale.z == 0.0)
  {
    return "text height of TEXT_VIEW_FACING is 0.0 (scale.z), ";
  }
  else if(marker.scale.x != 0.0 || marker.scale.y != 0.0)
  {
    return "scale.x and scale.y of TEXT_VIEW_FACING are ignored, ";
  }
  return "";
}

const char* checkColor(const visualization_msgs::Marker& marker)
{
  if(marker.color.a == 0.0)
    return "marker is fully transparent (color.a is 0.0), ";
  return "";
}

const char* checkPointsArrow(const visualization_msgs::Marker& marker)
{
  if(marker.points.size() != 0 && marker.points.size() != 2)
    return "Number of points for an ARROW marker should be either 0 or 2";
  return "";
}

const char* checkPointsNotEmpty(const visualization_msgs::Marker& marker)
{
  if(marker.points.empty())
    return "points should not be empty for specified marker type, " ;
  else if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST && (marker.points.size() % 3) != 0)
    return "number of points should be a multiple of 3 for TRIANGLE_LIST Marker, ";
  else if(marker.type == visualization_msgs::Marker::LINE_LIST && (marker.points.size() % 2) != 0)
    return "number of points should be a multiple of 2 for LINE_LIST Marker, ";
  else if(marker.type == visualization_msgs::Marker::LINE_STRIP && marker.points.size() <= 1)
    return "at least two points are required for a LINE_STRIP Marker, ";
  return "";
}

const char* checkPointsEmpty(const visualization_msgs::Marker& marker)
{
  if(!marker.points.empty())
    return "points array is ignored by specified marker type, ";
  return "";
}

const char* checkColors(const visualization_msgs::Marker& marker)
{
  if(marker.colors.size() == 0)
    return checkColor(marker);
  if(marker.colors.size() != marker.points.size())
    return "number of colors is not equal to number of points or 0, ";
  return "";
}

const char* checkColorsEmpty(const visualization_msgs::Marker& marker)
{
  if(!marker.colors.empty())
    return "colors array is ignored by specified marker type, ";
  return "";
}

const char* checkTextNotEmptyOrWhitespace(const visualization_msgs::Marker& marker)
{
  if(marker.text.empty())
    return "text is empty for TEXT_VIEW_FACING type marker, ";
  else if(marker.text.find_first_not_of(" \t\n\v\f\r") == std::string::npos)
    return "text of TEXT_VIEW_FACING Marker only consists of whitespaces, ";
  return "";
}

const char* checkTextEmpty(const visualization_msgs::Marker& marker)
{
  if(!marker.text.empty())
    return "text is ignored for specified marker type, ";
  return "";
}

const char* checkMesh(const visualization_msgs::Marker& marker)
{
  if(marker.mesh_resource.empty())
    return "path to mesh resource is empty for MESH_RESOURCE marker, ";
  return "";
}

const char* checkMeshEmpty(const visualization_msgs::Marker& marker)
{
  std::stringstream ss;
  if (!marker.mesh_resource.empty())
    ss << "mesh_resource is ignored for specified marker type, ";
  if (marker.mesh_use_embedded_materials)
    ss << "using embedded materials is not supported for markers other than MESH_RESOURCE, ";
  return ss.str().c_str();
}

} // namespace rviz
