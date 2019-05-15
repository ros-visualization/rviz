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
    checkQuaternion(marker, ss);
    checkScale(marker, ss);
    checkColor(marker, ss);
    checkPointsArrow(marker, ss);
    checkColorsEmpty(marker, ss);
    checkTextEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;

  case visualization_msgs::Marker::CUBE:
  case visualization_msgs::Marker::CYLINDER:
  case visualization_msgs::Marker::SPHERE:
    checkQuaternion(marker, ss);
    checkScale(marker, ss);
    checkColor(marker, ss);
    checkPointsEmpty(marker, ss);
    checkColorsEmpty(marker, ss);
    checkTextEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;


  case visualization_msgs::Marker::LINE_STRIP:
  case visualization_msgs::Marker::LINE_LIST:
    checkQuaternion(marker, ss);
    checkScaleLineStripAndList(marker, ss);
    checkPointsNotEmpty(marker, ss);
    checkColors(marker, ss);
    checkTextEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;

  case visualization_msgs::Marker::SPHERE_LIST:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::TRIANGLE_LIST:
    checkQuaternion(marker, ss);
    checkScale(marker, ss);
    checkPointsNotEmpty(marker, ss);
    checkColors(marker, ss);
    checkTextEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;

  case visualization_msgs::Marker::POINTS:
    checkScalePoints(marker, ss);
    checkPointsNotEmpty(marker, ss);
    checkColors(marker, ss);
    checkTextEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;

  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    checkColor(marker, ss);
    checkScaleText(marker, ss);
    checkTextNotEmptyOrWhitespace(marker, ss);
    checkPointsEmpty(marker, ss);
    checkColorsEmpty(marker, ss);
    checkMeshEmpty(marker, ss);
    break;

  case visualization_msgs::Marker::MESH_RESOURCE:
    checkQuaternion(marker, ss);
    checkColor(marker, ss);
    checkScale(marker, ss);
    checkPointsEmpty(marker, ss);
    checkColorsEmpty(marker, ss);
    checkTextEmpty(marker, ss);
    break;

  default:
    ss << "Unknown marker type: " << marker.type ;
  }

  if(ss.tellp() != 0) //stringstream is not empty
  {
    std::string warning = ss.str();

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

  for(int i = 0; i < array.markers.size(); i++)
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


void checkQuaternion(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if (marker.pose.orientation.x == 0.0
    && marker.pose.orientation.y == 0.0
    && marker.pose.orientation.z == 0.0
    && marker.pose.orientation.w == 0.0)
  {
    addCommaIfRequired(ss);
    ss << "uninitialized quaternion assuming identity";
  }
  else if(!validateQuaternions(marker.pose))
  {
    addCommaIfRequired(ss);
    ss << "unnormalized quaternion in marker message";
  }
}

void checkScale(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  // for ARROW markers, scale.z is the optional head length
  if(marker.type == visualization_msgs::Marker::ARROW && marker.scale.x != 0.0 && marker.scale.y != 0.0)
    return;

  if(marker.scale.x == 0.0  || marker.scale.y == 0.0  || marker.scale.z == 0.0)
  {
    if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST)
    {
      addCommaIfRequired(ss);
      ss << "scale contains 0.0 in x, y or z, TRIANGLE_LIST coordinates are scaled";
      return;
    }
    addCommaIfRequired(ss);
    ss << "scale contains 0.0 in x, y or z";
  }
}

void checkScaleLineStripAndList(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.scale.x == 0.0)
  {
    addCommaIfRequired(ss);
    ss << "width LINE_LIST or LINE_STRIP is 0.0 (scale.x)";
  }
  else if(marker.scale.y != 0.0 || marker.scale.z != 0.0)
  {
    addCommaIfRequired(ss);
    ss << "scale.y and scale.z of LINE_LIST or LINE_STRIP are ignored";
  }
}

void checkScalePoints(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.scale.x == 0.0 || marker.scale.y == 0.0)
  {
    addCommaIfRequired(ss);
    ss << "width and/or height of POINTS is 0.0 (scale.x, scale.y)";
  }
  else if(marker.scale.z != 0.0)
  {
    addCommaIfRequired(ss);
    ss << "scale.z of POINTS is ignored";
  }
}

void checkScaleText(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.scale.z == 0.0)
  {
    addCommaIfRequired(ss);
    ss << "text height of TEXT_VIEW_FACING is 0.0 (scale.z)";
  }
  else if(marker.scale.x != 0.0 || marker.scale.y != 0.0)
  {
    addCommaIfRequired(ss);
    ss << "scale.x and scale.y of TEXT_VIEW_FACING are ignored";
  }
}

void checkColor(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.color.a == 0.0)
  {
    addCommaIfRequired(ss);
    ss << "marker is fully transparent (color.a is 0.0)";
  }
}

void checkPointsArrow(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.points.size() != 0 && marker.points.size() != 2)
  {
    addCommaIfRequired(ss);
    ss << "Number of points for an ARROW marker should be either 0 or 2";
  }
}

void checkPointsNotEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.points.empty())
  {
    addCommaIfRequired(ss);
    ss << "points should not be empty for specified marker type" ;
  }
  else if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST && (marker.points.size() % 3) != 0)
  {
    addCommaIfRequired(ss);
    ss << "number of points should be a multiple of 3 for TRIANGLE_LIST Marker";
  }
  else if(marker.type == visualization_msgs::Marker::LINE_LIST && (marker.points.size() % 2) != 0)
  {
    addCommaIfRequired(ss);
    ss << "number of points should be a multiple of 2 for LINE_LIST Marker";
  }
  else if(marker.type == visualization_msgs::Marker::LINE_STRIP && marker.points.size() <= 1)
  {
    addCommaIfRequired(ss);
    ss << "at least two points are required for a LINE_STRIP Marker";
  }
}

void checkPointsEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(!marker.points.empty())
  {
    addCommaIfRequired(ss);
    ss << "points array is ignored by specified marker type";
  }
}

void checkColors(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.colors.size() == 0)
  {
    checkColor(marker, ss);
  }

  if(marker.colors.size() != marker.points.size())
  {
    addCommaIfRequired(ss);
    ss << "number of colors is not equal to number of points or 0";
  }
}

void checkColorsEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(!marker.colors.empty())
  {
    addCommaIfRequired(ss);
    ss << "colors array is ignored by specified marker type";
  }
}

void checkTextNotEmptyOrWhitespace(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.text.empty())
  {
    addCommaIfRequired(ss);
    ss << "text is empty for TEXT_VIEW_FACING type marker";
  }
  else if(marker.text.find_first_not_of(" \t\n\v\f\r") == std::string::npos)
  {
    addCommaIfRequired(ss);
    ss << "text of TEXT_VIEW_FACING Marker only consists of whitespaces";
  }
}

void checkTextEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(!marker.text.empty())
  {
    addCommaIfRequired(ss);
    ss << "text is ignored for specified marker type";
  }
}

void checkMesh(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if(marker.mesh_resource.empty())
  {
    addCommaIfRequired(ss);
    ss << "path to mesh resource is empty for MESH_RESOURCE marker";
  }
}

void checkMeshEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss)
{
  if (!marker.mesh_resource.empty())
  {
    addCommaIfRequired(ss);
    ss << "mesh_resource is ignored for specified marker type";
  }
  if (marker.mesh_use_embedded_materials)
  {
    addCommaIfRequired(ss);
    ss << "using embedded materials is not supported for markers other than MESH_RESOURCE";
  }
}

void addCommaIfRequired(std::stringstream& ss)
{
  if (ss.tellp() != 0) // check if string is not empty
  {
    ss << ", ";
  }
}

} // namespace rviz
