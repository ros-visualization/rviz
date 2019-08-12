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
  StatusProperty::Level level = StatusProperty::Ok;

  switch (marker.type) {
  case visualization_msgs::Marker::ARROW:
    checkQuaternion(marker, ss, level);
    checkScale(marker, ss, level);
    checkColor(marker, ss, level);
    checkPointsArrow(marker, ss, level);
    checkColorsEmpty(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::CUBE:
  case visualization_msgs::Marker::CYLINDER:
  case visualization_msgs::Marker::SPHERE:
    checkQuaternion(marker, ss, level);
    checkScale(marker, ss, level);
    checkColor(marker, ss, level);
    checkPointsEmpty(marker, ss, level);
    checkColorsEmpty(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;


  case visualization_msgs::Marker::LINE_STRIP:
  case visualization_msgs::Marker::LINE_LIST:
    checkQuaternion(marker, ss, level);
    checkScaleLineStripAndList(marker, ss, level);
    checkPointsNotEmpty(marker, ss, level);
    checkColors(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::SPHERE_LIST:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::TRIANGLE_LIST:
    checkQuaternion(marker, ss, level);
    checkScale(marker, ss, level);
    checkPointsNotEmpty(marker, ss, level);
    checkColors(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::POINTS:
    checkScalePoints(marker, ss, level);
    checkPointsNotEmpty(marker, ss, level);
    checkColors(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    checkColor(marker, ss, level);
    checkScaleText(marker, ss, level);
    checkTextNotEmptyOrWhitespace(marker, ss, level);
    checkPointsEmpty(marker, ss, level);
    checkColorsEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::MESH_RESOURCE:
    checkQuaternion(marker, ss, level);
    checkColor(marker, ss, level);
    checkScale(marker, ss, level);
    checkPointsEmpty(marker, ss, level);
    checkColorsEmpty(marker, ss, level);
    checkTextEmpty(marker, ss, level);
    break;

  default:
    ss << "Unknown marker type: " << marker.type ;
    level = StatusProperty::Error;
  }

  if(ss.tellp() != 0) //stringstream is not empty
  {
    std::string warning = ss.str();

    if (owner)
      owner->setMarkerStatus(MarkerID(marker.ns, marker.id), level, warning);
    ROS_LOG((level == StatusProperty::Warn ? ::ros::console::levels::Warn : ::ros::console::levels::Error),
            ROSCONSOLE_DEFAULT_NAME, "Marker '%s/%d': %s", marker.ns.c_str(), marker.id, warning.c_str());

    return false;
  }

  if (owner)
    owner->deleteMarkerStatus(MarkerID(marker.ns, marker.id));
  return true;
}

bool checkMarkerArrayMsg(const visualization_msgs::MarkerArray& array, MarkerDisplay* owner)
{
  std::vector<MarkerID> marker_ids;
  marker_ids.reserve(array.markers.size());

  bool add_marker_in_array = false;
  bool reset_status = true;
  for(const visualization_msgs::Marker& marker : array.markers)
  {
    if(marker.action == visualization_msgs::Marker::ADD)
      add_marker_in_array = true;

    if(add_marker_in_array && marker.action == visualization_msgs::Marker::DELETEALL)
    {
      const std::string warning = "found a DELETEALL after having markers added. These markers will never show";
      ROS_ERROR("MarkerArray: %s", warning.c_str());
      if (owner)
        owner->setStatusStd(StatusProperty::Error, "marker_array", warning);
      reset_status = false;
    }

    MarkerID current_id(marker.ns, marker.id);
    std::vector<MarkerID>::iterator search = std::lower_bound(marker_ids.begin(), marker_ids.end(), current_id);
    if (search != marker_ids.end())
    {
      std::stringstream ss;
      ss << "found '" <<  marker.ns.c_str() << "/" << marker.id << "' multiple times";
      const std::string ss_str = ss.str();
      ROS_WARN("MarkerArray: %s", ss_str.c_str());
      if (owner)
        owner->setStatusStd(StatusProperty::Warn, "marker_array", ss_str);
      reset_status = false;
    }
    else
    {
      marker_ids.insert(search, current_id);
    }
  }

  if (reset_status && owner)
    owner->setStatusStd(StatusProperty::Ok, "marker_array", "OK");

  return reset_status;
}


void checkQuaternion(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if (marker.pose.orientation.x == 0.0 && marker.pose.orientation.y == 0.0 && marker.pose.orientation.z == 0.0 &&
      marker.pose.orientation.w == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "uninitialized quaternion assuming identity";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
  else if(!validateQuaternions(marker.pose))
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "unnormalized quaternion in marker message";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkScale(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  // for ARROW markers, scale.z is the optional head length
  if(marker.type == visualization_msgs::Marker::ARROW && marker.scale.x != 0.0 && marker.scale.y != 0.0)
    return;

  if(marker.scale.x == 0.0  || marker.scale.y == 0.0  || marker.scale.z == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "scale contains 0.0 in x, y or z";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkScaleLineStripAndList(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.scale.x == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "width LINE_LIST or LINE_STRIP is 0.0 (scale.x)";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.scale.y != 0.0 || marker.scale.z != 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "scale.y and scale.z of LINE_LIST or LINE_STRIP are ignored";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkScalePoints(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.scale.x == 0.0 || marker.scale.y == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "width and/or height of POINTS is 0.0 (scale.x, scale.y)";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.scale.z != 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "scale.z of POINTS is ignored";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkScaleText(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.scale.z == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "text height of TEXT_VIEW_FACING is 0.0 (scale.z)";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.scale.x != 0.0 || marker.scale.y != 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "scale.x and scale.y of TEXT_VIEW_FACING are ignored";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkColor(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.color.a == 0.0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "marker is fully transparent (color.a is 0.0)";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkPointsArrow(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.points.size() != 0 && marker.points.size() != 2)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "Number of points for an ARROW marker should be either 0 or 2";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkPointsNotEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.points.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "points should not be empty for specified marker type" ;
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST && (marker.points.size() % 3) != 0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "number of points should be a multiple of 3 for TRIANGLE_LIST Marker";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.type == visualization_msgs::Marker::LINE_LIST && (marker.points.size() % 2) != 0)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "number of points should be a multiple of 2 for LINE_LIST Marker";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.type == visualization_msgs::Marker::LINE_STRIP && marker.points.size() <= 1)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "at least two points are required for a LINE_STRIP Marker";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkPointsEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(!marker.points.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "points array is ignored by specified marker type";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkColors(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.colors.size() == 0)
  {
    checkColor(marker, ss, level);
    return;
  }
  if(marker.colors.size() != marker.points.size())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "number of colors is not equal to number of points or 0";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkColorsEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(!marker.colors.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "colors array is ignored by specified marker type";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkTextNotEmptyOrWhitespace(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.text.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "text is empty for TEXT_VIEW_FACING type marker";
    increaseWarningLevel(StatusProperty::Error, level);
  }
  else if(marker.text.find_first_not_of(" \t\n\v\f\r") == std::string::npos)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "text of TEXT_VIEW_FACING Marker only consists of whitespaces";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkTextEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(!marker.text.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "text is ignored for specified marker type";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void checkMesh(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if(marker.mesh_resource.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "path to mesh resource is empty for MESH_RESOURCE marker";
    increaseWarningLevel(StatusProperty::Error, level);
  }
}

void checkMeshEmpty(const visualization_msgs::Marker& marker, std::stringstream& ss, StatusProperty::Level& level)
{
  if (!marker.mesh_resource.empty())
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "mesh_resource is ignored for specified marker type";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
  if (marker.mesh_use_embedded_materials)
  {
    addCommaAndNewlineIfRequired(ss);
    ss << "using embedded materials is not supported for markers other than MESH_RESOURCE";
    increaseWarningLevel(StatusProperty::Warn, level);
  }
}

void addCommaAndNewlineIfRequired(std::stringstream& ss)
{
  if (ss.tellp() != 0) // check if string is not empty
  {
    ss << ",\n";
  }
}

void increaseWarningLevel(StatusProperty::Level new_status, StatusProperty::Level& current_status)
{
    if(new_status > current_status)
        current_status = new_status;
}
} // namespace rviz
