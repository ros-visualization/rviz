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
#include <rviz/default_plugin/markers/shape_marker.h>
#include <rviz/default_plugin/markers/arrow_marker.h>
#include <rviz/default_plugin/markers/line_list_marker.h>
#include <rviz/default_plugin/markers/line_strip_marker.h>
#include <rviz/default_plugin/marker_display.h>
#include <rviz/default_plugin/markers/points_marker.h>
#include <rviz/default_plugin/markers/text_view_facing_marker.h>
#include <rviz/default_plugin/markers/mesh_resource_marker.h>
#include <rviz/default_plugin/markers/triangle_list_marker.h>
#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/status_list.h>
#include <rviz/validate_quaternions.h>
#include <rviz/validate_floats.h>

namespace rviz
{
MarkerBase*
createMarker(int marker_type, MarkerDisplay* owner, DisplayContext* context, Ogre::SceneNode* parent_node)
{
  switch (marker_type)
  {
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

QString getMarkerTypeName(unsigned int type)
{
  switch (type)
  {
  case visualization_msgs::Marker::ARROW:
    return "Arrow";
  case visualization_msgs::Marker::CUBE:
    return "Cube";
  case visualization_msgs::Marker::CUBE_LIST:
    return "Cube List";
  case visualization_msgs::Marker::TRIANGLE_LIST:
    return "Triangle List";
  case visualization_msgs::Marker::SPHERE:
    return "Sphere";
  case visualization_msgs::Marker::SPHERE_LIST:
    return "Sphere List";
  case visualization_msgs::Marker::CYLINDER:
    return "Cylinder";
  case visualization_msgs::Marker::LINE_STRIP:
    return "Line Strip";
  case visualization_msgs::Marker::LINE_LIST:
    return "Line List";
  case visualization_msgs::Marker::POINTS:
    return "Points";
  case visualization_msgs::Marker::TEXT_VIEW_FACING:
    return "Text View Facing";
  case visualization_msgs::Marker::MESH_RESOURCE:
    return "Mesh";
  default:
    return "Unknown";
  }
}

namespace
{
void addSeparatorIfRequired(std::stringstream& ss)
{
  if (ss.tellp() != 0) // check if string is not empty
  {
    ss << " \n";
  }
}

void increaseLevel(::ros::console::levels::Level new_status,
                   ::ros::console::levels::Level& current_status)
{
  if (new_status > current_status)
    current_status = new_status;
}

template <typename T>
constexpr const char* fieldName();
template <>
constexpr const char* fieldName<::geometry_msgs::Point>()
{
  return "Position";
}
template <>
constexpr const char* fieldName<::geometry_msgs::Quaternion>()
{
  return "Orientation";
}
template <>
constexpr const char* fieldName<::geometry_msgs::Vector3>()
{
  return "Scale";
}
template <>
constexpr const char* fieldName<::std_msgs::ColorRGBA>()
{
  return "Color";
}

template <typename T>
void checkFloats(const T& t, std::stringstream& ss, ::ros::console::levels::Level& level)
{
  if (!validateFloats(t))
  {
    addSeparatorIfRequired(ss);
    ss << fieldName<T>() << " contains invalid floating point values (nans or infs)";
    increaseLevel(::ros::console::levels::Error, level);
  }
}

void checkQuaternion(const visualization_msgs::Marker& marker,
                     std::stringstream& ss,
                     ::ros::console::levels::Level& level)
{
  checkFloats(marker.pose.orientation, ss, level);
  if (marker.pose.orientation.x == 0.0 && marker.pose.orientation.y == 0.0 &&
      marker.pose.orientation.z == 0.0 && marker.pose.orientation.w == 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "Uninitialized quaternion, assuming identity.";
    increaseLevel(::ros::console::levels::Info, level);
  }
  else if (!validateQuaternions(marker.pose))
  {
    addSeparatorIfRequired(ss);
    ss << "Unnormalized quaternion in marker message.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkScale(const visualization_msgs::Marker& marker,
                std::stringstream& ss,
                ::ros::console::levels::Level& level)
{
  checkFloats(marker.scale, ss, level);
  // for ARROW markers, scale.z is the optional head length
  if (marker.type == visualization_msgs::Marker::ARROW && marker.scale.x != 0.0 && marker.scale.y != 0.0)
    return;

  if (marker.scale.x == 0.0 || marker.scale.y == 0.0 || marker.scale.z == 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "Scale contains 0.0 in x, y or z.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkScaleLineStripAndList(const visualization_msgs::Marker& marker,
                                std::stringstream& ss,
                                ::ros::console::levels::Level& level)
{
  if (marker.scale.x == 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "Width LINE_LIST or LINE_STRIP is 0.0 (scale.x).";
    increaseLevel(::ros::console::levels::Warn, level);
  }
  else if (marker.scale.y != 0.0 || marker.scale.z != 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "scale.y and scale.z of LINE_LIST or LINE_STRIP are ignored.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkScalePoints(const visualization_msgs::Marker& marker,
                      std::stringstream& ss,
                      ::ros::console::levels::Level& level)
{
  if (marker.scale.x == 0.0 || marker.scale.y == 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "Width and/or height of POINTS is 0.0 (scale.x, scale.y).";
    increaseLevel(::ros::console::levels::Warn, level);
  }
  else if (marker.scale.z != 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "scale.z of POINTS is ignored.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkScaleText(const visualization_msgs::Marker& marker,
                    std::stringstream& ss,
                    ::ros::console::levels::Level& level)
{
  if (marker.scale.z == 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "Text height of TEXT_VIEW_FACING is 0.0 (scale.z).";
    increaseLevel(::ros::console::levels::Warn, level);
  }
  else if (marker.scale.x != 0.0 || marker.scale.y != 0.0)
  {
    addSeparatorIfRequired(ss);
    ss << "scale.x and scale.y of TEXT_VIEW_FACING are ignored.";
    increaseLevel(::ros::console::levels::Debug, level);
  }
}

void checkColor(const visualization_msgs::Marker& marker,
                std::stringstream& ss,
                ::ros::console::levels::Level& level)
{
  checkFloats(marker.color, ss, level);
  if (marker.color.a == 0.0 &&
      // Mesh markers use a color of (0,0,0,0) to indicate that the original color of the mesh should be
      // used as is
      !(marker.type == visualization_msgs::Marker::MESH_RESOURCE && marker.mesh_use_embedded_materials &&
        marker.color.r == 0.0 && marker.color.g == 0.0 && marker.color.b == 0.0))
  {
    addSeparatorIfRequired(ss);
    ss << "Marker is fully transparent (color.a is 0.0).";
    increaseLevel(::ros::console::levels::Info, level);
  }
}

void checkPointsArrow(const visualization_msgs::Marker& marker,
                      std::stringstream& ss,
                      ::ros::console::levels::Level& level)
{
  if (!marker.points.empty() && marker.points.size() != 2)
  {
    addSeparatorIfRequired(ss);
    ss << "Number of points for an ARROW marker should be either 0 or 2.";
    increaseLevel(::ros::console::levels::Error, level);
    return;
  }
  for (const auto& p : marker.points)
    checkFloats(p, ss, level);
}

void checkPointsNotEmpty(const visualization_msgs::Marker& marker,
                         std::stringstream& ss,
                         ::ros::console::levels::Level& level)
{
  if (marker.points.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Points should not be empty for specified marker type.";
    increaseLevel(::ros::console::levels::Error, level);
  }
  switch (marker.type)
  {
  case visualization_msgs::Marker::TRIANGLE_LIST:
    if (marker.points.size() % 3 != 0)
    {
      addSeparatorIfRequired(ss);
      ss << "Number of points should be a multiple of 3 for TRIANGLE_LIST marker.";
      increaseLevel(::ros::console::levels::Error, level);
    }
    break;
  case visualization_msgs::Marker::LINE_LIST:
    if (marker.points.size() % 2 != 0)
    {
      addSeparatorIfRequired(ss);
      ss << "Number of points should be a multiple of 2 for LINE_LIST marker.";
      increaseLevel(::ros::console::levels::Error, level);
    }
    break;
  case visualization_msgs::Marker::LINE_STRIP:
    if (marker.points.size() <= 1)
    {
      addSeparatorIfRequired(ss);
      ss << "At least two points are required for a LINE_STRIP marker.";
      increaseLevel(::ros::console::levels::Error, level);
    }
    break;
  default:
    break;
  }
}

void checkPointsEmpty(const visualization_msgs::Marker& marker,
                      std::stringstream& ss,
                      ::ros::console::levels::Level& level)
{
  if (!marker.points.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Non-empty points array is ignored.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkColors(const visualization_msgs::Marker& marker,
                 std::stringstream& ss,
                 ::ros::console::levels::Level& level,
                 unsigned int multiple)
{
  if (marker.colors.empty())
  {
    checkColor(marker, ss, level);
    return;
  }
  if (marker.colors.size() != marker.points.size() &&
      (multiple == 1 || multiple * marker.colors.size() != marker.points.size()))
  {
    addSeparatorIfRequired(ss);
    ss << "Number of colors doesn't match number of points.";
    increaseLevel(::ros::console::levels::Error, level);
  }
}

void checkColorsEmpty(const visualization_msgs::Marker& marker,
                      std::stringstream& ss,
                      ::ros::console::levels::Level& level)
{
  if (!marker.colors.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Non-empty colors array is ignored.";
    increaseLevel(::ros::console::levels::Warn, level);
  }
}

void checkTextNotEmptyOrWhitespace(const visualization_msgs::Marker& marker,
                                   std::stringstream& ss,
                                   ::ros::console::levels::Level& level)
{
  if (marker.text.find_first_not_of(" \t\n\v\f\r") == std::string::npos)
  {
    addSeparatorIfRequired(ss);
    ss << "Text is empty or only consists of whitespace.";
    increaseLevel(::ros::console::levels::Error, level);
  }
}

void checkTextEmpty(const visualization_msgs::Marker& marker,
                    std::stringstream& ss,
                    ::ros::console::levels::Level& level)
{
  if (!marker.text.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Non-empty text is ignored.";
    increaseLevel(::ros::console::levels::Info, level);
  }
}

void checkMesh(const visualization_msgs::Marker& marker,
               std::stringstream& ss,
               ::ros::console::levels::Level& level)
{
  if (marker.mesh_resource.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Empty mesh_resource path.";
    increaseLevel(::ros::console::levels::Error, level);
  }
}

void checkMeshEmpty(const visualization_msgs::Marker& marker,
                    std::stringstream& ss,
                    ::ros::console::levels::Level& level)
{
  if (!marker.mesh_resource.empty())
  {
    addSeparatorIfRequired(ss);
    ss << "Non-empty mesh_resource is ignored.";
    increaseLevel(::ros::console::levels::Info, level);
  }
  if (marker.mesh_use_embedded_materials)
  {
    addSeparatorIfRequired(ss);
    ss << "mesh_use_embedded_materials is ignored.";
    increaseLevel(::ros::console::levels::Info, level);
  }
}

} // namespace

bool checkMarkerMsg(const visualization_msgs::Marker& marker, MarkerDisplay* owner)
{
  if (marker.action != visualization_msgs::Marker::ADD)
    return true;

  std::stringstream ss;
  ::ros::console::levels::Level level = ::ros::console::levels::Debug;
  checkFloats(marker.pose.position, ss, level);

  switch (marker.type)
  {
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
    checkColors(marker, ss, level, 1);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::SPHERE_LIST:
  case visualization_msgs::Marker::CUBE_LIST:
  case visualization_msgs::Marker::TRIANGLE_LIST:
    checkQuaternion(marker, ss, level);
    checkScale(marker, ss, level);
    checkPointsNotEmpty(marker, ss, level);
    checkColors(marker, ss, level, marker.type == visualization_msgs::Marker::TRIANGLE_LIST ? 3 : 1);
    checkTextEmpty(marker, ss, level);
    checkMeshEmpty(marker, ss, level);
    break;

  case visualization_msgs::Marker::POINTS:
    checkScalePoints(marker, ss, level);
    checkPointsNotEmpty(marker, ss, level);
    checkColors(marker, ss, level, 1);
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
    checkMesh(marker, ss, level);
    break;

  default:
    ss << "Unknown marker type: " << marker.type << '.';
    level = ::ros::console::levels::Error;
  }

  if (ss.tellp() != 0) // stringstream is not empty
  {
    std::string warning = ss.str();

    bool fatal = level >= ::ros::console::levels::Error;
    if (owner)
      owner->setMarkerStatus(MarkerID(marker.ns, marker.id),
                             fatal ? StatusProperty::Error : StatusProperty::Warn, warning);
    else
      ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME ".marker", "Marker '%s/%d': %s", marker.ns.c_str(),
              marker.id, warning.c_str());

    return !fatal;
  }

  if (owner)
    owner->deleteMarkerStatus(MarkerID(marker.ns, marker.id));
  return true;
}

bool checkMarkerArrayMsg(const visualization_msgs::MarkerArray& array, MarkerDisplay* owner)
{
  std::vector<MarkerID> marker_ids;
  marker_ids.reserve(array.markers.size());

  ::ros::console::levels::Level level = ::ros::console::levels::Debug;
  std::stringstream ss;

  for (const visualization_msgs::Marker& marker : array.markers)
  {
    if (marker.action == visualization_msgs::Marker::DELETEALL)
    {
      if (!marker_ids.empty())
      {
        addSeparatorIfRequired(ss);
        ss << "DELETEALL after having markers added. These will never show.";
        increaseLevel(::ros::console::levels::Warn, level);
      }
      marker_ids.clear();
      continue;
    }

    MarkerID current_id(marker.ns, marker.id);
    std::vector<MarkerID>::iterator search =
        std::lower_bound(marker_ids.begin(), marker_ids.end(), current_id);
    if (search != marker_ids.end() && *search == current_id)
    {
      addSeparatorIfRequired(ss);
      increaseLevel(::ros::console::levels::Warn, level);
      if (marker.action == visualization_msgs::Marker::DELETE)
      {
        ss << "Deleting just added marker '" << marker.ns.c_str() << "/" << marker.id << "'.";
        marker_ids.erase(search);
      }
      else
        ss << "Adding marker '" << marker.ns.c_str() << "/" << marker.id << "' multiple times.";
    }
    else
    {
      marker_ids.insert(search, current_id);
    }
  }

  if (ss.tellp() != 0) // stringstream is not empty
  {
    std::string warning = ss.str();

    bool fatal = level >= ::ros::console::levels::Error;
    if (owner)
      owner->setStatusStd(fatal ? StatusProperty::Error : StatusProperty::Warn, "marker_array", warning);
    else
      ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME ".marker", "MarkerArray: %s", warning.c_str());

    return !fatal;
  }

  if (owner)
    owner->deleteStatusStd("marker_array");
  return true;
}

} // namespace rviz
