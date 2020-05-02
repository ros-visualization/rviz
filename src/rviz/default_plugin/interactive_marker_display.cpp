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

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/validate_quaternions.h"
#include "rviz/display_context.h"

#include "rviz/default_plugin/interactive_marker_display.h"


namespace rviz
{
//////////////
bool validateFloats(const visualization_msgs::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.pose);
  valid = valid && validateFloats(msg.scale);
  for (unsigned c = 0; c < msg.controls.size(); c++)
  {
    valid = valid && validateFloats(msg.controls[c].orientation);
    for (unsigned m = 0; m < msg.controls[c].markers.size(); m++)
    {
      valid = valid && validateFloats(msg.controls[c].markers[m].pose);
      valid = valid && validateFloats(msg.controls[c].markers[m].scale);
      valid = valid && validateFloats(msg.controls[c].markers[m].color);
      valid = valid && validateFloats(msg.controls[c].markers[m].points);
    }
  }
  return valid;
}

bool validateQuaternions(const visualization_msgs::InteractiveMarker& marker)
{
  if (!validateQuaternions(marker.pose.orientation))
    return false;
  for (size_t c = 0; c < marker.controls.size(); ++c)
  {
    if (!validateQuaternions(marker.controls[c].orientation))
      return false;
    for (size_t m = 0; m < marker.controls[c].markers.size(); ++m)
    {
      if (!validateQuaternions(marker.controls[c].markers[m].pose.orientation))
        return false;
    }
  }
  return true;
}
/////////////


InteractiveMarkerDisplay::InteractiveMarkerDisplay() : Display()
{
  marker_update_topic_property_ = new RosTopicProperty(
      "Update Topic", "", ros::message_traits::datatype<visualization_msgs::InteractiveMarkerUpdate>(),
      "visualization_msgs::InteractiveMarkerUpdate topic to subscribe to.", this, SLOT(updateTopic()));

  show_descriptions_property_ =
      new BoolProperty("Show Descriptions", true,
                       "Whether or not to show the descriptions of each Interactive Marker.", this,
                       SLOT(updateShowDescriptions()));

  show_axes_property_ =
      new BoolProperty("Show Axes", false, "Whether or not to show the axes of each Interactive Marker.",
                       this, SLOT(updateShowAxes()));

  show_visual_aids_property_ = new BoolProperty(
      "Show Visual Aids", false,
      "Whether or not to show visual helpers while moving/rotating Interactive Markers.", this,
      SLOT(updateShowVisualAids()));
  enable_transparency_property_ = new BoolProperty(
      "Enable Transparency", true,
      "Whether or not to allow transparency for auto-completed markers (e.g. rings and arrows).", this,
      SLOT(updateEnableTransparency()));
}

void InteractiveMarkerDisplay::onInitialize()
{
// TODO(wjwwood): remove this and use tf2 interface instead
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

  tf::Transformer* tf = context_->getFrameManager()->getTFClient();

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
  im_client_.reset(new interactive_markers::InteractiveMarkerClient(*tf, fixed_frame_.toStdString()));

  im_client_->setInitCb(boost::bind(&InteractiveMarkerDisplay::initCb, this, _1));
  im_client_->setUpdateCb(boost::bind(&InteractiveMarkerDisplay::updateCb, this, _1));
  im_client_->setResetCb(boost::bind(&InteractiveMarkerDisplay::resetCb, this, _1));
  im_client_->setStatusCb(boost::bind(&InteractiveMarkerDisplay::statusCb, this, _1, _2, _3));

  client_id_ = ros::this_node::getName() + "/" + getNameStd();

  onEnable();
}

void InteractiveMarkerDisplay::setTopic(const QString& topic, const QString& /*datatype*/)
{
  marker_update_topic_property_->setString(topic);
}

void InteractiveMarkerDisplay::onEnable()
{
  subscribe();
}

void InteractiveMarkerDisplay::onDisable()
{
  unsubscribe();
}

void InteractiveMarkerDisplay::updateTopic()
{
  unsubscribe();

  std::string update_topic = marker_update_topic_property_->getTopicStd();

  size_t idx = update_topic.find("/update");
  if (idx != std::string::npos)
  {
    topic_ns_ = update_topic.substr(0, idx);
    subscribe();
  }
  else
  {
    setStatusStd(StatusProperty::Error, "Topic", "Invalid topic name: " + update_topic);
  }
}

void InteractiveMarkerDisplay::subscribe()
{
  if (isEnabled())
  {
    im_client_->subscribe(topic_ns_);

    std::string feedback_topic = topic_ns_ + "/feedback";
    feedback_pub_ =
        update_nh_.advertise<visualization_msgs::InteractiveMarkerFeedback>(feedback_topic, 100, false);
  }
}

void InteractiveMarkerDisplay::publishFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback)
{
  feedback.client_id = client_id_;
  feedback_pub_.publish(feedback);
}

void InteractiveMarkerDisplay::onStatusUpdate(StatusProperty::Level level,
                                              const std::string& name,
                                              const std::string& text)
{
  setStatusStd(level, name, text);
}

void InteractiveMarkerDisplay::unsubscribe()
{
  if (im_client_)
  {
    im_client_->shutdown();
  }
  feedback_pub_.shutdown();
  Display::reset();
}

void InteractiveMarkerDisplay::update(float wall_dt, float /*ros_dt*/)
{
  im_client_->update();

  M_StringToStringToIMPtr::iterator server_it;
  for (server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++)
  {
    M_StringToIMPtr::iterator im_it;
    for (im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++)
    {
      im_it->second->update(wall_dt);
    }
  }
}

InteractiveMarkerDisplay::M_StringToIMPtr& InteractiveMarkerDisplay::getImMap(std::string server_id)
{
  M_StringToStringToIMPtr::iterator im_map_it = interactive_markers_.find(server_id);

  if (im_map_it == interactive_markers_.end())
  {
    im_map_it = interactive_markers_.insert(std::make_pair(server_id, M_StringToIMPtr())).first;
  }

  return im_map_it->second;
}

void InteractiveMarkerDisplay::updateMarkers(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarker>& markers)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < markers.size(); i++)
  {
    const visualization_msgs::InteractiveMarker& marker = markers[i];

    if (!validateFloats(marker))
    {
      setStatusStd(StatusProperty::Error, marker.name, "Marker contains invalid floats!");
      // setStatusStd( StatusProperty::Error, "General", "Marker " + marker.name + " contains invalid
      // floats!" );
      continue;
    }

    if (!validateQuaternions(marker))
    {
      ROS_WARN_ONCE_NAMED("quaternions",
                          "Interactive marker '%s' contains unnormalized quaternions. "
                          "This warning will only be output once but may be true for others; "
                          "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                          marker.name.c_str());
      ROS_DEBUG_NAMED("quaternions", "Interactive marker '%s' contains unnormalized quaternions.",
                      marker.name.c_str());
    }
    ROS_DEBUG("Processing interactive marker '%s'. %d", marker.name.c_str(), (int)marker.controls.size());

    std::map<std::string, IMPtr>::iterator int_marker_entry = im_map.find(marker.name);

    if (int_marker_entry == im_map.end())
    {
      int_marker_entry =
          im_map
              .insert(std::make_pair(marker.name, IMPtr(new InteractiveMarker(getSceneNode(), context_))))
              .first;
      connect(int_marker_entry->second.get(),
              SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)), this,
              SLOT(publishFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
      connect(int_marker_entry->second.get(),
              SIGNAL(statusUpdate(StatusProperty::Level, const std::string&, const std::string&)), this,
              SLOT(onStatusUpdate(StatusProperty::Level, const std::string&, const std::string&)));
    }

    if (int_marker_entry->second->processMessage(marker))
    {
      int_marker_entry->second->setShowAxes(show_axes_property_->getBool());
      int_marker_entry->second->setShowVisualAids(show_visual_aids_property_->getBool());
      int_marker_entry->second->setShowDescription(show_descriptions_property_->getBool());
    }
    else
    {
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::eraseMarkers(const std::string& server_id,
                                            const std::vector<std::string>& erases)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < erases.size(); i++)
  {
    im_map.erase(erases[i]);
    deleteStatusStd(erases[i]);
  }
}

void InteractiveMarkerDisplay::updatePoses(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarkerPose>& marker_poses)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < marker_poses.size(); i++)
  {
    const visualization_msgs::InteractiveMarkerPose& marker_pose = marker_poses[i];

    if (!validateFloats(marker_pose.pose))
    {
      setStatusStd(StatusProperty::Error, marker_pose.name, "Pose message contains invalid floats!");
      return;
    }

    if (!validateQuaternions(marker_pose.pose))
    {
      setStatusStd(StatusProperty::Error, marker_pose.name,
                   "Pose message contains invalid quaternions (length not equal to 1)!");
      return;
    }

    std::map<std::string, IMPtr>::iterator int_marker_entry = im_map.find(marker_pose.name);

    if (int_marker_entry != im_map.end())
    {
      int_marker_entry->second->processMessage(marker_pose);
    }
    else
    {
      setStatusStd(StatusProperty::Error, marker_pose.name,
                   "Pose received for non-existing marker '" + marker_pose.name);
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::initCb(visualization_msgs::InteractiveMarkerInitConstPtr msg)
{
  resetCb(msg->server_id);
  updateMarkers(msg->server_id, msg->markers);
}

void InteractiveMarkerDisplay::updateCb(visualization_msgs::InteractiveMarkerUpdateConstPtr msg)
{
  updateMarkers(msg->server_id, msg->markers);
  updatePoses(msg->server_id, msg->poses);
  eraseMarkers(msg->server_id, msg->erases);
}

void InteractiveMarkerDisplay::resetCb(std::string server_id)
{
  interactive_markers_.erase(server_id);
  deleteStatusStd(server_id);
}

void InteractiveMarkerDisplay::statusCb(interactive_markers::InteractiveMarkerClient::StatusT status,
                                        const std::string& server_id,
                                        const std::string& msg)
{
  setStatusStd(static_cast<StatusProperty::Level>(status), server_id, msg);
}

void InteractiveMarkerDisplay::fixedFrameChanged()
{
  if (im_client_)
    im_client_->setTargetFrame(fixed_frame_.toStdString());
  reset();
}

void InteractiveMarkerDisplay::reset()
{
  Display::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::updateShowDescriptions()
{
  bool show = show_descriptions_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for (server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++)
  {
    M_StringToIMPtr::iterator im_it;
    for (im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++)
    {
      im_it->second->setShowDescription(show);
    }
  }
}

void InteractiveMarkerDisplay::updateShowAxes()
{
  bool show = show_axes_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for (server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++)
  {
    M_StringToIMPtr::iterator im_it;
    for (im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++)
    {
      im_it->second->setShowAxes(show);
    }
  }
}

void InteractiveMarkerDisplay::updateShowVisualAids()
{
  bool show = show_visual_aids_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for (server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++)
  {
    M_StringToIMPtr::iterator im_it;
    for (im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++)
    {
      im_it->second->setShowVisualAids(show);
    }
  }
}

void InteractiveMarkerDisplay::updateEnableTransparency()
{
  // This is not very efficient... but it should do the trick.
  unsubscribe();
  im_client_->setEnableAutocompleteTransparency(enable_transparency_property_->getBool());
  subscribe();
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::InteractiveMarkerDisplay, rviz::Display)
