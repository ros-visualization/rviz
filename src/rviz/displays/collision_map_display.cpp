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
 *
 *
 */

#include "collision_map_display.h"
#include "visualization_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

namespace rviz
{

CollisionMapDisplay::CollisionMapDisplay(const std::string & name, VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
, render_operation_(collision_render_ops::CBoxes)
, override_color_(false)
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Collision Map" << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  cloud_ = new ogre_tools::PointCloud();
  setAlpha(1.0f);
  setPointSize(0.05f);
  scene_node_->attachObject(cloud_);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&CollisionMapDisplay::incomingMessage, this, _1));
}

CollisionMapDisplay::~CollisionMapDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject(manual_object_);

  delete cloud_;
}

void CollisionMapDisplay::clear()
{
  manual_object_->clear();
  cloud_->clear();
}

void CollisionMapDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void CollisionMapDisplay::setColor(const Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void CollisionMapDisplay::setOverrideColor(bool override)
{
  override_color_ = override;

  propertyChanged(override_color_property_);

  processMessage(current_message_);
  causeRender();
}

void CollisionMapDisplay::setRenderOperation(int op)
{
  render_operation_ = op;

  propertyChanged(render_operation_property_);

  processMessage(current_message_);
  causeRender();
}

void CollisionMapDisplay::setPointSize(float size)
{
  point_size_ = size;

  propertyChanged(point_size_property_);

  cloud_->setDimensions(size, size, size);
  causeRender();
}

void CollisionMapDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;
  cloud_->setAlpha(alpha);

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void CollisionMapDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void CollisionMapDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void CollisionMapDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void CollisionMapDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible(false);
}

void CollisionMapDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
  clear();
}

void CollisionMapDisplay::update(float wall_dt, float ros_dt)
{
}

void CollisionMapDisplay::processMessage(const mapping_msgs::CollisionMap::ConstPtr& msg)
{
  clear();

  if (!msg)
  {
    return;
  }

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, 0.0f)), msg->header.stamp,
      msg->header.frame_id);

  try
  {
    vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame 'map' to frame '%s'",
        fixed_frame_.c_str());
  }

  Ogre::Vector3 position = Ogre::Vector3(pose.getOrigin().x(),
      pose.getOrigin().y(), pose.getOrigin().z());
  robotToOgre(position);

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX(yaw, pitch, roll);

  Ogre::Matrix3 orientation(ogreMatrixFromRobotEulers(yaw, pitch, roll));

  manual_object_->clear();

  Ogre::ColourValue color;

  uint32_t num_boxes = msg->get_boxes_size();
  ROS_DEBUG("Collision map contains %d boxes.", num_boxes);

  // If we render points, we don't care about the order
  if (render_operation_ == collision_render_ops::CPoints)
  {
    typedef std::vector<ogre_tools::PointCloud::Point> V_Point;
    V_Point points;
    if (num_boxes > 0)
    {
      points.resize(num_boxes);
      for (uint32_t i = 0; i < num_boxes; i++)
      {
        ogre_tools::PointCloud::Point & current_point = points[i];

        current_point.x = msg->boxes[i].center.x;
        current_point.y = msg->boxes[i].center.y;
        current_point.z = msg->boxes[i].center.z;
        color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
        current_point.setColor(color.r, color.g, color.b);
      }
    }
    cloud_->clear();

    if (!points.empty())
    {
      cloud_->addPoints(&points.front(), points.size());
    }
  }
  else
  {
    robot_msgs::Point32 center, extents;
    color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
    if (num_boxes > 0)
    {
      for (uint32_t i = 0; i < num_boxes; i++)
      {
        manual_object_->estimateVertexCount(8);
        manual_object_->begin("BaseWhiteNoLighting",
            Ogre::RenderOperation::OT_LINE_STRIP);
        center.x = msg->boxes[i].center.x;
        center.y = msg->boxes[i].center.y;
        center.z = msg->boxes[i].center.z;
        extents.x = msg->boxes[i].extents.x;
        extents.y = msg->boxes[i].extents.y;
        extents.z = msg->boxes[i].extents.z;

        manual_object_->position(center.x - extents.x, center.y - extents.y,
            center.z - extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x - extents.x, center.y + extents.y,
            center.z - extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x + extents.x, center.y + extents.y,
            center.z - extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x + extents.x, center.y - extents.y,
            center.z - extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x + extents.x, center.y - extents.y,
            center.z + extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x + extents.x, center.y + extents.y,
            center.z + extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x - extents.x, center.y + extents.y,
            center.z + extents.z);
        manual_object_->colour(color);
        manual_object_->position(center.x - extents.x, center.y - extents.y,
            center.z + extents.z);
        manual_object_->colour(color);
        manual_object_->end();
      }
    }
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void CollisionMapDisplay::incomingMessage(const mapping_msgs::CollisionMap::ConstPtr& message)
{
  processMessage(message);
}

void CollisionMapDisplay::reset()
{
  clear();
}

void CollisionMapDisplay::targetFrameChanged()
{
}

void CollisionMapDisplay::createProperties()
{
  override_color_property_ = property_manager_->createProperty<BoolProperty> (
      "Override Color", property_prefix_, boost::bind(
          &CollisionMapDisplay::getOverrideColor, this), boost::bind(
          &CollisionMapDisplay::setOverrideColor, this, _1), category_,
      this);
  color_property_ = property_manager_->createProperty<ColorProperty> ("Color",
      property_prefix_, boost::bind(&CollisionMapDisplay::getColor, this),
      boost::bind(&CollisionMapDisplay::setColor, this, _1), category_,
      this);
  render_operation_property_
      = property_manager_->createProperty<EnumProperty> ("Render Operation",
          property_prefix_, boost::bind(
              &CollisionMapDisplay::getRenderOperation, this), boost::bind(
              &CollisionMapDisplay::setRenderOperation, this, _1),
          category_, this);
  EnumPropertyPtr render_prop = render_operation_property_.lock();
  render_prop->addOption("Boxes", collision_render_ops::CBoxes);
  render_prop->addOption("Points", collision_render_ops::CPoints);

  alpha_property_ = property_manager_->createProperty<FloatProperty> ("Alpha",
      property_prefix_, boost::bind(&CollisionMapDisplay::getAlpha, this),
      boost::bind(&CollisionMapDisplay::setAlpha, this, _1), category_,
      this);
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty> (
      "Topic", property_prefix_, boost::bind(&CollisionMapDisplay::getTopic,
          this), boost::bind(&CollisionMapDisplay::setTopic, this, _1),
      category_, this);
  ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(
      mapping_msgs::CollisionMap::__s_getDataType());
}

const char*
CollisionMapDisplay::getDescription()
{
  return ("Displays data from a mapping_msgs::CollisionMap message as either points or lines.");
}

} // namespace rviz
