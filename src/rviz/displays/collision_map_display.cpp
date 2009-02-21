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
 * $Id$
 *
 */

#include "collision_map_display.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/point_cloud.h>

namespace rviz
{

CollisionMapDisplay::CollisionMapDisplay(const std::string & name,
    VisualizationManager * manager) :
  Display(name, manager), color_(0.1f, 1.0f, 0.0f), render_operation_(
      collision_render_ops::CBoxes), override_color_(false), color_property_(
      NULL)
  , topic_property_(NULL)
  , override_color_property_(NULL)
  , render_operation_property_(NULL)
  , point_size_property_(NULL)
  , z_position_property_(NULL)
  , alpha_property_(NULL)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Collision Map" << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  cloud_ = new ogre_tools::PointCloud(scene_manager_, scene_node_);
  setAlpha(1.0f);
  setPointSize(0.05f);
  setZPosition(0.0f);

  notifier_ = new tf::MessageNotifier<robot_msgs::CollisionMap>(tf_,
      ros_node_, boost::bind(&CollisionMapDisplay::incomingMessage, this, _1),
      "", "", 1);
}

CollisionMapDisplay::~CollisionMapDisplay()
{
  unsubscribe();
  clear();

  delete notifier_;

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

  if (topic_property_)
    topic_property_->changed();

  causeRender();
}

void CollisionMapDisplay::setColor(const Color & color)
{
  color_ = color;

  if (color_property_)
    color_property_->changed();

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = CollisionMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void CollisionMapDisplay::setOverrideColor(bool override)
{
  override_color_ = override;

  if (override_color_property_)
    override_color_property_->changed();

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = CollisionMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void CollisionMapDisplay::setRenderOperation(int op)
{
  render_operation_ = op;

  if (render_operation_property_)
    render_operation_property_->changed();

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = CollisionMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void CollisionMapDisplay::setPointSize(float size)
{
  point_size_ = size;

  if (point_size_property_)
    point_size_property_->changed();

  cloud_->setBillboardDimensions(size, size);
  causeRender();
}

void CollisionMapDisplay::setZPosition(float z)
{
  z_position_ = z;

  if (z_position_property_)
    z_position_property_->changed();

  scene_node_->setPosition(0.0f, z, 0.0f);
  causeRender();
}

void CollisionMapDisplay::setAlpha(float alpha)
{
  alpha_ = alpha;
  cloud_->setAlpha(alpha);

  if (alpha_property_)
    alpha_property_->changed();

  message_mutex_.lock();
  new_message_ = current_message_;
  processMessage();
  new_message_ = CollisionMapPtr();
  message_mutex_.unlock();
  causeRender();
}

void CollisionMapDisplay::subscribe()
{
  if (!isEnabled())
    return;

  notifier_->setTopic(topic_);
}

void CollisionMapDisplay::unsubscribe()
{
  notifier_->setTopic("");
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
  notifier_->setTargetFrame(fixed_frame_);
  clear();
}

void CollisionMapDisplay::update(float dt)
{
  message_mutex_.lock();
  if (new_message_)
  {
    processMessage();
    current_message_ = new_message_;
    new_message_ = CollisionMapPtr();
    causeRender();
  }
  message_mutex_.unlock();
}

void CollisionMapDisplay::processMessage()
{
  if (!new_message_)
  {
    return;
  }

  clear();

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, z_position_)), new_message_->header.stamp,
      new_message_->header.frame_id);

  try
  {
    tf_->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame 'map' to frame '%s'\n",
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

  uint32_t num_boxes = new_message_->get_boxes_size();
  ROS_INFO("Collision map contains %d boxes.", num_boxes);

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

        current_point.x_ = new_message_->boxes[i].center.x;
        current_point.y_ = new_message_->boxes[i].center.y;
        current_point.z_ = new_message_->boxes[i].center.z;
        color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
        current_point.r_ = color.r;
        current_point.g_ = color.g;
        current_point.b_ = color.b;
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
        center.x = new_message_->boxes[i].center.x;
        center.y = new_message_->boxes[i].center.y;
        center.z = new_message_->boxes[i].center.z;
        extents.x = new_message_->boxes[i].extents.x;
        extents.y = new_message_->boxes[i].extents.y;
        extents.z = new_message_->boxes[i].extents.z;

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

void CollisionMapDisplay::incomingMessage(const CollisionMapPtr& message)
{
  message_mutex_.lock();
  new_message_ = message;
  message_mutex_.unlock();
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
          &CollisionMapDisplay::setOverrideColor, this, _1), parent_category_,
      this);
  color_property_ = property_manager_->createProperty<ColorProperty> ("Color",
      property_prefix_, boost::bind(&CollisionMapDisplay::getColor, this),
      boost::bind(&CollisionMapDisplay::setColor, this, _1), parent_category_,
      this);
  render_operation_property_
      = property_manager_->createProperty<EnumProperty> ("Render Operation",
          property_prefix_, boost::bind(
              &CollisionMapDisplay::getRenderOperation, this), boost::bind(
              &CollisionMapDisplay::setRenderOperation, this, _1),
          parent_category_, this);
  render_operation_property_->addOption("Boxes", collision_render_ops::CBoxes);
  render_operation_property_->addOption("Points", collision_render_ops::CPoints);

  z_position_property_
      = property_manager_->createProperty<FloatProperty> ("Z Position",
          property_prefix_, boost::bind(&CollisionMapDisplay::getZPosition,
              this), boost::bind(&CollisionMapDisplay::setZPosition, this, _1),
          parent_category_, this);
  alpha_property_ = property_manager_->createProperty<FloatProperty> ("Alpha",
      property_prefix_, boost::bind(&CollisionMapDisplay::getAlpha, this),
      boost::bind(&CollisionMapDisplay::setAlpha, this, _1), parent_category_,
      this);
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty> (
      "Topic", property_prefix_, boost::bind(&CollisionMapDisplay::getTopic,
          this), boost::bind(&CollisionMapDisplay::setTopic, this, _1),
      parent_category_, this);
  topic_property_->setMessageType(
      robot_msgs::CollisionMap::__s_getDataType());
}

const char*
CollisionMapDisplay::getDescription()
{
  return ("Displays data from a robot_msgs::CollisionMap message as either points or lines.");
}

} // namespace rviz
