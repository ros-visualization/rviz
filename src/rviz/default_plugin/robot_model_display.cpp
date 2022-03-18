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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <QTimer>

#include <urdf/model.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/robot_link.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"

#include "robot_model_display.h"

namespace rviz
{
void linkUpdaterStatusFunction(StatusProperty::Level level,
                               const std::string& link_name,
                               const std::string& text,
                               RobotModelDisplay* display)
{
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

RobotModelDisplay::RobotModelDisplay()
  : Display(), has_new_transforms_(false), time_since_last_transform_(0.0f)
{
  visual_enabled_property_ =
      new Property("Visual Enabled", true, "Whether to display the visual representation of the robot.",
                   this, SLOT(updateVisualVisible()));

  collision_enabled_property_ =
      new Property("Collision Enabled", false,
                   "Whether to display the collision representation of the robot.", this,
                   SLOT(updateCollisionVisible()));

  update_rate_property_ = new FloatProperty("Update Interval", 0,
                                            "Interval at which to update the links, in seconds. "
                                            "0 means to update every update cycle.",
                                            this);
  update_rate_property_->setMin(0);

  alpha_property_ = new FloatProperty("Alpha", 1, "Amount of transparency to apply to the links.", this,
                                      SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  robot_description_property_ =
      new StringProperty("Robot Description", "robot_description",
                         "Name of the parameter to search for to load the robot description.", this,
                         SLOT(updateRobotDescription()));

  tf_prefix_property_ = new StringProperty(
      "TF Prefix", "",
      "Robot Model normally assumes the link name is the same as the tf frame name. "
      " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
      this, SLOT(updateTfPrefix()));
}

RobotModelDisplay::~RobotModelDisplay()
{
  if (initialized())
  {
    delete robot_;
  }
}

void RobotModelDisplay::onInitialize()
{
  robot_ = new Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this);

  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
}

void RobotModelDisplay::updateAlpha()
{
  robot_->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void RobotModelDisplay::updateRobotDescription()
{
  if (isEnabled())
    load();
}

void RobotModelDisplay::updateVisualVisible()
{
  robot_->setVisualVisible(visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void RobotModelDisplay::updateCollisionVisible()
{
  robot_->setCollisionVisible(collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void RobotModelDisplay::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void RobotModelDisplay::load()
{
  clearStatuses();
  context_->queueRender();

  std::string content;
  try
  {
    if (!update_nh_.getParam(robot_description_property_->getStdString(), content))
    {
      std::string loc;
      if (update_nh_.searchParam(robot_description_property_->getStdString(), loc))
        update_nh_.getParam(loc, content);
      else
      {
        clear();
        setStatus(StatusProperty::Error, "URDF",
                  QString("Parameter [%1] does not exist, and was not found by searchParam()")
                      .arg(robot_description_property_->getString()));
        // try again in a second
        QTimer::singleShot(1000, this, SLOT(updateRobotDescription()));
        return;
      }
    }
  }
  catch (const ros::InvalidNameException& e)
  {
    clear();
    setStatus(StatusProperty::Error, "URDF",
              QString("Invalid parameter name: %1.\n%2")
                  .arg(robot_description_property_->getString(), e.what()));
    return;
  }

  if (content.empty())
  {
    clear();
    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }

  if (content == robot_description_)
  {
    return;
  }

  robot_description_ = content;

  urdf::Model descr;
  if (!descr.initString(robot_description_))
  {
    clear();
    setStatus(StatusProperty::Error, "URDF", "Failed to parse URDF model");
    return;
  }

  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
  robot_->load(descr);
  std::stringstream ss;
  for (const auto& name_link_pair : robot_->getLinks())
  {
    const std::string& err = name_link_pair.second->getGeometryErrors();
    if (!err.empty())
      ss << "\n• for link '" << name_link_pair.first << "':\n" << err;
  }
  if (ss.tellp())
    setStatus(StatusProperty::Error, "URDF",
              QString("Errors loading geometries:").append(ss.str().c_str()));

  robot_->update(TFLinkUpdater(context_->getFrameManager(),
                               boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                               tf_prefix_property_->getStdString()));
}

void RobotModelDisplay::onEnable()
{
  load();
  robot_->setVisible(true);
}

void RobotModelDisplay::onDisable()
{
  robot_->setVisible(false);
  clear();
}

void RobotModelDisplay::update(float wall_dt, float /*ros_dt*/)
{
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if (has_new_transforms_ || update)
  {
    robot_->update(TFLinkUpdater(context_->getFrameManager(),
                                 boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                                 tf_prefix_property_->getStdString()));
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void RobotModelDisplay::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelDisplay::clear()
{
  robot_->clear();
  clearStatuses();
  robot_description_.clear();
}

void RobotModelDisplay::reset()
{
  Display::reset();
  has_new_transforms_ = true;
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::RobotModelDisplay, rviz::Display)
