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

#include "robot_model_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <urdf/model.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

namespace rviz
{

void linkUpdaterStatusFunction(StatusLevel level, const std::string& link_name, const std::string& text, RobotModelDisplay* display)
{
  display->setStatus(level, link_name, text);
}

RobotModelDisplay::RobotModelDisplay()
: Display()
, description_param_("robot_description")
, has_new_transforms_( false )
, time_since_last_transform_( 0.0f )
, update_rate_( 0.0f )
, hidden_(false)
{
}

RobotModelDisplay::~RobotModelDisplay()
{
  delete robot_;
}

void RobotModelDisplay::onInitialize()
{
  robot_ = new Robot( vis_manager_, "Robot: " + name_ );

  setVisualVisible( true );
  setCollisionVisible( false );

  setAlpha(1.0f);
}

void RobotModelDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  robot_->setAlpha(alpha_);

  propertyChanged(alpha_property_);
}

void RobotModelDisplay::setRobotDescription( const std::string& description_param )
{
  description_param_ = description_param;

  propertyChanged(robot_description_property_);

  if ( isEnabled() )
  {
    load();
    causeRender();
  }
}

void RobotModelDisplay::hideVisible()
{
  hidden_ = true;
  robot_->setVisible( enabled_ && !hidden_ );
}

void RobotModelDisplay::restoreVisible()
{
  hidden_ = false;
  robot_->setVisible( enabled_ && !hidden_ );
}

void RobotModelDisplay::setVisualVisible( bool visible )
{
  robot_->setVisualVisible( visible && !hidden_ );

  propertyChanged(visual_enabled_property_);

  causeRender();
}

void RobotModelDisplay::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible && !hidden_ );

  propertyChanged(collision_enabled_property_);

  causeRender();
}

void RobotModelDisplay::setUpdateRate( float rate )
{
  update_rate_ = rate;

  propertyChanged(update_rate_property_);

  causeRender();
}

void RobotModelDisplay::setTFPrefix(const std::string& prefix)
{
  clearStatuses();

  tf_prefix_ = prefix;

  propertyChanged(tf_prefix_property_);

  causeRender();
}

bool RobotModelDisplay::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool RobotModelDisplay::isCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void RobotModelDisplay::load()
{
  std::string content;
  if (!update_nh_.getParam(description_param_, content))
  {
    std::string loc;
    if (update_nh_.searchParam(description_param_, loc))
    {
      update_nh_.getParam(loc, content);
    }
    else
    {
      clear();

      std::stringstream ss;
      ss << "Parameter [" << description_param_ << "] does not exist, and was not found by searchParam()";
      setStatus(status_levels::Error, "URDF", ss.str());
      return;
    }
  }

  if (content.empty())
  {
    clear();
    setStatus(status_levels::Error, "URDF", "URDF is empty");
    return;
  }

  if ( content == robot_description_ )
  {
    return;
  }

  robot_description_ = content;

  TiXmlDocument doc;
  doc.Parse(robot_description_.c_str());
  if (!doc.RootElement())
  {
    clear();
    setStatus(status_levels::Error, "URDF", "URDF failed XML parse");
    return;
  }

  urdf::Model descr;
  if (!descr.initXml(doc.RootElement()))
  {
    clear();
    setStatus(status_levels::Error, "URDF", "URDF failed Model parse");
    return;
  }

  setStatus(status_levels::Ok, "URDF", "URDF parsed OK");
  robot_->load( doc.RootElement(), descr );
  robot_->update( TFLinkUpdater(vis_manager_->getFrameManager(), boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this), tf_prefix_) );
}

void RobotModelDisplay::onEnable()
{
  load();
  robot_->setVisible( enabled_ && !hidden_ );
}

void RobotModelDisplay::onDisable()
{
  robot_->setVisible( enabled_ && !hidden_ );
  clear();
}

void RobotModelDisplay::update(float wall_dt, float ros_dt)
{
  time_since_last_transform_ += wall_dt;

  bool update = update_rate_ < 0.0001f || time_since_last_transform_ >= update_rate_;

  if ( has_new_transforms_ || update )
  {
    robot_->update(TFLinkUpdater(vis_manager_->getFrameManager(), boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this), tf_prefix_));
    causeRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void RobotModelDisplay::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelDisplay::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Visual Enabled", property_prefix_, boost::bind( &RobotModelDisplay::isVisualVisible, this ),
                                                                               boost::bind( &RobotModelDisplay::setVisualVisible, this, _1 ), parent_category_, this );
  setPropertyHelpText(visual_enabled_property_, "Whether to display the visual representation of the robot.");
  collision_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Collision Enabled", property_prefix_, boost::bind( &RobotModelDisplay::isCollisionVisible, this ),
                                                                                 boost::bind( &RobotModelDisplay::setCollisionVisible, this, _1 ), parent_category_, this );
  setPropertyHelpText(collision_enabled_property_, "Whether to display the collision representation of the robot.");
  update_rate_property_ = property_manager_->createProperty<FloatProperty>( "Update Interval", property_prefix_, boost::bind( &RobotModelDisplay::getUpdateRate, this ),
                                                                                  boost::bind( &RobotModelDisplay::setUpdateRate, this, _1 ), parent_category_, this );
  setPropertyHelpText(update_rate_property_, "Interval at which to update the links, in seconds.  0 means to update every update cycle.");
  FloatPropertyPtr float_prop = update_rate_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->addLegacyName("Update Rate");

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &RobotModelDisplay::getAlpha, this ),
                                                                      boost::bind( &RobotModelDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the links.");
  float_prop = alpha_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->setMax( 1.0 );

  robot_description_property_ = property_manager_->createProperty<StringProperty>( "Robot Description", property_prefix_, boost::bind( &RobotModelDisplay::getRobotDescription, this ),
                                                                                   boost::bind( &RobotModelDisplay::setRobotDescription, this, _1 ), parent_category_, this );
  setPropertyHelpText(robot_description_property_, "Name of the parameter to search for to load the robot description.");

  tf_prefix_property_ = property_manager_->createProperty<StringProperty>( "TF Prefix", property_prefix_, boost::bind( &RobotModelDisplay::getTFPrefix, this ),
                                                                           boost::bind( &RobotModelDisplay::setTFPrefix, this, _1 ), parent_category_, this );
  setPropertyHelpText(tf_prefix_property_, "Robot Model normally assumes the link name is the same as the tf frame name.  This option allows you to set a prefix.  Mainly useful for multi-robot situations.");

  robot_->setPropertyManager( property_manager_, parent_category_ );
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

