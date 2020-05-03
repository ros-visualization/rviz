/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include <rviz/robot/robot_joint.h>
#include <rviz/robot/robot_link.h>
#include <rviz/robot/robot.h>

#include <OgreSceneNode.h>

#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/load_resource.h>

namespace rviz
{
RobotJoint::RobotJoint(Robot* robot, const urdf::JointConstSharedPtr& joint)
  : robot_(robot)
  , name_(joint->name)
  , parent_link_name_(joint->parent_link_name)
  , child_link_name_(joint->child_link_name)
  , has_decendent_links_with_geometry_(true)
  , doing_set_checkbox_(false)
  , axes_(nullptr)
  , axis_(nullptr)
{
  joint_property_ = new Property(name_.c_str(), true, "", nullptr, SLOT(updateChildVisibility()), this);
  joint_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotJoint.png"));

  details_ = new Property("Details", QVariant(), "", nullptr);

  axes_property_ = new Property("Show Axes", false, "Enable/disable showing the axes of this joint.",
                                joint_property_, SLOT(updateAxes()), this);

  position_property_ =
      new VectorProperty("Position", Ogre::Vector3::ZERO,
                         "Position of this joint, in the current Fixed Frame.  (Not editable)",
                         joint_property_);
  position_property_->setReadOnly(true);

  orientation_property_ =
      new QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                             "Orientation of this joint, in the current Fixed Frame.  (Not editable)",
                             joint_property_);
  orientation_property_->setReadOnly(true);

  std::string type = "";
  if (joint->type == urdf::Joint::UNKNOWN)
    type = "unknown";
  else if (joint->type == urdf::Joint::REVOLUTE)
    type = "revolute";
  else if (joint->type == urdf::Joint::CONTINUOUS)
    type = "continuous";
  else if (joint->type == urdf::Joint::PRISMATIC)
    type = "prismatic";
  else if (joint->type == urdf::Joint::FLOATING)
    type = "floating";
  else if (joint->type == urdf::Joint::PLANAR)
    type = "planar";
  else if (joint->type == urdf::Joint::FIXED)
    type = "fixed";

  type_property_ = new StringProperty("Type", QString::fromStdString(type),
                                      "Type of this joint.  (Not editable)", joint_property_);
  type_property_->setReadOnly(true);

  if (joint->limits)
  {
    // continuous joints have lower limit and upper limits of zero,
    // which means this isn't very useful but show it anyhow.
    lower_limit_property_ =
        new FloatProperty("Lower Limit", joint->limits->lower,
                          "Lower limit of this joint.  (Not editable)", joint_property_);
    lower_limit_property_->setReadOnly(true);

    upper_limit_property_ =
        new FloatProperty("Upper Limit", joint->limits->upper,
                          "Upper limit of this joint.  (Not editable)", joint_property_);
    upper_limit_property_->setReadOnly(true);
  }

  if ((type == "continuous") || (type == "revolute") || (type == "prismatic") || (type == "planar"))
  {
    show_axis_property_ =
        new Property("Show Joint Axis", false, "Enable/disable showing the axis of this joint.",
                     joint_property_, SLOT(updateAxis()), this);

    axis_property_ =
        new VectorProperty("Joint Axis", Ogre::Vector3(joint->axis.x, joint->axis.y, joint->axis.z),
                           "Axis of this joint.  (Not editable)", joint_property_);
    axis_property_->setReadOnly(true);
  }

  joint_property_->collapse();

  const urdf::Vector3& pos = joint->parent_to_joint_origin_transform.position;
  const urdf::Rotation& rot = joint->parent_to_joint_origin_transform.rotation;
  joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
  joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

RobotJoint::~RobotJoint()
{
  delete axes_;
  if (axis_)
    delete axis_;
  delete details_;
  delete joint_property_;
}

void RobotJoint::setJointPropertyDescription()
{
  int links_with_geom;
  int links_with_geom_checked;
  int links_with_geom_unchecked;
  getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, true);

  std::stringstream desc;
  desc << "Joint <b>" << name_ << "</b> with parent link <b>" << parent_link_name_
       << "</b> and child link <b>" << child_link_name_ << "</b>.";

  if (links_with_geom == 0)
  {
    desc << "  This joint's descendents have NO geometry.";
    setJointCheckbox(QVariant());
    has_decendent_links_with_geometry_ = false;
  }
  else if (styleIsTree())
  {
    desc << "  Check/uncheck to show/hide all links descended from this joint.";
    setJointCheckbox(links_with_geom_unchecked == 0);
    has_decendent_links_with_geometry_ = true;
  }
  else
  {
    getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, false);
    if (links_with_geom == 0)
    {
      desc << "  This joint's child link has NO geometry.";
      setJointCheckbox(QVariant());
      has_decendent_links_with_geometry_ = false;
    }
    else
    {
      desc << "  Check/uncheck to show/hide this joint's child link.";
      setJointCheckbox(links_with_geom_unchecked == 0);
      has_decendent_links_with_geometry_ = true;
    }
  }

  joint_property_->setDescription(desc.str().c_str());
}

void RobotJoint::setJointCheckbox(const QVariant& val)
{
  // setting doing_set_checkbox_ to true prevents updateChildVisibility() from
  // updating child link enables.
  doing_set_checkbox_ = true;
  joint_property_->setValue(val);
  doing_set_checkbox_ = false;
}

RobotJoint* RobotJoint::getParentJoint()
{
  RobotLink* parent_link = robot_->getLink(parent_link_name_);
  if (!parent_link)
    return nullptr;

  const std::string& parent_joint_name = parent_link->getParentJointName();
  if (parent_joint_name.empty())
    return nullptr;

  return robot_->getJoint(parent_joint_name);
}

void RobotJoint::calculateJointCheckboxesRecursive(int& links_with_geom,
                                                   int& links_with_geom_checked,
                                                   int& links_with_geom_unchecked)
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  RobotLink* link = robot_->getLink(child_link_name_);
  if (link && link->hasGeometry())
  {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }
  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (!styleIsTree())
  {
    if (!links_with_geom)
    {
      setJointCheckbox(QVariant());
    }
    else
    {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }

  std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
  std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
  for (; child_joint_it != child_joint_end; ++child_joint_it)
  {
    RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
    if (child_joint)
    {
      int child_links_with_geom;
      int child_links_with_geom_checked;
      int child_links_with_geom_unchecked;
      child_joint->calculateJointCheckboxesRecursive(
          child_links_with_geom, child_links_with_geom_checked, child_links_with_geom_unchecked);
      links_with_geom_checked += child_links_with_geom_checked;
      links_with_geom_unchecked += child_links_with_geom_unchecked;
    }
  }
  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (styleIsTree())
  {
    if (!links_with_geom)
    {
      setJointCheckbox(QVariant());
    }
    else
    {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }
}


void RobotJoint::getChildLinkState(int& links_with_geom,
                                   int& links_with_geom_checked,
                                   int& links_with_geom_unchecked,
                                   bool recursive) const
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  RobotLink* link = robot_->getLink(child_link_name_);
  if (link && link->hasGeometry())
  {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }

  if (recursive)
  {
    std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
    std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
    for (; child_joint_it != child_joint_end; ++child_joint_it)
    {
      RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
      if (child_joint)
      {
        int child_links_with_geom;
        int child_links_with_geom_checked;
        int child_links_with_geom_unchecked;
        child_joint->getChildLinkState(child_links_with_geom, child_links_with_geom_checked,
                                       child_links_with_geom_unchecked, recursive);
        links_with_geom_checked += child_links_with_geom_checked;
        links_with_geom_unchecked += child_links_with_geom_unchecked;
      }
    }
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;
}


bool RobotJoint::getEnabled() const
{
  if (!hasDescendentLinksWithGeometry())
    return true;
  return joint_property_->getValue().toBool();
}

bool RobotJoint::styleIsTree() const
{
  return details_->getParent() != nullptr;
}

void RobotJoint::updateChildVisibility()
{
  if (doing_set_checkbox_)
    return;

  if (!hasDescendentLinksWithGeometry())
    return;

  bool visible = getEnabled();

  RobotLink* link = robot_->getLink(child_link_name_);
  if (link)
  {
    if (link->hasGeometry())
    {
      link->getLinkProperty()->setValue(visible);
    }

    if (styleIsTree())
    {
      std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
      std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
      for (; child_joint_it != child_joint_end; ++child_joint_it)
      {
        RobotJoint* child_joint = robot_->getJoint(*child_joint_it);
        if (child_joint)
        {
          child_joint->getJointProperty()->setValue(visible);
        }
      }
    }
  }
}

void RobotJoint::updateAxes()
{
  if (axes_property_->getValue().toBool())
  {
    if (!axes_)
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for joint " << name_ << count++;
      axes_ = new Axes(robot_->getSceneManager(), robot_->getOtherNode(), 0.1, 0.01);
      axes_->getSceneNode()->setVisible(getEnabled());

      axes_->setPosition(position_property_->getVector());
      axes_->setOrientation(orientation_property_->getQuaternion());
    }
  }
  else
  {
    if (axes_)
    {
      delete axes_;
      axes_ = nullptr;
    }
  }
}

void RobotJoint::updateAxis()
{
  if (show_axis_property_->getValue().toBool())
  {
    if (!axis_)
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axis for joint " << name_ << count++;
      axis_ = new Arrow(robot_->getSceneManager(), robot_->getOtherNode(), 0.15, 0.05, 0.05, 0.08);
      axis_->getSceneNode()->setVisible(getEnabled());

      axis_->setPosition(position_property_->getVector());
      axis_->setOrientation(orientation_property_->getQuaternion());

      // TODO(lucasw) store an Ogre::ColorValue and set it according to
      // joint type.
      axis_->setColor(0.0, 0.8, 0.0, 1.0);
    }
  }
  else
  {
    if (axis_)
    {
      delete axis_;
      axis_ = nullptr;
    }
  }
}

void RobotJoint::setTransforms(const Ogre::Vector3& parent_link_position,
                               const Ogre::Quaternion& parent_link_orientation)
{
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  if (axes_)
  {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
  }
  if (axis_)
  {
    axis_->setPosition(position);
    axis_->setOrientation(orientation);
    axis_->setDirection(parent_link_orientation * axis_property_->getVector());
  }
}

void RobotJoint::hideSubProperties(bool hide)
{
  position_property_->setHidden(hide);
  orientation_property_->setHidden(hide);
  axes_property_->setHidden(hide);
  show_axis_property_->setHidden(hide);
  axis_property_->setHidden(hide);
}

Ogre::Vector3 RobotJoint::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotJoint::getOrientation()
{
  return orientation_property_->getQuaternion();
}

void RobotJoint::setParentProperty(Property* new_parent)
{
  Property* old_parent = joint_property_->getParent();
  if (old_parent)
    old_parent->takeChild(joint_property_);

  if (new_parent)
    new_parent->addChild(joint_property_);
}

// if use_detail:
//    - all sub properties become children of details_ property.
//    - details_ property becomes a child of joint_property_
// else (!use_detail)
//    - all sub properties become children of joint_property_.
//    details_ property does not have a parent.
void RobotJoint::useDetailProperty(bool use_detail)
{
  Property* old_parent = details_->getParent();
  if (old_parent)
    old_parent->takeChild(details_);

  if (use_detail)
  {
    while (joint_property_->numChildren() > 0)
    {
      Property* child = joint_property_->childAt(0);
      joint_property_->takeChild(child);
      details_->addChild(child);
    }

    joint_property_->addChild(details_);
  }
  else
  {
    while (details_->numChildren() > 0)
    {
      Property* child = details_->childAt(0);
      details_->takeChild(child);
      joint_property_->addChild(child);
    }
  }
}

void RobotJoint::expandDetails(bool expand)
{
  Property* parent = details_->getParent() ? details_ : joint_property_;
  if (expand)
  {
    parent->expand();
  }
  else
  {
    parent->collapse();
  }
}

} // namespace rviz
