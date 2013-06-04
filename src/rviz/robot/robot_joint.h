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

#ifndef RVIZ_ROBOT_JOINT_H
#define RVIZ_ROBOT_JOINT_H

#include <string>
#include <map>

#include <QObject>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreAny.h>
#include <OGRE/OgreMaterial.h>

#include "rviz/ogre_helpers/object.h"
#include "rviz/selection/forwards.h"

namespace Ogre
{
class SceneManager;
class Entity;
class SubEntity;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
class RibbonTrail;
}

namespace urdf
{
class ModelInterface;
class Link;
class Joint;
class Geometry;
class Pose;
}

namespace rviz
{
class Shape;
class Axes;
class DisplayContext;
class FloatProperty;
class Property;
class BoolProperty;
class QuaternionProperty;
class Robot;
class RobotLinkSelectionHandler;
class VectorProperty;
class RobotJoint;


/**
 * \struct RobotJoint
 * \brief Contains any data we need from a joint in the robot.
 */
class RobotJoint: public QObject
{
Q_OBJECT
public:
  RobotJoint( Robot* robot, const boost::shared_ptr<const urdf::Joint>& joint );
  virtual ~RobotJoint();


  void setTransforms(const Ogre::Vector3& parent_link_position,
                     const Ogre::Quaternion& parent_link_orientation);

  const std::string& getName() { return name_; }
  const std::string& getParentLinkName() { return parent_link_name_; }
  const std::string& getChildLinkName() { return child_link_name_; }
  Property* getJointProperty() { return joint_property_; }
  void hideSubProperties(bool hide);

  // Remove joint_property_ from its old parent and add to new_parent.  If new_parent==NULL then leav unparented.
  void setParentProperty(Property* new_parent);

  Ogre::Vector3 getPosition();
  Ogre::Quaternion getOrientation();

  void setRobotAlpha(float a) {}

  bool hasDescendentLinksWithGeometry() const { return has_decendent_links_with_geometry_; }
  bool checkForDescendentLinksWithGeometry();

  // place subproperties as children of details_ or joint_property_
  void useDetailProperty(bool use_detail);

  // expand all sub properties
  void expandDetails(bool expand);

private Q_SLOTS:
  void updateAxes();
  void updateChildVisibility();

private:
  bool getEnabled() const;
  void setJointPropertyDescription(bool has_descendent_geometry);

protected:
  Robot* robot_;
  std::string name_;                          ///< Name of this joint
  std::string parent_link_name_;
  std::string child_link_name_;

  // properties
  Property* joint_property_;
  Property* details_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  Property* axes_property_;

private:
  Ogre::Vector3 joint_origin_pos_;
  Ogre::Quaternion joint_origin_rot_;
  bool has_decendent_links_with_geometry_;

  Axes* axes_;
};

} // namespace rviz

#endif // RVIZ_ROBOT_LINK_H
