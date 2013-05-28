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

#include <boost/filesystem.hpp>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRibbonTrail.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/console.h>

#include <resource_retriever/retriever.h>

#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include "rviz/mesh_loader.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/object.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/robot/robot.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "rviz/robot/robot_joint.h"
#include "rviz/robot/robot_link.h"

namespace fs=boost::filesystem;

namespace rviz
{

RobotJoint::RobotJoint( Robot* robot, const boost::shared_ptr<const urdf::Joint>& joint )
  : robot_( robot )
  , name_( joint->name )
  , child_link_name_( joint->child_link_name )
  , parent_link_name_( joint->parent_link_name )
  , axes_( NULL )
{
  joint_property_ = new Property(
                              name_.c_str(),
                              true,
                              "",
                              NULL,
                              SLOT( updateChildVisibility() ),
                              this);

  axes_property_ = new Property(
                              "Show Axes",
                              false,
                              "Enable/disable showing the axes of this joint.",
                              joint_property_,
                              SLOT( updateAxes() ),
                              this );

  position_property_ = new VectorProperty(
                              "Position",
                              Ogre::Vector3::ZERO,
                              "Position of this joint, in the current Fixed Frame.  (Not editable)",
                              joint_property_ );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty(
                              "Orientation",
                              Ogre::Quaternion::IDENTITY,
                              "Orientation of this joint, in the current Fixed Frame.  (Not editable)",
                              joint_property_ );
  orientation_property_->setReadOnly( true );

  joint_property_->collapse();

  std::stringstream desc;
  desc
    << "Joint " << name_
    << " with parent link " << parent_link_name_
    << " and child link " << child_link_name_
    << "."
    << "  Check/uncheck to show/hide all links descended from this joint.";
  joint_property_->setDescription(desc.str().c_str());

  const urdf::Vector3& pos = joint->parent_to_joint_origin_transform.position;
  const urdf::Rotation& rot = joint->parent_to_joint_origin_transform.rotation;
  joint_origin_pos_ = Ogre::Vector3(pos.x, pos.y, pos.z);
  joint_origin_rot_ = Ogre::Quaternion(rot.w, rot.x, rot.y, rot.z);
}

RobotJoint::~RobotJoint()
{
  delete axes_;
}

bool RobotJoint::getEnabled() const
{
  return joint_property_->getValue().toBool();
}

void RobotJoint::updateChildVisibility()
{
  bool visible = getEnabled();

  RobotLink *link = robot_->getLink(child_link_name_);
  if (link)
  {
    link->getLinkProperty()->setValue(visible);
    
    std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
    std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
    for ( ; child_joint_it != child_joint_end ; ++child_joint_it )
    {
      RobotJoint* child_joint = robot_->getJoint( *child_joint_it );
      if (child_joint)
      {
        child_joint->getJointProperty()->setValue(visible);
      }
    }
  }
}

void RobotJoint::updateAxes()
{
  if( axes_property_->getValue().toBool() )
  {
    if( !axes_ )
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for joint " << name_ << count++;
      axes_ = new Axes( robot_->getSceneManager(), robot_->getOtherNode(), 0.1, 0.01 );
      axes_->getSceneNode()->setVisible( getEnabled() );

      axes_->setPosition( position_property_->getVector() );
      axes_->setOrientation( orientation_property_->getQuaternion() );
    }
  }
  else
  {
    if( axes_ )
    {
      delete axes_;
      axes_ = NULL;
    }
  }
}

void RobotJoint::setTransforms( const Ogre::Vector3& parent_link_position,
                                const Ogre::Quaternion& parent_link_orientation )
{
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector( position );
  orientation_property_->setQuaternion( orientation );

  if ( visual_node_ )
  {
    visual_node_->setPosition( position );
    visual_node_->setOrientation( orientation );
  }

  if ( axes_ )
  {
    axes_->setPosition( position );
    axes_->setOrientation( orientation );
  }
}

void RobotJoint::hideSubProperties(bool hide)
{
  position_property_->setHidden(hide);
  orientation_property_->setHidden(hide);
  axes_property_->setHidden(hide);
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

} // namespace rviz

