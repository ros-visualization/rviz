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

#include "robot.h"
#include "robot_link.h"
#include "common.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "visualization_manager.h"

#include "ogre_tools/object.h"
#include "ogre_tools/shape.h"
#include "ogre_tools/axes.h"

#include <urdf/model.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRibbonTrail.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreResourceGroupManager.h>

#include <ros/console.h>

namespace rviz
{

Robot::Robot( VisualizationManager* manager, const std::string& name )
: scene_manager_(manager->getSceneManager())
, visual_visible_( true )
, collision_visible_( false )
, vis_manager_(manager)
, property_manager_(NULL)
, name_( name )
{
  root_visual_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_collision_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_other_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
  setAlpha(1.0f);
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode( root_visual_node_->getName() );
  scene_manager_->destroySceneNode( root_collision_node_->getName() );
  scene_manager_->destroySceneNode( root_other_node_->getName() );
}

void Robot::setVisible( bool visible )
{
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
    root_collision_node_->setVisible( collision_visible_ );
  }
  else
  {
    root_visual_node_->setVisible( false );
    root_collision_node_->setVisible( false );
  }
}

void Robot::setVisualVisible( bool visible )
{
  visual_visible_ = visible;
  root_visual_node_->setVisible( visible );
}

void Robot::setCollisionVisible( bool visible )
{
  collision_visible_ = visible;
  root_collision_node_->setVisible( visible );
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}

bool Robot::isCollisionVisible()
{
  return collision_visible_;
}

void Robot::setAlpha(float a)
{
  alpha_ = a;

  M_NameToLink::iterator it = links_.begin();
  M_NameToLink::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    RobotLink* info = it->second;

    info->setAlpha(alpha_);
  }
}

void Robot::clear()
{
  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* info = link_it->second;
    delete info;
  }

  if ( property_manager_ )
  {
    property_manager_->deleteByUserData( this );
  }

  links_category_.reset();
  links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
}

void Robot::setPropertyManager( PropertyManager* property_manager, const CategoryPropertyWPtr& parent )
{
  ROS_ASSERT( property_manager );
  ROS_ASSERT( parent.lock() );

  property_manager_ = property_manager;
  parent_property_ = parent;

  if ( !links_.empty() )
  {
    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end; ++link_it )
    {
      RobotLink* info = link_it->second;

      info->createProperties();
    }
  }
}

void Robot::load( TiXmlElement* root_element, urdf::Model &descr, bool visual, bool collision )
{
  clear();

  if ( property_manager_ )
  {
    ROS_ASSERT(!links_category_.lock());
    links_category_ = property_manager_->createCategory( "Links", name_, parent_property_, this );
  }

  typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
  V_Link links;
  descr.getLinks(links);
  V_Link::iterator it = links.begin();
  V_Link::iterator end = links.end();
  for (; it != end; ++it)
  {
    const boost::shared_ptr<urdf::Link>& link = *it;

    RobotLink* link_info = new RobotLink(this, vis_manager_);
    link_info->load(root_element, descr, link, visual, collision);

    bool inserted = links_.insert( std::make_pair( link_info->getName(), link_info ) ).second;
    ROS_ASSERT( inserted );

    link_info->setAlpha(alpha_);
  }

  CategoryPropertyPtr cat_prop = links_category_.lock();
  cat_prop->collapse();

  setVisualVisible(isVisualVisible());
  setCollisionVisible(isCollisionVisible());
}

RobotLink* Robot::getLink( const std::string& name )
{
  M_NameToLink::iterator it = links_.find( name );
  if ( it == links_.end() )
  {
    ROS_WARN( "Link [%s] does not exist", name.c_str() );
    return NULL;
  }

  return it->second;
}

void Robot::update(const LinkUpdater& updater)
{
  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* info = link_it->second;

    info->setToNormalMaterial();

    Ogre::Vector3 visual_position, collision_position;
    Ogre::Quaternion visual_orientation, collision_orientation;
    bool apply_offset_transforms;
    if (updater.getLinkTransforms(info->getName(), visual_position, visual_orientation, collision_position, collision_orientation, apply_offset_transforms))
    {
      info->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation, apply_offset_transforms );
    }
    else
    {
      info->setToErrorMaterial();
    }
  }
}

void Robot::setPosition( const Ogre::Vector3& position )
{
  root_visual_node_->setPosition( position );
  root_collision_node_->setPosition( position );
}

void Robot::setOrientation( const Ogre::Quaternion& orientation )
{
  root_visual_node_->setOrientation( orientation );
  root_collision_node_->setOrientation( orientation );
}

void Robot::setScale( const Ogre::Vector3& scale )
{
  root_visual_node_->setScale( scale );
  root_collision_node_->setScale( scale );
}

const Ogre::Vector3& Robot::getPosition()
{
  return root_visual_node_->getPosition();
}

const Ogre::Quaternion& Robot::getOrientation()
{
  return root_visual_node_->getOrientation();
}

} // namespace rviz
