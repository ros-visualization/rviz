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
#include "properties/property.h"
#include "display_context.h"

#include "ogre_helpers/object.h"
#include "ogre_helpers/shape.h"
#include "ogre_helpers/axes.h"

#include <urdf_model/model.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreResourceGroupManager.h>

#include <ros/console.h>
#include <ros/assert.h>

namespace rviz
{

Robot::Robot( Ogre::SceneNode* root_node, DisplayContext* context, const std::string& name, Property* parent_property )
  : scene_manager_( context->getSceneManager() )
  , visual_visible_( true )
  , collision_visible_( false )
  , context_( context )
  , name_( name )
{
  root_visual_node_ = root_node->createChildSceneNode();
  root_collision_node_ = root_node->createChildSceneNode();
  root_other_node_ = root_node->createChildSceneNode();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
  setAlpha(1.0f);

  links_category_ = new Property( "Links", QVariant(), "", parent_property );
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
  updateLinkVisibilities();
}

void Robot::setCollisionVisible( bool visible )
{
  collision_visible_ = visible;
  updateLinkVisibilities();
}

void Robot::updateLinkVisibilities()
{
  M_NameToLink::iterator it = links_.begin();
  M_NameToLink::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    RobotLink* link = it->second;
    link->updateVisibility();
  }
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

    info->setRobotAlpha(alpha_);
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

  links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
  root_other_node_->removeAndDestroyAllChildren();
}

class LinkComparator
{
public:
  bool operator()(const boost::shared_ptr<urdf::Link>& lhs, const boost::shared_ptr<urdf::Link>& rhs)
  {
    return lhs->name < rhs->name;
  }
};

void Robot::load( const urdf::ModelInterface &descr, bool visual, bool collision )
{
  clear();

  typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
  V_Link links;
  descr.getLinks( links );
  std::sort( links.begin(), links.end(), LinkComparator() );
  V_Link::iterator it = links.begin();
  V_Link::iterator end = links.end();
  for( ; it != end; ++it )
  {
    const boost::shared_ptr<urdf::Link>& link = *it;

    RobotLink* link_info = new RobotLink( this, context_, links_category_ );

    link_info->load( descr, link, visual, collision );

    if( !link_info->isValid() )
    {
      delete link_info;
      continue;
    }

    bool inserted = links_.insert( std::make_pair( link_info->getName(), link_info ) ).second;
    ROS_ASSERT( inserted );

    link_info->setRobotAlpha( alpha_ );
  }

  links_category_->collapse();

  setVisualVisible( isVisualVisible() );
  setCollisionVisible( isCollisionVisible() );
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
    if( updater.getLinkTransforms( info->getName(),
                                   visual_position, visual_orientation,
                                   collision_position, collision_orientation
                                   ))
    {
      info->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation );
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
