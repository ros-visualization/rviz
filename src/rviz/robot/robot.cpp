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
#include "robot_joint.h"
#include "properties/property.h"
#include "properties/enum_property.h"
#include "properties/bool_property.h"
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
  , visible_( true )
  , visual_visible_( true )
  , collision_visible_( false )
  , context_( context )
  , name_( name )
{
  root_visual_node_ = root_node->createChildSceneNode();
  root_collision_node_ = root_node->createChildSceneNode();
  root_other_node_ = root_node->createChildSceneNode();

  link_factory_ = new LinkFactory();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
  setAlpha(1.0f);

  link_tree_ = new Property( "Links", QVariant(), "", parent_property );
  link_tree_style_ = new EnumProperty(
                            "Link Tree Style",
                            "",
                            "How the list of links is displayed",
                            link_tree_,
                            SLOT( changedLinkTreeStyle() ),
                            this );
  initLinkTreeStyle();
  link_tree_expand_joints_ = new BoolProperty(
                            "Expand all joints",
                            false,
                            "Expand or collapse all joint properties",
                            link_tree_,
                            SLOT( changedExpandJoints() ),
                            this );
  link_tree_expand_links_ = new BoolProperty(
                            "Expand all links",
                            false,
                            "Expand or collapse all link properties",
                            link_tree_,
                            SLOT( changedExpandLinks() ),
                            this );
  hide_details_ = new BoolProperty(
                            "Hide details",
                            false,
                            "Hide details (sub properties) to see the simple tree structure.",
                            link_tree_,
                            SLOT( changedHideSubProperties() ),
                            this );
  hide_details_->hide();
  show_all_links_ = new BoolProperty(
                            "Enable all links",
                            true,
                            "Turn all links on or off.",
                            link_tree_,
                            SLOT( changedShowAllLinks() ),
                            this );
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode( root_visual_node_->getName() );
  scene_manager_->destroySceneNode( root_collision_node_->getName() );
  scene_manager_->destroySceneNode( root_other_node_->getName() );
  delete link_factory_;
}

void Robot::setLinkFactory(LinkFactory *link_factory)
{
  if (link_factory)
  {
    delete link_factory_;
    link_factory_ = link_factory;
  }
}

void Robot::setVisible( bool visible )
{
  visible_ = visible;
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
    root_collision_node_->setVisible( collision_visible_ );
    updateLinkVisibilities();
  }
  else
  {
    root_visual_node_->setVisible( false );
    root_collision_node_->setVisible( false );
    updateLinkVisibilities();
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

bool Robot::isVisible()
{
  return visible_;
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
    RobotLink* link = it->second;

    link->setRobotAlpha(alpha_);
  }
}

void Robot::clear()
{
  unparentLinkProperties();

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    RobotLink* link = link_it->second;
    delete link;
  }

  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end; ++joint_it )
  {
    RobotJoint* joint = joint_it->second;
    delete joint;
  }

  links_.clear();
  joints_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
  root_other_node_->removeAndDestroyAllChildren();
}

RobotLink* Robot::LinkFactory::createLink(
    Robot* robot,
    const boost::shared_ptr<const urdf::Link>& link,
    const std::string& parent_joint_name,
    bool visual,
    bool collision)
{
  return new RobotLink(robot, link, parent_joint_name, visual, collision);
}

RobotJoint* Robot::LinkFactory::createJoint(
    Robot* robot,
    const boost::shared_ptr<const urdf::Joint>& joint)
{
  return new RobotJoint(robot, joint);
}

void Robot::load( const urdf::ModelInterface &urdf, bool visual, bool collision )
{
  // clear out any data (properties, shapes, etc) from a previously loaded robot.
  clear();

  // the root link is discovered below.  Set to NULL until found.
  root_link_ = NULL;

  // Create properties for each link.
  // Properties are not added to display until changedLinkTreeStyle() is called (below).
  typedef std::map<std::string, boost::shared_ptr<urdf::Link> > M_NameToUrdfLink;
  M_NameToUrdfLink::const_iterator link_it = urdf.links_.begin();
  M_NameToUrdfLink::const_iterator link_end = urdf.links_.end();
  for( ; link_it != link_end; ++link_it )
  {
    const boost::shared_ptr<const urdf::Link>& urdf_link = link_it->second;
    std::string parent_joint_name;

    if (urdf_link != urdf.getRoot() && urdf_link->parent_joint)
    {
      parent_joint_name = urdf_link->parent_joint->name;
    }
    
    RobotLink* link = link_factory_->createLink( this,
                                                 urdf_link,
                                                 parent_joint_name,
                                                 visual,
                                                 collision );

    if (urdf_link == urdf.getRoot())
    {
      root_link_ = link;
    }

    links_[urdf_link->name] = link;

    link->setRobotAlpha( alpha_ );
  }

  // Create properties for each joint.
  // Properties are not added to display until changedLinkTreeStyle() is called (below).
  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > M_NameToUrdfJoint;
  M_NameToUrdfJoint::const_iterator joint_it = urdf.joints_.begin();
  M_NameToUrdfJoint::const_iterator joint_end = urdf.joints_.end();
  for( ; joint_it != joint_end; ++joint_it )
  {
    const boost::shared_ptr<const urdf::Joint>& urdf_joint = joint_it->second;
    RobotJoint* joint = link_factory_->createJoint( this, urdf_joint );

    joints_[urdf_joint->name] = joint;

    joint->setRobotAlpha( alpha_ );
  }


  // set the link tree style and add link/joint properties to rviz pane.
  setLinkTreeStyle(LinkTreeStyle(link_tree_style_->getOptionInt()));
  changedLinkTreeStyle();

  // at startup the link tree is collapsed since it is large and not often needed.
  link_tree_->collapse();

  setVisualVisible( isVisualVisible() );
  setCollisionVisible( isCollisionVisible() );
}

void Robot::unparentLinkProperties()
{
  // remove link properties from their parents
  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end ; ++link_it )
  {
    link_it->second->setParentProperty(NULL);
  }

  // remove joint properties from their parents
  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end ; ++joint_it )
  {
    joint_it->second->setParentProperty(NULL);
  }
}

void Robot::changedExpandJoints()
{
  bool expand = link_tree_expand_joints_->getBool();
  
  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end ; ++joint_it )
  {
    if (expand)
      joint_it->second->getJointProperty()->expand();
    else
      joint_it->second->getJointProperty()->collapse();
  }
}

void Robot::changedExpandLinks()
{
  bool expand = link_tree_expand_links_->getBool();

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end ; ++link_it )
  {
    if (expand)
      link_it->second->getLinkProperty()->expand();
    else
      link_it->second->getLinkProperty()->collapse();
  }
}

void Robot::changedHideSubProperties()
{
  bool hide = hide_details_->getBool();

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end ; ++link_it )
  {
    link_it->second->hideSubProperties(hide);
  }

  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end ; ++joint_it )
  {
    joint_it->second->hideSubProperties(hide);
  }
}

void Robot::changedShowAllLinks()
{
  bool show = show_all_links_->getBool();

  M_NameToLink::iterator link_it = links_.begin();
  M_NameToLink::iterator link_end = links_.end();
  for ( ; link_it != link_end ; ++link_it )
  {
    link_it->second->getLinkProperty()->setValue(show);
  }

  M_NameToJoint::iterator joint_it = joints_.begin();
  M_NameToJoint::iterator joint_end = joints_.end();
  for ( ; joint_it != joint_end ; ++joint_it )
  {
    joint_it->second->getJointProperty()->setValue(show);
  }
}

void Robot::initLinkTreeStyle()
{
  style_name_map_.clear();
  style_name_map_[STYLE_VISIBLE_LINK_LIST] = "List of visible links";
	style_name_map_[STYLE_LINK_LIST] = "List of Links";
	style_name_map_[STYLE_JOINT_LIST] = "List of Joints";
	style_name_map_[STYLE_JOINT_LINK_LIST] = "List of Links and Joints in hierarchy order";
	style_name_map_[STYLE_LINK_TREE] = "Tree of links";
	style_name_map_[STYLE_JOINT_TREE] = "Tree of joints";
	style_name_map_[STYLE_JOINT_LINK_TREE] = "Tree of joints with links";

  link_tree_style_->clearOptions();
  std::map<LinkTreeStyle, std::string>::const_iterator style_it = style_name_map_.begin();
  std::map<LinkTreeStyle, std::string>::const_iterator style_end = style_name_map_.end();
  for ( ; style_it != style_end ; ++style_it )
  {
    link_tree_style_->addOptionStd( style_it->second, style_it->first );
  }
}

bool Robot::styleShowLink(LinkTreeStyle style)
{
  return 
    style == STYLE_VISIBLE_LINK_LIST ||
    style == STYLE_LINK_LIST ||
    style == STYLE_JOINT_LINK_LIST ||
    style == STYLE_LINK_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}

bool Robot::styleShowJoint(LinkTreeStyle style)
{
  return 
    style == STYLE_JOINT_LIST ||
    style == STYLE_JOINT_LINK_LIST ||
    style == STYLE_JOINT_TREE ||
    style == STYLE_JOINT_LINK_TREE;
}

void Robot::setLinkTreeStyle(LinkTreeStyle style)
{
  std::map<LinkTreeStyle, std::string>::const_iterator style_it = style_name_map_.find(style);
  if (style_it == style_name_map_.end())
    link_tree_style_->setValue(style_name_map_[STYLE_DEFAULT].c_str());
  else
    link_tree_style_->setValue(style_it->second.c_str());
}

// insert properties into link_tree_ according to style
void Robot::changedLinkTreeStyle()
{
  LinkTreeStyle style = LinkTreeStyle(link_tree_style_->getOptionInt());

  unparentLinkProperties();

  hide_details_->setValue(false);
  link_tree_expand_joints_->setValue(false);
  link_tree_expand_links_->setValue(false);

  switch (style)
  {
  case STYLE_LINK_TREE:
  case STYLE_JOINT_TREE:
  case STYLE_JOINT_LINK_TREE:
  case STYLE_JOINT_LINK_LIST:
    if (root_link_)
    {
      addLinkToLinkTree(style, link_tree_, root_link_);
    }
    break;

  case STYLE_JOINT_LIST:
  {
    M_NameToJoint::iterator joint_it = joints_.begin();
    M_NameToJoint::iterator joint_end = joints_.end();
    for ( ; joint_it != joint_end ; ++joint_it )
    {
      joint_it->second->setParentProperty(link_tree_);
    }
    break;
  }

  case STYLE_LINK_LIST:
  case STYLE_VISIBLE_LINK_LIST:
  default:
    M_NameToLink::iterator link_it = links_.begin();
    M_NameToLink::iterator link_end = links_.end();
    for ( ; link_it != link_end ; ++link_it )
    {
      if (style != STYLE_LINK_LIST && !link_it->second->isValid())
        continue;
      link_it->second->setParentProperty(link_tree_);
    }
    break;
  }

  switch (style)
  {
  case STYLE_LINK_TREE:
    link_tree_->setName("Link Tree");
    link_tree_->setDescription("A tree of all links in the robot.  Uncheck a link to hide its geometry.");
    link_tree_expand_joints_->hide();
    link_tree_expand_links_->show();
    hide_details_->show();
    break;
  case STYLE_JOINT_TREE:
    link_tree_->setName("Joint Tree");
    link_tree_->setDescription("A tree of all joints in the robot.");
    link_tree_expand_joints_->show();
    link_tree_expand_links_->hide();
    hide_details_->show();
    break;
  case STYLE_JOINT_LINK_TREE:
    link_tree_->setName("Joint Tree");
    link_tree_->setDescription("A tree of all joints and links in the robot.  Uncheck a link to hide its geometry.");
    link_tree_expand_joints_->show();
    link_tree_expand_links_->show();
    hide_details_->show();
    break;
  case STYLE_JOINT_LINK_LIST:
    link_tree_->setName("Links");
    link_tree_->setDescription("All joints and links in the robot in hierarchical order.  Uncheck a link to hide its geometry.");
    link_tree_expand_joints_->show();
    link_tree_expand_links_->show();
    hide_details_->hide();
    break;
  case STYLE_JOINT_LIST:
    link_tree_->setName("Joints");
    link_tree_->setDescription("All joints in the robot.");
    link_tree_expand_joints_->show();
    link_tree_expand_links_->hide();
    hide_details_->hide();
    break;
  case STYLE_LINK_LIST:
    link_tree_->setName("Links");
    link_tree_->setDescription("All links in the robot.  Uncheck a link to hide its geometry.");
    link_tree_expand_joints_->hide();
    link_tree_expand_links_->show();
    hide_details_->hide();
    break;
  case STYLE_VISIBLE_LINK_LIST:
  default:
    link_tree_->setName("Links");
    link_tree_->setDescription("All links with visible or collision geometry in the robot.  Uncheck a link to hide its geometry.");
    link_tree_expand_joints_->hide();
    link_tree_expand_links_->show();
    hide_details_->hide();
    break;
  }
}


// recursive helper for setLinkTreeStyle() when style is *_TREE or STYLE_JOINT_LINK_LIST
void Robot::addLinkToLinkTree(LinkTreeStyle style, Property *parent, RobotLink *link)
{
  if (styleShowLink(style))
  {
    link->setParentProperty(parent);
    if (style == STYLE_LINK_TREE)
      parent = link->getLinkProperty();
  }

  std::vector<std::string>::const_iterator child_joint_it = link->getChildJointNames().begin();
  std::vector<std::string>::const_iterator child_joint_end = link->getChildJointNames().end();
  for ( ; child_joint_it != child_joint_end ; ++child_joint_it )
  {
    RobotJoint* child_joint = getJoint( *child_joint_it );
    if (child_joint)
    {
      addJointToLinkTree(style, parent, child_joint);
    }
  }
}

// recursive helper for setLinkTreeStyle() when style is *_TREE or STYLE_JOINT_LINK_LIST
void Robot::addJointToLinkTree(LinkTreeStyle style, Property *parent, RobotJoint *joint)
{
  if (styleShowJoint(style))
  {
    joint->setParentProperty(parent);
    if (style != STYLE_JOINT_LINK_LIST)
      parent = joint->getJointProperty();
  }

  RobotLink *link = getLink( joint->getChildLinkName() );
  if (link)
  {
    addLinkToLinkTree(style, parent, link);
  }
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

RobotJoint* Robot::getJoint( const std::string& name )
{
  M_NameToJoint::iterator it = joints_.find( name );
  if ( it == joints_.end() )
  {
    ROS_WARN( "Joint [%s] does not exist", name.c_str() );
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
    RobotLink* link = link_it->second;

    link->setToNormalMaterial();

    Ogre::Vector3 visual_position, collision_position;
    Ogre::Quaternion visual_orientation, collision_orientation;
    if( updater.getLinkTransforms( link->getName(),
                                   visual_position, visual_orientation,
                                   collision_position, collision_orientation
                                   ))
    {
      link->setTransforms( visual_position, visual_orientation, collision_position, collision_orientation );

      std::vector<std::string>::const_iterator joint_it = link->getChildJointNames().begin();
      std::vector<std::string>::const_iterator joint_end = link->getChildJointNames().end();
      for ( ; joint_it != joint_end ; ++joint_it )
      {
        RobotJoint *joint = getJoint(*joint_it);
        if (joint)
        {
          joint->setTransforms(visual_position, visual_orientation);
        }
      }
    }
    else
    {
      link->setToErrorMaterial();
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
