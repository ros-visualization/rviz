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

#include "robot_link.h"
#include "robot.h"
#include "common.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "visualization_manager.h"
#include "selection/selection_manager.h"

#include "ogre_tools/object.h"
#include "ogre_tools/shape.h"
#include "ogre_tools/axes.h"

#include <tf/transform_listener.h>
#include <planning_models/kinematic.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRibbonTrail.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/console.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace rviz
{

class RobotLinkSelectionHandler : public SelectionHandler
{
public:
  RobotLinkSelectionHandler(RobotLink* link);
  virtual ~RobotLinkSelectionHandler();

  virtual void createProperties(const Picked& obj, PropertyManager* property_manager);

private:
  RobotLink* link_;
};

RobotLinkSelectionHandler::RobotLinkSelectionHandler(RobotLink* link)
: link_(link)
{
}

RobotLinkSelectionHandler::~RobotLinkSelectionHandler()
{
}

void RobotLinkSelectionHandler::createProperties(const Picked& obj, PropertyManager* property_manager)
{
  std::stringstream ss;
  ss << link_->getName() << " Link " << link_->getName();

  CategoryPropertyWPtr cat = property_manager->createCategory( "Link " + link_->getName(), ss.str(), CategoryPropertyWPtr(), (void*)obj.handle );
  properties_.push_back(cat);

  properties_.push_back(property_manager->createProperty<Vector3Property>( "Position", ss.str(), boost::bind( &RobotLink::getPositionInRobotFrame, link_ ),
                                                                                Vector3Property::Setter(), cat, (void*)obj.handle ));

  properties_.push_back(property_manager->createProperty<QuaternionProperty>( "Orientation", ss.str(), boost::bind( &RobotLink::getOrientationInRobotFrame, link_ ),
                                                                                      QuaternionProperty::Setter(), cat, (void*)obj.handle ));

  properties_.push_back(property_manager->createProperty<DoubleProperty>( "Joint Position", ss.str(), boost::bind( &RobotLink::getJointPosition, link_ ),
                                                                                      DoubleProperty::Setter(), cat, (void*)obj.handle ));
  properties_.push_back(property_manager->createProperty<DoubleProperty>( "Joint Velocity", ss.str(), boost::bind( &RobotLink::getJointVelocity, link_ ),
                                                                                      DoubleProperty::Setter(), cat, (void*)obj.handle ));
  properties_.push_back(property_manager->createProperty<DoubleProperty>( "Joint Applied Effort", ss.str(), boost::bind( &RobotLink::getJointAppliedEffort, link_ ),
                                                                                            DoubleProperty::Setter(), cat, (void*)obj.handle ));
  properties_.push_back(property_manager->createProperty<DoubleProperty>( "Joint Commanded Effort", ss.str(), boost::bind( &RobotLink::getJointCommandedEffort, link_ ),
                                                                                                DoubleProperty::Setter(), cat, (void*)obj.handle ));
}

RobotLink::RobotLink(Robot* parent, VisualizationManager* manager)
: parent_(parent)
, scene_manager_(manager->getSceneManager())
, property_manager_(manager->getPropertyManager())
, vis_manager_(manager)
, visual_mesh_( NULL )
, collision_mesh_( NULL )
, collision_object_( NULL )
, visual_node_( NULL )
, collision_node_( NULL )
, collision_offset_position_(Ogre::Vector3::ZERO)
, collision_offset_orientation_(Ogre::Quaternion::IDENTITY)
, position_(Ogre::Vector3::ZERO)
, orientation_(Ogre::Quaternion::IDENTITY)
, trail_( NULL )
, axes_( NULL )
, selection_object_(NULL)
{
}

RobotLink::~RobotLink()
{
  if ( visual_mesh_ )
  {
    scene_manager_->destroyEntity( visual_mesh_ );
  }

  if ( trail_ )
  {
    scene_manager_->destroyRibbonTrail( trail_ );
  }

  delete axes_;
  delete collision_object_;

  if (selection_object_)
  {
    vis_manager_->getSelectionManager()->removeObject(selection_object_);
  }
}

void RobotLink::load(mechanism::Robot& descr, const mechanism::Link& link, bool visual, bool collision)
{
  name_ = link.name_;
  joint_name_ = link.joint_name_;

  if ( visual )
  {
    createVisual( link );
    createSelection( descr, link );
  }

  if ( collision )
  {
    createCollision( link );
  }

  if ( property_manager_ )
  {
    createProperties();
  }
}

void RobotLink::setAlpha(float a)
{
  if (collision_object_)
  {
    collision_object_->setColor( 0.0f, 0.6f, 1.0f, a );
  }

  if (visual_mesh_)
  {
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(material_name_);

    Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
    color.a = a;
    material->setDiffuse( color );

    if ( a < 0.9998 )
    {
      material->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
      material->setDepthWriteEnabled( false );
    }
    else
    {
      material->setSceneBlending( Ogre::SBT_REPLACE );
      material->setDepthWriteEnabled( true );
    }
  }
}

void RobotLink::createCollision( const mechanism::Link &link)
{
  if (!link.collision_)
    return;

  const mechanism::Collision &collision = *link.collision_.get();
  collision_node_ = parent_->getCollisionNode()->createChildSceneNode();

  switch (collision.geometry_->type_)
  {
  case mechanism::Geometry::SPHERE:
  {
    const mechanism::Sphere *sphere = static_cast<mechanism::Sphere*>(collision.geometry_.get());
    ogre_tools::Shape* obj = new ogre_tools::Shape( ogre_tools::Shape::Sphere, scene_manager_, collision_node_ );

    Ogre::Vector3 scale( sphere->radius_, sphere->radius_, sphere->radius_ );

    obj->setScale( scale );
    collision_object_ = obj;
    break;
  }
  case mechanism::Geometry::BOX:
  {
    const mechanism::Box *box = static_cast<mechanism::Box*>(collision.geometry_.get());
    ogre_tools::Shape* obj = new ogre_tools::Shape( ogre_tools::Shape::Cube, scene_manager_, collision_node_ );

    Ogre::Vector3 scale( box->dim_[0], box->dim_[1], box->dim_[2] );
    robotToOgre( scale );

    obj->setScale( scale );
    collision_object_ = obj;

    break;
  }
  case mechanism::Geometry::CYLINDER:
  {
    const mechanism::Cylinder *cylinder = static_cast<mechanism::Cylinder*>(collision.geometry_.get());

    ogre_tools::Shape* obj = new ogre_tools::Shape( ogre_tools::Shape::Cylinder, scene_manager_, collision_node_ );
    Ogre::Vector3 scale( cylinder->radius_*2, cylinder->length_, cylinder->radius_*2 );

    obj->setScale( scale );

    collision_object_ = obj;
    break;
  }
  case mechanism::Geometry::MESH:
  {
    ROS_WARN("Mesh type is not supported for collisions");
    break;
  }
  default:
    ROS_WARN("Unsupported geometry type for collision element: %d", collision.geometry_->type_);
    break;
  }
}

void RobotLink::createVisual( const mechanism::Link &link )
{
  if (!link.visual_)
    return;
  if (link.visual_->geometry_->type_ != mechanism::Geometry::MESH)
    return;

  mechanism::Mesh *mesh = static_cast<mechanism::Mesh*>(link.visual_->geometry_.get());

  if ( mesh->filename_.empty() )
    return;

  std::string model_name = mesh->filename_ + ".mesh";

  static int count = 0;
  std::stringstream ss;
  ss << "RobotVis" << count++ << " Link " << link.name_ ;

  try
  {
    visual_mesh_ = scene_manager_->createEntity( ss.str(), model_name );
  }
  catch( Ogre::Exception& e )
  {
    printf( "Could not load model '%s' for link '%s': %s\n", model_name.c_str(), link.name_.c_str(), e.what() );
  }

  if ( visual_mesh_ )
  {
    visual_node_ = parent_->getVisualNode()->createChildSceneNode();
    visual_node_->attachObject( visual_mesh_ );

    std::string material_name = link.visual_->maps_.get("gazebo_material", "material");

    static int count = 0;
    std::stringstream ss;
    ss << material_name << count++ << "Robot";
    std::string cloned_name = ss.str();

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(material_name);
    if (material.isNull())
    {
      material = Ogre::MaterialManager::getSingleton().getByName("Gazebo/Red");
    }

    material->clone(cloned_name);

    material_name_ = cloned_name;
    visual_mesh_->setMaterialName( material_name_ );
  }
}

void RobotLink::createSelection(const mechanism::Robot& descr, const mechanism::Link &link)
{
  if (visual_mesh_)
  {
    selection_handler_ = RobotLinkSelectionHandlerPtr(new RobotLinkSelectionHandler(this));
    selection_object_ = vis_manager_->getSelectionManager()->createCollisionForEntity(visual_mesh_, selection_handler_);
  }
}

void RobotLink::createProperties()
{
  ROS_ASSERT( property_manager_ );

  std::stringstream ss;
  ss << name_ << " Link " << name_;

  CategoryPropertyWPtr cat = property_manager_->createCategory( name_, ss.str(), parent_->getLinksCategory(), this );


  trail_property_ = property_manager_->createProperty<BoolProperty>( "Show Trail", ss.str(), boost::bind( &RobotLink::getShowTrail, this ),
                                                                          boost::bind( &RobotLink::setShowTrail, this, _1 ), cat, this );

  axes_property_ = property_manager_->createProperty<BoolProperty>( "Show Axes", ss.str(), boost::bind( &RobotLink::getShowAxes, this ),
                                                                          boost::bind( &RobotLink::setShowAxes, this, _1 ), cat, this );

  position_property_ = property_manager_->createProperty<Vector3Property>( "Position", ss.str(), boost::bind( &RobotLink::getPositionInRobotFrame, this ),
                                                                                Vector3Property::Setter(), cat, this );
  orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", ss.str(), boost::bind( &RobotLink::getOrientationInRobotFrame, this ),
                                                                                      QuaternionProperty::Setter(), cat, this );

  joint_position_property_ = property_manager_->createProperty<DoubleProperty>( "Joint Position", ss.str(), boost::bind( &RobotLink::getJointPosition, this ),
                                                                                      DoubleProperty::Setter(), cat, this );
  joint_velocity_property_ = property_manager_->createProperty<DoubleProperty>( "Joint Velocity", ss.str(), boost::bind( &RobotLink::getJointVelocity, this ),
                                                                                      DoubleProperty::Setter(), cat, this );
  joint_applied_effort_property_ = property_manager_->createProperty<DoubleProperty>( "Joint Applied Effort", ss.str(), boost::bind( &RobotLink::getJointAppliedEffort, this ),
                                                                                            DoubleProperty::Setter(), cat, this );
  joint_commanded_effort_property_ = property_manager_->createProperty<DoubleProperty>( "Joint Commanded Effort", ss.str(), boost::bind( &RobotLink::getJointCommandedEffort, this ),
                                                                                              DoubleProperty::Setter(), cat, this );

  CategoryPropertyPtr cat_prop = cat.lock();
  cat_prop->collapse();
}


Ogre::Vector3 RobotLink::getPositionInRobotFrame()
{
  Ogre::Vector3 pos( position_ );
  ogreToRobot( pos );

  return pos;
}

Ogre::Quaternion RobotLink::getOrientationInRobotFrame()
{
  Ogre::Quaternion orient( orientation_ );
  ogreToRobot( orient );

  return orient;
}

void RobotLink::setShowTrail(bool show)
{
  if ( show )
  {
    if ( !trail_ )
    {
      if ( visual_node_ )
      {
        static int count = 0;
        std::stringstream ss;
        ss << "Trail for link " << name_ << count++;
        trail_ = scene_manager_->createRibbonTrail( ss.str() );
        trail_->setMaxChainElements( 100 );
        trail_->setInitialWidth( 0, 0.01f );
        trail_->setInitialColour( 0, 0.0f, 0.5f, 0.5f );
        trail_->addNode( visual_node_ );
        trail_->setTrailLength( 2.0f );
        parent_->getOtherNode()->attachObject( trail_ );
      }
      else
      {
        ROS_WARN( "No visual node for link %s, cannot create a trail", name_.c_str() );
      }
    }
  }
  else
  {
    if ( trail_ )
    {
      scene_manager_->destroyRibbonTrail( trail_ );
      trail_ = NULL;
    }
  }

  propertyChanged(trail_property_);
}

bool RobotLink::getShowTrail()
{
  return trail_ != NULL;
}

void RobotLink::setShowAxes(bool show)
{
  if ( show )
  {
    if ( !axes_ )
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for link " << name_ << count++;
      axes_ = new ogre_tools::Axes( scene_manager_, parent_->getOtherNode(), 0.1, 0.01 );
    }
  }
  else
  {
    if ( axes_ )
    {
      delete axes_;
      axes_ = NULL;
    }
  }

  propertyChanged(axes_property_);
}

bool RobotLink::getShowAxes()
{
  return axes_ != NULL;
}

void RobotLink::setTransforms( const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                          const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation, bool applyOffsetTransforms )
{
  position_ = visual_position;
  orientation_ = visual_orientation;

  if ( visual_node_ )
  {
    visual_node_->setPosition( visual_position );
    visual_node_->setOrientation( visual_orientation );
  }

  if ( collision_node_ )
  {
    Ogre::Quaternion initial_orientation;
    ogreToRobot( initial_orientation );

    if ( applyOffsetTransforms )
    {
      collision_object_->setPosition( collision_offset_position_ );
      collision_object_->setOrientation( initial_orientation * collision_offset_orientation_ );
    }
    else
    {
      collision_object_->setPosition( Ogre::Vector3::ZERO );
      collision_object_->setOrientation( initial_orientation );
    }

    collision_node_->setPosition( collision_position );
    collision_node_->setOrientation( collision_orientation );
  }

  if (property_manager_)
  {
    propertyChanged(position_property_);
    propertyChanged(orientation_property_);
  }


  if ( axes_ )
  {
    axes_->setPosition( position_ );
    axes_->setOrientation( orientation_ );
  }
}

void RobotLink::setToErrorMaterial()
{
  if (visual_mesh_)
  {
    visual_mesh_->setMaterialName("BaseWhiteNoLighting");
  }
}

void RobotLink::setToNormalMaterial()
{
  if (visual_mesh_)
  {
    visual_mesh_->setMaterialName(material_name_);
  }
}

void RobotLink::setJointState(const robot_msgs::JointState& state)
{
  joint_state_ = state;

  if ( property_manager_ )
  {
    propertyChanged(joint_applied_effort_property_);
    propertyChanged(joint_commanded_effort_property_);
    propertyChanged(joint_position_property_);
    propertyChanged(joint_velocity_property_);
  }
}

void RobotLink::setColor(float r, float g, float b, float a)
{
  // TODO: make this work on meshes as well?

  collision_object_->setColor(r, g, b, a);
}

} // namespace rviz

