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
#include "properties/property.h"
#include "properties/property_manager.h"
#include "visualization_manager.h"
#include "selection/selection_manager.h"
#include "mesh_loader.h"

#include "ogre_helpers/object.h"
#include "ogre_helpers/shape.h"
#include "ogre_helpers/axes.h"

#include <urdf/model.h>
#include <urdf_interface/link.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRibbonTrail.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTextureManager.h>

#include <ros/console.h>

#include <resource_retriever/retriever.h>

#include <boost/filesystem.hpp>

namespace fs=boost::filesystem;

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
}

RobotLink::RobotLink(Robot* parent, VisualizationManager* manager)
: parent_(parent)
, scene_manager_(manager->getSceneManager())
, property_manager_(0)
, vis_manager_(manager)
, enabled_( true )
, visual_mesh_( NULL )
, collision_mesh_( NULL )
, visual_node_( NULL )
, collision_node_( NULL )
, position_(Ogre::Vector3::ZERO)
, orientation_(Ogre::Quaternion::IDENTITY)
, trail_( NULL )
, axes_( NULL )
, material_alpha_( 1.0 )
, link_alpha_( 1.0 )
, selection_object_(NULL)
{
}

RobotLink::~RobotLink()
{
  if ( visual_mesh_ )
  {
    scene_manager_->destroyEntity( visual_mesh_ );
  }

  if ( collision_mesh_ )
  {
    scene_manager_->destroyEntity( collision_mesh_ );
  }

  if ( trail_ )
  {
    scene_manager_->destroyRibbonTrail( trail_ );
  }

  delete axes_;

  if (selection_object_)
  {
    vis_manager_->getSelectionManager()->removeObject(selection_object_);
  }

  if (property_manager_)
  {
    property_manager_->deleteByUserData(this);
  }
}

bool RobotLink::isValid()
{
  return visual_mesh_ || collision_mesh_;
}

void RobotLink::load(TiXmlElement* root_element, urdf::Model& descr, const urdf::LinkConstPtr& link, bool visual, bool collision)
{
  name_ = link->name;

  if ( visual )
  {
    createVisual( root_element, link );
  }

  if ( collision )
  {
    createCollision( root_element, link );
  }

  if (collision || visual)
  {
    createSelection( descr, link );
  }

  if ( property_manager_ )
  {
    createProperties();
  }
}

void RobotLink::setAlpha(float a)
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::updateAlpha()
{
  M_SubEntityToMaterial::iterator it = materials_.begin();
  M_SubEntityToMaterial::iterator end = materials_.end();
  for (; it != end; ++it)
  {
    const Ogre::MaterialPtr& material = it->second;

    Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
    color.a = robot_alpha_ * material_alpha_ * link_alpha_;
    material->setDiffuse( color );

    if ( color.a < 0.9998 )
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

void RobotLink::setLinkAlpha( float a )
{
  link_alpha_ = a;
  propertyChanged( alpha_property_ );
  updateAlpha();
}

float RobotLink::getLinkAlpha()
{
  return link_alpha_;
}

void RobotLink::setLinkEnabled( bool enabled )
{
  enabled_ = enabled;
  propertyChanged( link_property_ );
  updateVisibility();
}

void RobotLink::updateVisibility()
{
  if( visual_node_ )
  {
    visual_node_->setVisible( enabled_ && parent_->isVisualVisible() );
  }
  if( collision_node_ )
  {
    collision_node_->setVisible( enabled_ && parent_->isCollisionVisible() );
  }
  if( trail_ )
  {
    trail_->setVisible( enabled_ );
  }
  if( axes_ )
  {
    axes_->getSceneNode()->setVisible( enabled_ );
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink(TiXmlElement* root_element, const urdf::LinkConstPtr& link)
{
  if (!link->visual || !link->visual->material)
  {
    return Ogre::MaterialManager::getSingleton().getByName("RVIZ/Red");
  }

  static int count = 0;
  std::stringstream ss;
  ss << "Robot Link Material" << count;

  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), ROS_PACKAGE_NAME);
  mat->getTechnique(0)->setLightingEnabled(true);
  if (link->visual->material->texture_filename.empty())
  {
    const urdf::Color& col = link->visual->material->color;
    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
    mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);

    material_alpha_ = col.a;
  }
  else
  {
    std::string filename = link->visual->material->texture_filename;
    if (!Ogre::TextureManager::getSingleton().resourceExists(filename))
    {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
        res = retriever.get(filename);
      }
      catch (resource_retriever::Exception& e)
      {
        ROS_ERROR("%s", e.what());
      }

      if (res.size != 0)
      {
        Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
        Ogre::Image image;
        std::string extension = fs::extension(fs::path(filename));

        if (extension[0] == '.')
        {
          extension = extension.substr(1, extension.size() - 1);
        }

        try
        {
          image.load(stream, extension);
          Ogre::TextureManager::getSingleton().loadImage(filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(), e.what());
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();;
    tex_unit->setTextureName(filename);
  }

  return mat;
}

void RobotLink::createEntityForGeometryElement(TiXmlElement* root_element, const urdf::LinkConstPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, Ogre::SceneNode* parent_node, Ogre::Entity*& entity, Ogre::SceneNode*& scene_node, Ogre::SceneNode*& offset_node)
{
  scene_node = parent_node->createChildSceneNode();
  offset_node = scene_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "Robot Link" << count++;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);


  {
    Ogre::Vector3 position( origin.position.x, origin.position.y, origin.position.z );
    Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
    orientation = orientation * Ogre::Quaternion( origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z  );

    offset_position = position;
    offset_orientation = orientation;
  }

  switch (geom.type)
  {
  case urdf::Geometry::SPHERE:
  {
    const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geom);
    entity = Shape::createEntity(entity_name, Shape::Sphere, scene_manager_);

    scale = Ogre::Vector3( sphere.radius*2, sphere.radius*2, sphere.radius*2 );
    break;
  }
  case urdf::Geometry::BOX:
  {
    const urdf::Box& box = static_cast<const urdf::Box&>(geom);
    entity = Shape::createEntity(entity_name, Shape::Cube, scene_manager_);

    scale = Ogre::Vector3( box.dim.x, box.dim.y, box.dim.z );

    break;
  }
  case urdf::Geometry::CYLINDER:
  {
    const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geom);

    Ogre::Quaternion rotX;
    rotX.FromAngleAxis( Ogre::Degree(90), Ogre::Vector3::UNIT_X );
    offset_orientation = offset_orientation * rotX;

    entity = Shape::createEntity(entity_name, Shape::Cylinder, scene_manager_);
    scale = Ogre::Vector3( cylinder.radius*2, cylinder.length, cylinder.radius*2 );
    break;
  }
  case urdf::Geometry::MESH:
  {
    const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

    if ( mesh.filename.empty() )
      return;

    scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);

    std::string model_name = mesh.filename;
    loadMeshFromResource(model_name);

    try
    {
      entity = scene_manager_->createEntity( ss.str(), model_name );
    }
    catch( Ogre::Exception& e )
    {
      ROS_ERROR( "Could not load model '%s' for link '%s': %s\n", model_name.c_str(), link->name.c_str(), e.what() );
    }
    break;
  }
  default:
    ROS_WARN("Unsupported geometry type for element: %d", geom.type);
    break;
  }

  if ( entity )
  {
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    if (default_material_name_.empty())
    {
      default_material_ = getMaterialForLink(root_element, link);

      static int count = 0;
      std::stringstream ss;
      ss << default_material_->getName() << count++ << "Robot";
      std::string cloned_name = ss.str();

      default_material_ = default_material_->clone(cloned_name);
      default_material_name_ = default_material_->getName();
    }

    for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
    {
      // Assign materials only if the submesh does not have one already
      Ogre::SubEntity* sub = entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();

      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
      {
        sub->setMaterialName(default_material_name_);
      }
      else
      {
        // Need to clone here due to how selection works.  Once selection id is done per object and not per material,
        // this can go away
        static int count = 0;
        std::stringstream ss;
        ss << material_name << count++ << "Robot";
        std::string cloned_name = ss.str();
        sub->getMaterial()->clone(cloned_name);
        sub->setMaterialName(cloned_name);
      }

      materials_[sub] = sub->getMaterial();
    }
  }
}

void RobotLink::createCollision(TiXmlElement* root_element, const urdf::LinkConstPtr& link)
{
  if (!link->collision || !link->collision->geometry)
    return;

  createEntityForGeometryElement(root_element, link, *link->collision->geometry, link->collision->origin, parent_->getCollisionNode(), collision_mesh_, collision_node_, collision_offset_node_);
  collision_node_->setVisible( enabled_ );
}

void RobotLink::createVisual(TiXmlElement* root_element, const urdf::LinkConstPtr& link )
{
  if (!link->visual || !link->visual->geometry)
    return;

  createEntityForGeometryElement(root_element, link, *link->visual->geometry, link->visual->origin, parent_->getVisualNode(), visual_mesh_, visual_node_, visual_offset_node_);
  visual_node_->setVisible( enabled_ );
}

void RobotLink::createSelection(const urdf::Model& descr, const urdf::LinkConstPtr& link)
{
  selection_handler_ = RobotLinkSelectionHandlerPtr(new RobotLinkSelectionHandler(this));
  SelectionManager* sel_man = vis_manager_->getSelectionManager();
  selection_object_ = sel_man->createHandle();
  sel_man->addObject(selection_object_, selection_handler_);

  M_SubEntityToMaterial::iterator it = materials_.begin();
  M_SubEntityToMaterial::iterator end = materials_.end();
  for (; it != end; ++it)
  {
    const Ogre::MaterialPtr& material = it->second;

    sel_man->addPickTechnique(selection_object_, material);
  }

  if (visual_mesh_)
  {
    selection_handler_->addTrackedObject(visual_mesh_);
  }

  if (collision_mesh_)
  {
    selection_handler_->addTrackedObject(collision_mesh_);
  }
}

void RobotLink::setPropertyManager(PropertyManager* property_manager)
{
  property_manager_ = property_manager;
}

void RobotLink::createProperties()
{
  ROS_ASSERT( property_manager_ );

  std::stringstream ss;
  ss << parent_->getName() << " Link " << name_;

  link_property_ = property_manager_->createCheckboxCategory( name_, "Enabled", ss.str(),
                                                              boost::bind( &RobotLink::getLinkEnabled, this ),
                                                              boost::bind( &RobotLink::setLinkEnabled, this, _1 ),
                                                              parent_->getLinksCategory(), this );

  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", ss.str(),
                                                                      boost::bind( &RobotLink::getLinkAlpha, this ),
                                                                      boost::bind( &RobotLink::setLinkAlpha, this, _1 ),
                                                                      link_property_, this );
                                                                                   
  trail_property_ = property_manager_->createProperty<BoolProperty>( "Show Trail", ss.str(), boost::bind( &RobotLink::getShowTrail, this ),
                                                                          boost::bind( &RobotLink::setShowTrail, this, _1 ), link_property_, this );
  setPropertyHelpText(trail_property_, "Enable/disable a 2 meter \"ribbon\" which follows this link.");

  axes_property_ = property_manager_->createProperty<BoolProperty>( "Show Axes", ss.str(), boost::bind( &RobotLink::getShowAxes, this ),
                                                                          boost::bind( &RobotLink::setShowAxes, this, _1 ), link_property_, this );
  setPropertyHelpText(axes_property_, "Enable/disable showing the axes of this link.");

  position_property_ = property_manager_->createProperty<Vector3Property>( "Position", ss.str(), boost::bind( &RobotLink::getPositionInRobotFrame, this ),
                                                                                Vector3Property::Setter(), link_property_, this );
  setPropertyHelpText(position_property_, "Position of this link, in the current Fixed Frame.  (Not editable)");
  orientation_property_ = property_manager_->createProperty<QuaternionProperty>( "Orientation", ss.str(), boost::bind( &RobotLink::getOrientationInRobotFrame, this ),
                                                                                      QuaternionProperty::Setter(), link_property_, this );
  setPropertyHelpText(orientation_property_, "Orientation of this link, in the current Fixed Frame.  (Not editable)");

  CategoryPropertyPtr cat_prop = link_property_.lock();
  cat_prop->collapse();
}


Ogre::Vector3 RobotLink::getPositionInRobotFrame()
{
  return position_;
}

Ogre::Quaternion RobotLink::getOrientationInRobotFrame()
{
  return orientation_;
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
        trail_->setVisible( enabled_ );
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
      axes_ = new Axes( scene_manager_, parent_->getOtherNode(), 0.1, 0.01 );
      axes_->getSceneNode()->setVisible( enabled_ );
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
                          const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation, bool apply_offset_transforms )
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

  if (collision_mesh_)
  {
    collision_mesh_->setMaterialName("BaseWhiteNoLighting");
  }
}

void RobotLink::setToNormalMaterial()
{
  M_SubEntityToMaterial::iterator it = materials_.begin();
  M_SubEntityToMaterial::iterator end = materials_.end();
  for (; it != end; ++it)
  {
    it->first->setMaterial(it->second);
  }
}

} // namespace rviz

