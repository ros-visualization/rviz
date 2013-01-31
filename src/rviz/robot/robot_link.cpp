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

#include "rviz/mesh_loader.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/ogre_helpers/object.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/robot/robot.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"

#include "rviz/robot/robot_link.h"

namespace fs=boost::filesystem;

namespace rviz
{

class RobotLinkSelectionHandler : public SelectionHandler
{
public:
  RobotLinkSelectionHandler( RobotLink* link, DisplayContext* context );
  virtual ~RobotLinkSelectionHandler();

  virtual void createProperties( const Picked& obj, Property* parent_property );
  virtual void updateProperties();

private:
  RobotLink* link_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
};

RobotLinkSelectionHandler::RobotLinkSelectionHandler( RobotLink* link, DisplayContext* context )
  : SelectionHandler( context )
  , link_( link )
{
}

RobotLinkSelectionHandler::~RobotLinkSelectionHandler()
{
}

void RobotLinkSelectionHandler::createProperties( const Picked& obj, Property* parent_property )
{
  Property* group = new Property( "Link " + QString::fromStdString( link_->getName() ), QVariant(), "", parent_property );
  properties_.push_back( group );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO, "", group );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY, "", group );
  orientation_property_->setReadOnly( true );

  group->expand();
}

void RobotLinkSelectionHandler::updateProperties()
{
  position_property_->setVector( link_->getPosition() );
  orientation_property_->setQuaternion( link_->getOrientation() );
}

RobotLink::RobotLink( Robot* parent, DisplayContext* context, Property* parent_property )
: parent_( parent )
, scene_manager_( context->getSceneManager() )
, context_( context )
, visual_mesh_( NULL )
, collision_mesh_( NULL )
, visual_node_( NULL )
, collision_node_( NULL )
, trail_( NULL )
, axes_( NULL )
, material_alpha_( 1.0 )
, robot_alpha_(1.0)
, only_render_depth_(false)
, using_color_( false )
{
  link_property_ = new Property( "", true, "", parent_property, SLOT( updateVisibility() ), this );

  alpha_property_ = new FloatProperty( "Alpha", 1,
                                       "Amount of transparency to apply to this link.",
                                       link_property_, SLOT( updateAlpha() ), this );
                                                                                   
  trail_property_ = new Property( "Show Trail", false,
                                  "Enable/disable a 2 meter \"ribbon\" which follows this link.",
                                  link_property_, SLOT( updateTrail() ), this );

  axes_property_ = new Property( "Show Axes", false,
                                 "Enable/disable showing the axes of this link.",
                                 link_property_, SLOT( updateAxes() ), this );

  position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO,
                                           "Position of this link, in the current Fixed Frame.  (Not editable)",
                                           link_property_ );
  position_property_->setReadOnly( true );

  orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY,
                                                  "Orientation of this link, in the current Fixed Frame.  (Not editable)",
                                                  link_property_ );
  orientation_property_->setReadOnly( true );

  link_property_->collapse();

  std::stringstream ss;
  static int count = 1;
  ss << "robot link color material " << count;
  color_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), ROS_PACKAGE_NAME );
  color_material_->setReceiveShadows(false);
  color_material_->getTechnique(0)->setLightingEnabled(true);
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
  delete link_property_;
}

bool RobotLink::isValid()
{
  return visual_mesh_ || collision_mesh_;
}

bool RobotLink::getEnabled() const
{
  return link_property_->getValue().toBool();
}

void RobotLink::load( const urdf::ModelInterface& descr, const urdf::LinkConstPtr& link, bool visual, bool collision)
{
  name_ = link->name;
  link_property_->setName( QString::fromStdString( name_ ));

  if ( visual )
  {
    createVisual( link );
  }

  if ( collision )
  {
    createCollision( link );
  }

  if (collision || visual)
  {
    createSelection();
  }
}

void RobotLink::setRobotAlpha( float a )
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::setRenderQueueGroup( Ogre::uint8 group )
{
  Ogre::SceneNode::ObjectIterator it = visual_offset_node_->getAttachedObjectIterator();
  while( it.hasMoreElements() )
  {
    Ogre::MovableObject* obj = it.getNext();
    obj->setRenderQueueGroup(group);
  }

}

void RobotLink::setOnlyRenderDepth(bool onlyRenderDepth)
{
  setRenderQueueGroup( onlyRenderDepth ? Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN );
  only_render_depth_ = onlyRenderDepth;
  updateAlpha();
}

void RobotLink::updateAlpha()
{
  float link_alpha = alpha_property_->getFloat();
  M_SubEntityToMaterial::iterator it = materials_.begin();
  M_SubEntityToMaterial::iterator end = materials_.end();
  for (; it != end; ++it)
  {
    const Ogre::MaterialPtr& material = it->second;

    if ( only_render_depth_ )
    {
      material->setColourWriteEnabled( false );
      material->setDepthWriteEnabled( true );
    }
    else
    {
      Ogre::ColourValue color = material->getTechnique(0)->getPass(0)->getDiffuse();
      color.a = robot_alpha_ * material_alpha_ * link_alpha;
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
}

void RobotLink::updateVisibility()
{
  bool enabled = getEnabled();
  if( visual_node_ )
  {
    visual_node_->setVisible( enabled && parent_->isVisualVisible() );
  }
  if( collision_node_ )
  {
    collision_node_->setVisible( enabled && parent_->isCollisionVisible() );
  }
  if( trail_ )
  {
    trail_->setVisible( enabled );
  }
  if( axes_ )
  {
    axes_->getSceneNode()->setVisible( enabled );
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink( const urdf::LinkConstPtr& link)
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

void RobotLink::createEntityForGeometryElement(const urdf::LinkConstPtr& link, const urdf::Geometry& geom, const urdf::Pose& origin, Ogre::SceneNode* parent_node, Ogre::Entity*& entity, Ogre::SceneNode*& scene_node, Ogre::SceneNode*& offset_node)
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
      default_material_ = getMaterialForLink(link);

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

void RobotLink::createCollision(const urdf::LinkConstPtr& link)
{
  if (!link->collision || !link->collision->geometry)
    return;

  createEntityForGeometryElement(link, *link->collision->geometry, link->collision->origin, parent_->getCollisionNode(), collision_mesh_, collision_node_, collision_offset_node_);
  collision_node_->setVisible( getEnabled() );
}

void RobotLink::createVisual(const urdf::LinkConstPtr& link )
{
  if (!link->visual || !link->visual->geometry)
    return;

  createEntityForGeometryElement(link, *link->visual->geometry, link->visual->origin, parent_->getVisualNode(), visual_mesh_, visual_node_, visual_offset_node_);
  visual_node_->setVisible( getEnabled() );
}

void RobotLink::createSelection()
{
  selection_handler_.reset( new RobotLinkSelectionHandler( this, context_ ));
  if( visual_mesh_ )
  {
    selection_handler_->addTrackedObject( visual_mesh_ );
  }
  if( collision_mesh_ )
  {
    selection_handler_->addTrackedObject( collision_mesh_ );
  }
}

void RobotLink::updateTrail()
{
  if( trail_property_->getValue().toBool() )
  {
    if( !trail_ )
    {
      if( visual_node_ )
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
        trail_->setVisible( getEnabled() );
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
    if( trail_ )
    {
      scene_manager_->destroyRibbonTrail( trail_ );
      trail_ = NULL;
    }
  }
}

void RobotLink::updateAxes()
{
  if( axes_property_->getValue().toBool() )
  {
    if( !axes_ )
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for link " << name_ << count++;
      axes_ = new Axes( scene_manager_, parent_->getOtherNode(), 0.1, 0.01 );
      axes_->getSceneNode()->setVisible( getEnabled() );
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

void RobotLink::setTransforms( const Ogre::Vector3& visual_position, const Ogre::Quaternion& visual_orientation,
                               const Ogre::Vector3& collision_position, const Ogre::Quaternion& collision_orientation )
{
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

  position_property_->setVector( visual_position );
  orientation_property_->setQuaternion( visual_orientation );

  if ( axes_ )
  {
    axes_->setPosition( visual_position );
    axes_->setOrientation( visual_orientation );
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
  if( using_color_ )
  {
    if (visual_mesh_)
    {
      visual_mesh_->setMaterial( color_material_ );
    }
    if (collision_mesh_)
    {
      collision_mesh_->setMaterial( color_material_ );
    }
    if ( visual_node_ )
    {
      visual_node_->setScale( 1, 1, 1.0001 );
    }
    if ( collision_node_ )
    {
      collision_node_->setScale( 1, 1, 1.0001 );
    }
  }
  else
  {
    if ( visual_node_ )
    {
      visual_node_->setScale( 1, 1, 1 );
    }
    if ( collision_node_ )
    {
      collision_node_->setScale( 1, 1, 1 );
    }
    M_SubEntityToMaterial::iterator it = materials_.begin();
    M_SubEntityToMaterial::iterator end = materials_.end();
    for (; it != end; ++it)
    {
      it->first->setMaterial(it->second);
    }
  }
}

void RobotLink::setColor( float red, float green, float blue )
{
  Ogre::ColourValue color( red, green, blue );
  color_material_->getTechnique(0)->setAmbient( 0.5 * color );
  color_material_->getTechnique(0)->setDiffuse( color );

  using_color_ = true;
  setToNormalMaterial();
}

void RobotLink::unsetColor()
{
  using_color_ = false;
  setToNormalMaterial();
}

Ogre::Vector3 RobotLink::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotLink::getOrientation()
{
  return orientation_property_->getQuaternion();
}

} // namespace rviz

