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

#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreRibbonTrail.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSubEntity.h>
#include <OgreTextureManager.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>

#include <ros/console.h>

#include <resource_retriever/retriever.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>

#include <rviz/mesh_loader.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/object.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/robot/robot.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/load_resource.h>

#include <rviz/robot/robot_link.h>
#include <rviz/robot/robot_joint.h>

namespace fs = boost::filesystem;

namespace rviz
{
class RobotLinkSelectionHandler : public SelectionHandler
{
public:
  RobotLinkSelectionHandler(RobotLink* link, DisplayContext* context);
  ~RobotLinkSelectionHandler() override;

  void createProperties(const Picked& obj, Property* parent_property) override;
  void updateProperties() override;

  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;

private:
  RobotLink* link_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
};

RobotLinkSelectionHandler::RobotLinkSelectionHandler(RobotLink* link, DisplayContext* context)
  : SelectionHandler(context), link_(link)
{
}

RobotLinkSelectionHandler::~RobotLinkSelectionHandler()
{
}

void RobotLinkSelectionHandler::createProperties(const Picked& /*obj*/, Property* parent_property)
{
  Property* group =
      new Property("Link " + QString::fromStdString(link_->getName()), QVariant(), "", parent_property);
  properties_.push_back(group);

  position_property_ = new VectorProperty("Position", Ogre::Vector3::ZERO, "", group);
  position_property_->setReadOnly(true);

  orientation_property_ = new QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY, "", group);
  orientation_property_->setReadOnly(true);

  group->expand();
}

void RobotLinkSelectionHandler::updateProperties()
{
  position_property_->setVector(link_->getPosition());
  orientation_property_->setQuaternion(link_->getOrientation());
}


void RobotLinkSelectionHandler::preRenderPass(uint32_t /*pass*/)
{
  if (!link_->is_selectable_)
  {
    if (link_->visual_node_)
    {
      link_->visual_node_->setVisible(false);
    }
    if (link_->collision_node_)
    {
      link_->collision_node_->setVisible(false);
    }
    if (link_->trail_)
    {
      link_->trail_->setVisible(false);
    }
    if (link_->axes_)
    {
      link_->axes_->getSceneNode()->setVisible(false);
    }
  }
}

void RobotLinkSelectionHandler::postRenderPass(uint32_t /*pass*/)
{
  if (!link_->is_selectable_)
  {
    link_->updateVisibility();
  }
}

static std::map<const RobotLink*, std::string> errors;

RobotLink::RobotLink(Robot* robot,
                     const urdf::LinkConstSharedPtr& link,
                     const std::string& parent_joint_name,
                     bool visual,
                     bool collision)
  : robot_(robot)
  , scene_manager_(robot->getDisplayContext()->getSceneManager())
  , context_(robot->getDisplayContext())
  , name_(link->name)
  , parent_joint_name_(parent_joint_name)
  , visual_node_(nullptr)
  , collision_node_(nullptr)
  , trail_(nullptr)
  , axes_(nullptr)
  , robot_alpha_(1.0)
  , only_render_depth_(false)
  , is_selectable_(true)
  , material_mode_flags_(ORIGINAL)
{
  link_property_ =
      new Property(link->name.c_str(), true, "", nullptr, &RobotLink::updateVisibility, this);
  link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLink.png"));

  details_ = new Property("Details", QVariant(), "", nullptr);

  alpha_property_ = new FloatProperty("Alpha", 1, "Amount of transparency to apply to this link.",
                                      link_property_, &RobotLink::updateAlpha, this);

  trail_property_ =
      new Property("Show Trail", false, "Enable/disable a 2 meter \"ribbon\" which follows this link.",
                   link_property_, &RobotLink::updateTrail, this);

  axes_property_ = new Property("Show Axes", false, "Enable/disable showing the axes of this link.",
                                link_property_, &RobotLink::updateAxes, this);

  position_property_ =
      new VectorProperty("Position", Ogre::Vector3::ZERO,
                         "Position of this link, in the current Fixed Frame.  (Not editable)",
                         link_property_);
  position_property_->setReadOnly(true);

  orientation_property_ =
      new QuaternionProperty("Orientation", Ogre::Quaternion::IDENTITY,
                             "Orientation of this link, in the current Fixed Frame.  (Not editable)",
                             link_property_);
  orientation_property_->setReadOnly(true);

  link_property_->collapse();

  visual_node_ = robot_->getVisualNode()->createChildSceneNode();
  collision_node_ = robot_->getCollisionNode()->createChildSceneNode();

  // create material for coloring links
  std::string material_name = "robot link " + link->name + ":color material";
  color_material_ = Ogre::MaterialPtr(new Ogre::Material(
      nullptr, material_name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));
  color_material_->setReceiveShadows(false);
  color_material_->getTechnique(0)->setLightingEnabled(true);

  // create the ogre objects to display

  if (visual)
  {
    createVisual(link);
  }

  if (collision)
  {
    createCollision(link);
  }

  if (collision || visual)
  {
    createSelection();
  }

  // create description and fill in child_joint_names_ vector
  std::stringstream desc;
  if (parent_joint_name_.empty())
  {
    desc << "Root Link <b>" << name_ << "</b>";
  }
  else
  {
    desc << "Link <b>" << name_ << "</b>";
    desc << " with parent joint <b>" << parent_joint_name_ << "</b>";
  }

  if (link->child_joints.empty())
  {
    desc << " has no children.";
  }
  else
  {
    desc << " has " << link->child_joints.size();

    if (link->child_joints.size() > 1)
    {
      desc << " child joints: ";
    }
    else
    {
      desc << " child joint: ";
    }

    std::vector<urdf::JointSharedPtr>::const_iterator child_it = link->child_joints.begin();
    std::vector<urdf::JointSharedPtr>::const_iterator child_end = link->child_joints.end();
    for (; child_it != child_end; ++child_it)
    {
      urdf::Joint* child_joint = child_it->get();
      if (child_joint && !child_joint->name.empty())
      {
        child_joint_names_.push_back(child_joint->name);
        desc << "<b>" << child_joint->name << "</b>" << ((child_it + 1 == child_end) ? "." : ", ");
      }
    }
  }
  if (hasGeometry())
  {
    desc << "  Check/uncheck to show/hide this link in the display.";
    if (visual_meshes_.empty())
    {
      desc << "  This link has collision geometry but no visible geometry.";
    }
    else if (collision_meshes_.empty())
    {
      desc << "  This link has visible geometry but no collision geometry.";
    }
  }
  else
  {
    desc << "  This link has NO geometry.";
  }

  link_property_->setDescription(desc.str().c_str());

  if (!hasGeometry())
  {
    link_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotLinkNoGeom.png"));
    alpha_property_->hide();
    link_property_->setValue(QVariant());
  }
}

RobotLink::~RobotLink()
{
  for (size_t i = 0; i < visual_meshes_.size(); i++)
  {
    scene_manager_->destroyEntity(visual_meshes_[i]);
  }

  for (size_t i = 0; i < collision_meshes_.size(); i++)
  {
    scene_manager_->destroyEntity(collision_meshes_[i]);
  }

  scene_manager_->destroySceneNode(visual_node_);
  scene_manager_->destroySceneNode(collision_node_);

  if (trail_)
  {
    scene_manager_->destroyRibbonTrail(trail_);
  }

  delete axes_;
  delete details_;
  delete link_property_;
  errors.erase(this);
}

void RobotLink::addError(const char* format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  std::string& err = const_cast<std::string&>(getGeometryErrors());
  if (!err.empty())
    err.append("\n");
  err.append(buffer);
}

const std::string& RobotLink::getGeometryErrors() const
{
  return errors[this];
}

bool RobotLink::hasGeometry() const
{
  return visual_meshes_.size() + collision_meshes_.size() > 0;
}

bool RobotLink::getEnabled() const
{
  if (!hasGeometry())
    return true;
  return link_property_->getValue().toBool();
}

void RobotLink::setRobotAlpha(float a)
{
  robot_alpha_ = a;
  updateAlpha();
}

void RobotLink::setRenderQueueGroup(Ogre::uint8 group)
{
  Ogre::SceneNode::ChildNodeIterator child_it = visual_node_->getChildIterator();
  while (child_it.hasMoreElements())
  {
    Ogre::SceneNode* child = dynamic_cast<Ogre::SceneNode*>(child_it.getNext());
    if (child)
    {
      Ogre::SceneNode::ObjectIterator object_it = child->getAttachedObjectIterator();
      while (object_it.hasMoreElements())
      {
        Ogre::MovableObject* obj = object_it.getNext();
        obj->setRenderQueueGroup(group);
      }
    }
  }
}

void RobotLink::setOnlyRenderDepth(bool onlyRenderDepth)
{
  setRenderQueueGroup(onlyRenderDepth ? Ogre::RENDER_QUEUE_BACKGROUND : Ogre::RENDER_QUEUE_MAIN);
  only_render_depth_ = onlyRenderDepth;
  updateAlpha();
}

void RobotLink::updateAlpha()
{
  float link_alpha = alpha_property_->getFloat();
  for (auto& item : materials_)
  {
    Ogre::MaterialPtr& active = item.second.first;
    const Ogre::MaterialPtr& original = item.second.second;

    if (only_render_depth_)
    {
      active->setColourWriteEnabled(false);
      active->setDepthWriteEnabled(true);
    }
    else
    {
      Ogre::ColourValue color = active->getTechnique(0)->getPass(0)->getDiffuse();
      const float material_alpha = original->getTechnique(0)->getPass(0)->getDiffuse().a;
      color.a = robot_alpha_ * material_alpha * link_alpha;
      active->setDiffuse(color);

      if (color.a < 0.9998) // transparent
      {
        active->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        active->setDepthWriteEnabled(false);
      }
      else if (active == original)
      {
        active->setSceneBlending(Ogre::SBT_REPLACE);
        active->setDepthWriteEnabled(true);
      }
      else // restore original material
      {
        original->copyDetailsTo(active);
      }
    }
  }

  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = robot_alpha_ * link_alpha;
  color_material_->setDiffuse(color);

  if (color.a < 0.9998)
  {
    color_material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    color_material_->setDepthWriteEnabled(false);
  }
  else
  {
    color_material_->setSceneBlending(Ogre::SBT_REPLACE);
    color_material_->setDepthWriteEnabled(true);
  }
}

void RobotLink::updateVisibility()
{
  bool enabled = getEnabled();

  robot_->calculateJointCheckboxes();

  if (visual_node_)
  {
    visual_node_->setVisible(enabled && robot_->isVisible() && robot_->isVisualVisible());
  }
  if (collision_node_)
  {
    collision_node_->setVisible(enabled && robot_->isVisible() && robot_->isCollisionVisible());
  }
  if (trail_)
  {
    trail_->setVisible(enabled && robot_->isVisible());
  }
  if (axes_)
  {
    axes_->getSceneNode()->setVisible(enabled && robot_->isVisible());
  }
}

Ogre::MaterialPtr RobotLink::getMaterialForLink(const urdf::LinkConstSharedPtr& link,
                                                urdf::MaterialConstSharedPtr material)
{
  // only the first visual's material actually comprises color values, all others only have the name
  // hence search for the first visual with given material name (better fix the bug in urdf parser)
  if (material && !material->name.empty())
  {
    for (const auto& visual : link->visual_array)
    {
      if (visual->material_name == material->name)
      {
        material = visual->material;
        break;
      }
    }
  }
  if (!material && link->visual && link->visual->material)
    material = link->visual->material; // fallback to visual's material

  std::string name = "robot link " + link->name;
  if (material)
    name += ":" + material->name;

  Ogre::MaterialPtr mat = Ogre::MaterialPtr(
      new Ogre::Material(nullptr, name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));

  if (!material)
  {
    // clone default material (for modification by link)
    *mat = *Ogre::MaterialManager::getSingleton().getByName("RVIZ/ShadedRed");
    return mat;
  }

  mat->getTechnique(0)->setLightingEnabled(true);
  if (material->texture_filename.empty())
  {
    const urdf::Color& col = material->color;
    mat->getTechnique(0)->setAmbient(col.r * 0.5, col.g * 0.5, col.b * 0.5);
    mat->getTechnique(0)->setDiffuse(col.r, col.g, col.b, col.a);
  }
  else
  {
    std::string filename = material->texture_filename;
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
          Ogre::TextureManager::getSingleton().loadImage(
              filename, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }
        catch (Ogre::Exception& e)
        {
          ROS_ERROR("Could not load texture [%s]: %s", filename.c_str(), e.what());
        }
      }
    }

    Ogre::Pass* pass = mat->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* tex_unit = pass->createTextureUnitState();
    ;
    tex_unit->setTextureName(filename);
  }

  return mat;
}

void RobotLink::createEntityForGeometryElement(const urdf::LinkConstSharedPtr& link,
                                               const urdf::Geometry& geom,
                                               const urdf::MaterialSharedPtr& material,
                                               const urdf::Pose& origin,
                                               Ogre::SceneNode* scene_node,
                                               Ogre::Entity*& entity)
{
  entity = nullptr; // default in case nothing works.
  Ogre::SceneNode* offset_node = scene_node->createChildSceneNode();

  static unsigned count = 0;
  std::stringstream ss;
  ss << "Robot Link" << ++count;
  std::string entity_name = ss.str();

  Ogre::Vector3 scale(Ogre::Vector3::UNIT_SCALE);

  Ogre::Vector3 offset_position(Ogre::Vector3::ZERO);
  Ogre::Quaternion offset_orientation(Ogre::Quaternion::IDENTITY);


  {
    Ogre::Vector3 position(origin.position.x, origin.position.y, origin.position.z);
    Ogre::Quaternion orientation(Ogre::Quaternion::IDENTITY);
    orientation = orientation * Ogre::Quaternion(origin.rotation.w, origin.rotation.x, origin.rotation.y,
                                                 origin.rotation.z);

    offset_position = position;
    offset_orientation = orientation;
  }

  switch (geom.type)
  {
  case urdf::Geometry::SPHERE:
  {
    const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geom);
    entity = Shape::createEntity(entity_name, Shape::Sphere, scene_manager_);

    scale = Ogre::Vector3(sphere.radius * 2, sphere.radius * 2, sphere.radius * 2);
    break;
  }
  case urdf::Geometry::BOX:
  {
    const urdf::Box& box = static_cast<const urdf::Box&>(geom);
    entity = Shape::createEntity(entity_name, Shape::Cube, scene_manager_);

    scale = Ogre::Vector3(box.dim.x, box.dim.y, box.dim.z);

    break;
  }
  case urdf::Geometry::CYLINDER:
  {
    const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geom);

    Ogre::Quaternion rotX;
    rotX.FromAngleAxis(Ogre::Degree(90), Ogre::Vector3::UNIT_X);
    offset_orientation = offset_orientation * rotX;

    entity = Shape::createEntity(entity_name, Shape::Cylinder, scene_manager_);
    scale = Ogre::Vector3(cylinder.radius * 2, cylinder.length, cylinder.radius * 2);
    break;
  }
  case urdf::Geometry::MESH:
  {
    const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

    if (mesh.filename.empty())
      return;


    scale = Ogre::Vector3(mesh.scale.x, mesh.scale.y, mesh.scale.z);

    const std::string& model_name = mesh.filename;

    try
    {
      if (loadMeshFromResource(model_name).isNull())
        addError("Could not load mesh resource '%s'", model_name.c_str());
      else
        entity = scene_manager_->createEntity(ss.str(), model_name);
    }
    catch (Ogre::InvalidParametersException& e)
    {
      addError("Could not convert mesh resource '%s': %s", model_name.c_str(), e.what());
    }
    catch (Ogre::Exception& e)
    {
      addError("Could not load model '%s': %s", model_name.c_str(), e.what());
    }
    break;
  }
  default:
    ROS_WARN("Unsupported geometry type for element: %d", geom.type);
    break;
  }

  if (entity)
  {
    offset_node->attachObject(entity);
    offset_node->setScale(scale);
    offset_node->setPosition(offset_position);
    offset_node->setOrientation(offset_orientation);

    if (default_material_.isNull() || material)
    {
      // latest used material becomes the default for this link
      default_material_ = getMaterialForLink(link, material);
    }

    for (uint32_t i = 0; i < entity->getNumSubEntities(); ++i)
    {
      // Assign materials only if the submesh does not have one already
      Ogre::SubEntity* sub = entity->getSubEntity(i);
      const std::string& material_name = sub->getMaterialName();

      Ogre::MaterialPtr active, original;
      if (material_name == "BaseWhite" || material_name == "BaseWhiteNoLighting")
        original = default_material_;
      else
        original = sub->getMaterial();

      // create a new material copy for each instance of a RobotLink to allow modification per link
      active = Ogre::MaterialPtr(new Ogre::Material(
          nullptr, material_name, 0, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME));
      *active = *original;
      sub->setMaterial(active);
      materials_[sub] = std::make_pair(active, original);
    }
  }
}

void RobotLink::createCollision(const urdf::LinkConstSharedPtr& link)
{
  bool valid_collision_found = false;
#if URDF_MAJOR_VERSION == 0 && URDF_MINOR_VERSION == 2
  std::map<std::string, boost::shared_ptr<std::vector<urdf::CollisionSharedPtr> > >::const_iterator mi;
  for (mi = link->collision_groups.begin(); mi != link->collision_groups.end(); mi++)
  {
    if (mi->second)
    {
      std::vector<urdf::CollisionSharedPtr>::const_iterator vi;
      for (vi = mi->second->begin(); vi != mi->second->end(); vi++)
      {
        urdf::CollisionSharedPtr collision = *vi;
        if (collision && collision->geometry)
        {
          Ogre::Entity* collision_mesh = NULL;
          createEntityForGeometryElement(link, *collision->geometry, urdf::MaterialSharedPtr(),
                                         collision->origin, collision_node_, collision_mesh);
          if (collision_mesh)
          {
            collision_meshes_.push_back(collision_mesh);
            valid_collision_found = true;
          }
        }
      }
    }
  }
#else
  std::vector<urdf::CollisionSharedPtr>::const_iterator vi;
  for (vi = link->collision_array.begin(); vi != link->collision_array.end(); vi++)
  {
    urdf::CollisionSharedPtr collision = *vi;
    if (collision && collision->geometry)
    {
      Ogre::Entity* collision_mesh = nullptr;
      createEntityForGeometryElement(link, *collision->geometry, urdf::MaterialSharedPtr(),
                                     collision->origin, collision_node_, collision_mesh);
      if (collision_mesh)
      {
        collision_meshes_.push_back(collision_mesh);
      }
      valid_collision_found |= collision == link->collision; // don't consider the same geometry twice
    }
  }
#endif

  if (!valid_collision_found && link->collision && link->collision->geometry)
  {
    Ogre::Entity* collision_mesh = nullptr;
    createEntityForGeometryElement(link, *link->collision->geometry, urdf::MaterialSharedPtr(),
                                   link->collision->origin, collision_node_, collision_mesh);
    if (collision_mesh)
    {
      collision_meshes_.push_back(collision_mesh);
    }
  }

  collision_node_->setVisible(getEnabled());
}

void RobotLink::createVisual(const urdf::LinkConstSharedPtr& link)
{
  bool valid_visual_found = false;
#if URDF_MAJOR_VERSION == 0 && URDF_MINOR_VERSION == 2
  std::map<std::string, boost::shared_ptr<std::vector<urdf::VisualSharedPtr> > >::const_iterator mi;
  for (mi = link->visual_groups.begin(); mi != link->visual_groups.end(); mi++)
  {
    if (mi->second)
    {
      std::vector<urdf::VisualSharedPtr>::const_iterator vi;
      for (vi = mi->second->begin(); vi != mi->second->end(); vi++)
      {
        urdf::VisualSharedPtr visual = *vi;
        if (visual && visual->geometry)
        {
          Ogre::Entity* visual_mesh = NULL;
          createEntityForGeometryElement(link, *visual->geometry, visual->material, visual->origin,
                                         visual_node_, visual_mesh);
          if (visual_mesh)
          {
            visual_meshes_.push_back(visual_mesh);
            valid_visual_found = true;
          }
        }
      }
    }
  }
#else
  std::vector<urdf::VisualSharedPtr>::const_iterator vi;
  for (vi = link->visual_array.begin(); vi != link->visual_array.end(); vi++)
  {
    urdf::VisualSharedPtr visual = *vi;
    if (visual && visual->geometry)
    {
      Ogre::Entity* visual_mesh = nullptr;
      createEntityForGeometryElement(link, *visual->geometry, visual->material, visual->origin,
                                     visual_node_, visual_mesh);
      if (visual_mesh)
      {
        visual_meshes_.push_back(visual_mesh);
      }
      valid_visual_found |= visual == link->visual; // don't consider the same geometry again
    }
  }
#endif

  if (!valid_visual_found && link->visual && link->visual->geometry)
  {
    Ogre::Entity* visual_mesh = nullptr;
    createEntityForGeometryElement(link, *link->visual->geometry, link->visual->material,
                                   link->visual->origin, visual_node_, visual_mesh);
    if (visual_mesh)
    {
      visual_meshes_.push_back(visual_mesh);
    }
  }

  visual_node_->setVisible(getEnabled());
}

void RobotLink::createSelection()
{
  selection_handler_.reset(new RobotLinkSelectionHandler(this, context_));
  for (size_t i = 0; i < visual_meshes_.size(); i++)
  {
    selection_handler_->addTrackedObject(visual_meshes_[i]);
  }
  for (size_t i = 0; i < collision_meshes_.size(); i++)
  {
    selection_handler_->addTrackedObject(collision_meshes_[i]);
  }
}

void RobotLink::updateTrail()
{
  if (trail_property_->getValue().toBool())
  {
    if (!trail_)
    {
      if (visual_node_)
      {
        static int count = 0;
        std::stringstream ss;
        ss << "Trail for link " << name_ << count++;
        trail_ = scene_manager_->createRibbonTrail(ss.str());
        trail_->setMaxChainElements(100);
        trail_->setInitialWidth(0, 0.01f);
        trail_->setInitialColour(0, 0.0f, 0.5f, 0.5f);
        trail_->addNode(visual_node_);
        trail_->setTrailLength(2.0f);
        trail_->setVisible(getEnabled());
        robot_->getOtherNode()->attachObject(trail_);
      }
      else
      {
        ROS_WARN("No visual node for link %s, cannot create a trail", name_.c_str());
      }
    }
  }
  else
  {
    if (trail_)
    {
      scene_manager_->destroyRibbonTrail(trail_);
      trail_ = nullptr;
    }
  }
}

void RobotLink::updateAxes()
{
  if (axes_property_->getValue().toBool())
  {
    if (!axes_)
    {
      static int count = 0;
      std::stringstream ss;
      ss << "Axes for link " << name_ << count++;
      axes_ = new Axes(scene_manager_, robot_->getOtherNode(), 0.1, 0.01);
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

void RobotLink::setTransforms(const Ogre::Vector3& visual_position,
                              const Ogre::Quaternion& visual_orientation,
                              const Ogre::Vector3& collision_position,
                              const Ogre::Quaternion& collision_orientation)
{
  if (visual_node_)
  {
    visual_node_->setPosition(visual_position);
    visual_node_->setOrientation(visual_orientation);
  }

  if (collision_node_)
  {
    collision_node_->setPosition(collision_position);
    collision_node_->setOrientation(collision_orientation);
  }

  position_property_->setVector(visual_position);
  orientation_property_->setQuaternion(visual_orientation);

  if (axes_)
  {
    axes_->setPosition(visual_position);
    axes_->setOrientation(visual_orientation);
  }
}

void RobotLink::setToErrorMaterial()
{
  setMaterialMode(material_mode_flags_ | ERROR);
}

void RobotLink::setToNormalMaterial()
{
  setMaterialMode(material_mode_flags_ & ~ERROR);
}

void RobotLink::setMaterialMode(unsigned char mode_flags)
{
  if (material_mode_flags_ == mode_flags)
    return; // nothing to change

  material_mode_flags_ = mode_flags;

  if (mode_flags == ORIGINAL)
  {
    for (const auto& item : materials_)
      item.first->setMaterial(item.second.first);
    return;
  }

  auto error_material = Ogre::MaterialManager::getSingleton().getByName(
      "BaseWhiteNoLighting", Ogre::ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
  auto material = mode_flags == COLOR ? color_material_ : error_material;

  for (const auto& mesh : visual_meshes_)
    mesh->setMaterial(material);
  for (const auto& mesh : collision_meshes_)
    mesh->setMaterial(material);
}

void RobotLink::setColor(float red, float green, float blue)
{
  Ogre::ColourValue color = color_material_->getTechnique(0)->getPass(0)->getDiffuse();
  color.r = red;
  color.g = green;
  color.b = blue;
  color_material_->getTechnique(0)->setAmbient(0.5 * color);
  color_material_->getTechnique(0)->setDiffuse(color);

  setMaterialMode(COLOR | (material_mode_flags_ & ERROR));
}

void RobotLink::unsetColor()
{
  setMaterialMode(ORIGINAL | (material_mode_flags_ & ERROR));
}

bool RobotLink::setSelectable(bool selectable)
{
  bool old = is_selectable_;
  is_selectable_ = selectable;
  return old;
}

bool RobotLink::getSelectable()
{
  return is_selectable_;
}

void RobotLink::hideSubProperties(bool hide)
{
  position_property_->setHidden(hide);
  orientation_property_->setHidden(hide);
  trail_property_->setHidden(hide);
  axes_property_->setHidden(hide);
  alpha_property_->setHidden(hide);
}

Ogre::Vector3 RobotLink::getPosition()
{
  return position_property_->getVector();
}

Ogre::Quaternion RobotLink::getOrientation()
{
  return orientation_property_->getQuaternion();
}

void RobotLink::setParentProperty(Property* new_parent)
{
  Property* old_parent = link_property_->getParent();
  if (old_parent)
    old_parent->takeChild(link_property_);

  if (new_parent)
    new_parent->addChild(link_property_);
}

// if use_detail:
//    - all sub properties become children of details_ property.
//    - details_ property becomes a child of link_property_
// else (!use_detail)
//    - all sub properties become children of link_property_.
//    details_ property does not have a parent.
void RobotLink::useDetailProperty(bool use_detail)
{
  Property* old_parent = details_->getParent();
  if (old_parent)
    old_parent->takeChild(details_);

  if (use_detail)
  {
    while (link_property_->numChildren() > 0)
    {
      Property* child = link_property_->childAt(0);
      link_property_->takeChild(child);
      details_->addChild(child);
    }

    link_property_->addChild(details_);
  }
  else
  {
    while (details_->numChildren() > 0)
    {
      Property* child = details_->childAt(0);
      details_->takeChild(child);
      link_property_->addChild(child);
    }
  }
}

void RobotLink::expandDetails(bool expand)
{
  Property* parent = details_->getParent() ? details_ : link_property_;
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
