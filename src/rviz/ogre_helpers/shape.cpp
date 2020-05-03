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

#include "shape.h"
#include <ros/assert.h>

#include <rviz/ogre_helpers/version_check.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <stdint.h>

namespace rviz
{
Ogre::Entity* Shape::createEntity(const std::string& name, Type type, Ogre::SceneManager* scene_manager)
{
  if (type == Mesh)
    return nullptr; // the entity is initialized after the vertex data was specified

  std::string mesh_name;
  switch (type)
  {
  case Cone:
    mesh_name = "rviz_cone.mesh";
    break;

  case Cube:
    mesh_name = "rviz_cube.mesh";
    break;

  case Cylinder:
    mesh_name = "rviz_cylinder.mesh";
    break;

  case Sphere:
    mesh_name = "rviz_sphere.mesh";
    break;

  default:
    ROS_BREAK();
  }

  return scene_manager->createEntity(name, mesh_name);
}

Shape::Shape(Type type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : Object(scene_manager), type_(type)
{
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "Shape" << count++;

  entity_ = createEntity(ss.str(), type, scene_manager);

  if (!parent_node)
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();
  offset_node_ = scene_node_->createChildSceneNode();
  if (entity_)
    offset_node_->attachObject(entity_);

  ss << "Material";
  material_name_ = ss.str();
  material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(true);
  material_->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);

  if (entity_)
    entity_->setMaterial(material_);
}

Shape::~Shape()
{
  scene_manager_->destroySceneNode(scene_node_);
  scene_manager_->destroySceneNode(offset_node_);

  if (entity_)
    scene_manager_->destroyEntity(entity_);

  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

void Shape::setColor(const Ogre::ColourValue& c)
{
  material_->getTechnique(0)->setAmbient(c * 0.5);
  material_->getTechnique(0)->setDiffuse(c);

  if (c.a < 0.9998)
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

void Shape::setColor(float r, float g, float b, float a)
{
  setColor(Ogre::ColourValue(r, g, b, a));
}

void Shape::setOffset(const Ogre::Vector3& offset)
{
  offset_node_->setPosition(offset);
}

void Shape::setPosition(const Ogre::Vector3& position)
{
  scene_node_->setPosition(position);
}

void Shape::setOrientation(const Ogre::Quaternion& orientation)
{
  scene_node_->setOrientation(orientation);
}

void Shape::setScale(const Ogre::Vector3& scale)
{
  scene_node_->setScale(scale);
}

const Ogre::Vector3& Shape::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& Shape::getOrientation()
{
  return scene_node_->getOrientation();
}

void Shape::setUserData(const Ogre::Any& data)
{
  if (entity_)
    entity_->getUserObjectBindings().setUserAny(data);
  else
    ROS_ERROR("Shape not yet fully constructed. Cannot set user data. Did you add triangles to the mesh "
              "already?");
}

} // namespace rviz
