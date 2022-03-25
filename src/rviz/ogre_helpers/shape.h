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

#ifndef OGRE_TOOLS_SHAPE_H
#define OGRE_TOOLS_SHAPE_H

#include "object.h"

#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
class Entity;
} // namespace Ogre

namespace rviz
{
/**
 */
class Shape : public Object
{
public:
  enum Type
  {
    Cone,
    Cube,
    Cylinder,
    Sphere,
    Mesh,
  };

  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene
   * node.
   */
  Shape(Type shape_type, Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);
  ~Shape() override;

  Type getType()
  {
    return type_;
  }

  /**
   * \brief Set the offset for this shape
   *
   * The default is no offset, which puts the pivot point directly in the center of the object.
   *
   * @param offset Amount to offset the center of the object from the pivot point
   */
  void setOffset(const Ogre::Vector3& offset);

  void setColor(float r, float g, float b, float a) override;
  void setColor(const Ogre::ColourValue& c);
  void setPosition(const Ogre::Vector3& position) override;
  void setOrientation(const Ogre::Quaternion& orientation) override;
  void setScale(const Ogre::Vector3& scale) override;
  const Ogre::Vector3& getPosition() override;
  const Ogre::Quaternion& getOrientation() override;

  /**
   * \brief Get the root scene node (pivot node) for this object
   *
   * @return The root scene node of this object
   */
  Ogre::SceneNode* getRootNode()
  {
    return scene_node_;
  }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  void setUserData(const Ogre::Any& data) override;

  Ogre::Entity* getEntity()
  {
    return entity_;
  }

  Ogre::MaterialPtr getMaterial()
  {
    return material_;
  }

  static Ogre::Entity*
  createEntity(const std::string& name, Type shape_type, Ogre::SceneManager* scene_manager);

protected:
  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::Entity* entity_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  Type type_;
};

} // namespace rviz

#endif
