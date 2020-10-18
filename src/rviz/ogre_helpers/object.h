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

#ifndef OGRE_TOOLS_OBJECT_H
#define OGRE_TOOLS_OBJECT_H

#include "rviz/rviz_export.h"

namespace Ogre
{
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
class Any;
} // namespace Ogre

namespace rviz
{
/**
 * \class Object
 * \brief Base class for visible objects, providing a minimal generic interface.
 */
class RVIZ_EXPORT Object
{
public:
  /**
   *
   * @param scene_manager The scene manager this object should be part of.
   */
  Object(Ogre::SceneManager* scene_manager);
  virtual ~Object()
  {
  }

  /**
   * \brief Set the position of this object
   * @param Position vector position to set to.
   */
  virtual void setPosition(const Ogre::Vector3& position) = 0;

  /**
   * \brief Set the orientation of the object
   * @param Orientation quaternion orientation to set to.
   */
  virtual void setOrientation(const Ogre::Quaternion& orientation) = 0;

  /**
   * \brief Set the scale of the object.  Always relative to the identity orientation of the object.
   * @param Scale vector scale to set to.
   */
  virtual void setScale(const Ogre::Vector3& scale) = 0;

  /**
   * \brief Set the color of the object.  Values are in the range [0, 1]
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  virtual void setColor(float r, float g, float b, float a) = 0;

  /**
   * \brief Get the local position of this object
   * @return The position
   */
  virtual const Ogre::Vector3& getPosition() = 0;
  /**
   * \brief Get the local orientation of this object
   * @return The orientation
   */
  virtual const Ogre::Quaternion& getOrientation() = 0;

  /**
   * \brief Set the user data on this object
   * @param data
   */
  virtual void setUserData(const Ogre::Any& data) = 0;

protected:
  Ogre::SceneManager* scene_manager_; ///< Ogre scene manager this object is part of
};

} // namespace rviz

#endif /* OGRE_TOOLS_OBJECT_H */
