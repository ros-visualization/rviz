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

#include "object.h"

#ifndef OGRE_TOOLS_ARROW_H
#define OGRE_TOOLS_ARROW_H

#include <OgrePrerequisites.h>

namespace Ogre
{
class Any;
} // namespace Ogre


namespace rviz
{
class Shape;

/**
 * \class Arrow
 * \brief An arrow consisting of a cylinder and a cone.
 *
 * The base of the arrow is at the position sent to setPosition().
 * The arrow points in the direction of the negative Z axis by
 * default, and -Z is the identity direction of it.  To set a
 * different direction, call setOrientation() with a rotation from -Z
 * to the desired vector.
 */
class Arrow : public Object
{
public:
  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager to use to construct any necessary objects
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene
   * node.
   * @param shaft_length Length of the arrow's shaft
   * @param shaft_diameter Diameter of the arrow's shaft
   * @param head_length Length of the arrow's head
   * @param head_diameter Diameter of the arrow's head
   */
  Arrow(Ogre::SceneManager* scene_manager,
        Ogre::SceneNode* parent_node = nullptr,
        float shaft_length = 1.0f,
        float shaft_diameter = 0.1f,
        float head_length = 0.3f,
        float head_diameter = 0.2f);
  ~Arrow() override;

  /**
   * \brief Set the parameters for this arrow
   *
   * @param shaft_length Length of the arrow's shaft
   * @param shaft_diameter Diameter of the arrow's shaft
   * @param head_length Length of the arrow's head
   * @param head_diameter Diameter of the arrow's head
   */
  void set(float shaft_length, float shaft_diameter, float head_length, float head_diameter);

  /**
   * \brief Set the color of this arrow.  Sets both the head and shaft color to the same value.  Values
   * are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  void setColor(float r, float g, float b, float a) override;
  void setColor(const Ogre::ColourValue& color);

  /**
   * \brief Set the color of the arrow's head.  Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  void setHeadColor(float r, float g, float b, float a = 1.0f);
  void setHeadColor(const Ogre::ColourValue& color);
  /**
   * \brief Set the color of the arrow's shaft.  Values are in the range [0, 1]
   *
   * @param r Red component
   * @param g Green component
   * @param b Blue component
   */
  void setShaftColor(float r, float g, float b, float a = 1.0f);
  void setShaftColor(const Ogre::ColourValue& color);

  /** @brief Set the orientation.
   *
   * Note that negative Z is the identity orientation.
   *
   * Both setOrientation() and setDirection() change the direction the arrow points. */
  void setOrientation(const Ogre::Quaternion& orientation) override;

  /** @brief Set the position of the base of the arrow */
  void setPosition(const Ogre::Vector3& position) override;

  /** @brief Set the direction of the arrow.
   *
   * @param direction The direction the arrow should point, in the
   * coordinate frame of the parent Ogre::SceneNode.
   *
   * If direction is zero, this does not change the arrow.
   *
   * Both setOrientation() and setDirection() change the direction the arrow points. */
  void setDirection(const Ogre::Vector3& direction);

  void setScale(const Ogre::Vector3& scale) override;
  const Ogre::Vector3& getPosition() override;
  const Ogre::Quaternion& getOrientation() override;

  /**
   * \brief Get the scene node associated with this arrow
   * @return the scene node associated with this arrow
   */
  Ogre::SceneNode* getSceneNode()
  {
    return scene_node_;
  }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  void setUserData(const Ogre::Any& data) override;

  Shape* getShaft()
  {
    return shaft_;
  }
  Shape* getHead()
  {
    return head_;
  }

private:
  Ogre::SceneNode* scene_node_;

  Shape* shaft_; ///< Cylinder used for the shaft of the arrow
  Shape* head_;  ///< Cone used for the head of the arrow
};

} // namespace rviz

#endif /* OGRE_TOOLS_ARROW_H */
