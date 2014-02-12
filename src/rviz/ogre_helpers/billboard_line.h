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

#ifndef OGRE_TOOLS_BILLBOARD_LINE_H
#define OGRE_TOOLS_BILLBOARD_LINE_H

#include "object.h"

#include <stdint.h>

#include <vector>
#include <OgreVector3.h>
#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Quaternion;
class Any;
class BillboardChain;
}

namespace rviz
{

/**
 * \class BillboardLine
 * \brief An object that displays a multi-segment line strip rendered as billboards
 */
class BillboardLine : public Object
{
public:
  /**
   * \brief Constructor
   * @param manager Scene manager this object is a part of
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  BillboardLine( Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = NULL );
  virtual ~BillboardLine();

  void clear();
  void newLine();
  void addPoint(const Ogre::Vector3& point);
  void addPoint(const Ogre::Vector3& point, const Ogre::ColourValue& color);

  void setLineWidth( float width );

  void setMaxPointsPerLine(uint32_t max);
  void setNumLines(uint32_t num);

  // overrides from Object
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual void setColor( float r, float g, float b, float a );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

  /**
   * \brief Get the scene node associated with this object
   * @return The scene node associated with this object
   */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /**
   * \brief We have no objects that we can set user data on
   */
  void setUserData( const Ogre::Any& data ) {}

  Ogre::MaterialPtr getMaterial() { return material_; }

private:
  void setupChains();
  Ogre::BillboardChain* createChain();

  Ogre::SceneNode* scene_node_;

  typedef std::vector<Ogre::BillboardChain*> V_Chain;
  V_Chain chains_;
  Ogre::MaterialPtr material_;

  Ogre::ColourValue color_;
  float width_;

  uint32_t current_line_;

  // Ogre 1.4 doesn't have getNumChainElements()
  typedef std::vector<uint32_t> V_uint32;
  V_uint32 num_elements_;
  uint32_t total_elements_;

  uint32_t num_lines_;
  uint32_t max_points_per_line_;
  uint32_t lines_per_chain_;

  uint32_t current_chain_;
  uint32_t elements_in_current_chain_;
};

} // namespace rviz

#endif


