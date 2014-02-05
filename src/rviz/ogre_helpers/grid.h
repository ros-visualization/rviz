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

#ifndef OGRE_TOOLS_GRID_H
#define OGRE_TOOLS_GRID_H

#include <stdint.h>

#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

namespace Ogre
{
class SceneManager;

class ManualObject;
class SceneNode;

class Any;
}

namespace rviz
{

class BillboardLine;

/**
 * \class Grid
 * \brief Displays a grid of cells, drawn with lines
 *
 * Displays a grid of cells, drawn with lines.  A grid with an identity orientation is drawn along the XZ plane.
 */
class Grid
{
public:
  enum Style
  {
    Lines,
    Billboards,
  };

  /**
   * \brief Constructor
   *
   * @param manager The scene manager this object is part of
   * @param cell_count The number of cells to draw
   * @param cell_length The size of each cell
   * @param r Red color component, in the range [0, 1]
   * @param g Green color component, in the range [0, 1]
   * @param b Blue color component, in the range [0, 1]
   */
  Grid( Ogre::SceneManager* manager, Ogre::SceneNode* parent_node, Style style, uint32_t cell_count, float cell_length, float line_width, const Ogre::ColourValue& color );
  ~Grid();

  void create();

  /**
   * \brief Get the Ogre scene node associated with this grid
   *
   * @return The Ogre scene node associated with this grid
   */
  Ogre::SceneNode* getSceneNode() { return scene_node_; }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  void setUserData( const Ogre::Any& data );

  void setStyle(Style style);
  Style getStyle() { return style_; }

  void setColor(const Ogre::ColourValue& color);
  Ogre::ColourValue getColor() { return color_; }

  void setCellCount(uint32_t count);
  float getCellCount() { return cell_count_; }

  void setCellLength(float len);
  float getCellLength() { return cell_length_; }

  void setLineWidth(float width);
  float getLineWidth() { return line_width_; }

  void setHeight(uint32_t count);
  uint32_t getHeight() { return height_; }

private:
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;           ///< The scene node that this grid is attached to
  Ogre::ManualObject* manual_object_;     ///< The manual object used to draw the grid

  BillboardLine* billboard_line_;

  Ogre::MaterialPtr material_;

  Style style_;
  uint32_t cell_count_;
  float cell_length_;
  float line_width_;
  uint32_t height_;
  Ogre::ColourValue color_;
};

} // namespace rviz

#endif
