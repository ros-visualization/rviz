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

#ifndef RVIZ_GRID_DISPLAY_H
#define RVIZ_GRID_DISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"
#include <OGRE/OgreVector3.h>

#include <boost/signals.hpp>

namespace ogre_tools
{
class Grid;
}

class wxFocusEvent;
class wxCommandEvent;

namespace rviz
{

/**
 * \class GridDisplay
 * \brief Displays a grid along the XZ plane (XY in robot space)
 *
 * For more information see ogre_tools::Grid
 */
class GridDisplay : public Display
{
public:
  enum Plane
  {
    XY,
    XZ,
    YZ,
  };

  GridDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~GridDisplay();

  /**
   * \brief Set the number of cells
   * @param cell_count The number of cells
   */
  void setCellCount( uint32_t cell_count );
  /**
   * \brief Set the cell size
   * @param cell_size The cell size
   */
  void setCellSize( float cell_size );
  void setHeight(uint32_t height);
  /**
   * \brief Set the color
   */
  void setColor( const Color& color );
  const Color& getColor() { return color_; }

  void setStyle(int style);
  void setLineWidth(float width);

  void setAlpha(float a);
  float getAlpha() { return alpha_; }

  void setOffset(const Ogre::Vector3& offset);
  Ogre::Vector3 getOffset() { return offset_; }

  void setPlane( int plane );
  Plane getPlane() { return plane_; }

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged() {}
  virtual void createProperties();

protected:
  /**
   * \brief Creates the grid with the currently-set parameters
   */
  void create();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  Color color_;
  float alpha_;
  ogre_tools::Grid* grid_;            ///< Handles actually drawing the grid

  Ogre::Vector3 offset_;
  Plane plane_;

  IntPropertyWPtr cell_count_property_;
  IntPropertyWPtr height_property_;
  FloatPropertyWPtr cell_size_property_;
  FloatPropertyWPtr line_width_property_;
  EnumPropertyWPtr style_property_;
  ColorPropertyWPtr color_property_;
  FloatPropertyWPtr alpha_property_;
  EnumPropertyWPtr plane_property_;
  Vector3PropertyWPtr offset_property_;
};

} // namespace rviz

 #endif
