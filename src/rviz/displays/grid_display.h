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

#ifndef OGRE_VISUALIZER_GRID_DISPLAY_H
#define OGRE_VISUALIZER_GRID_DISPLAY_H

#include "display.h"
#include "helpers/color.h"

namespace ogre_tools
{
class Grid;
}

class wxFocusEvent;
class wxCommandEvent;

class GridOptionsPanel;

namespace ogre_vis
{

class IntProperty;
class FloatProperty;
class ColorProperty;

/**
 * \class GridDisplay
 * \brief Displays a grid along the XZ plane (XY in robot space)
 *
 * For more information see ogre_tools::Grid
 */
class GridDisplay : public Display
{
public:
  GridDisplay( const std::string& name, VisualizationManager* manager );
  virtual ~GridDisplay();

  /**
   * @return The cell count for this grid
   */
  uint32_t getCellCount() { return cell_count_; }
  /**
   * @return The cell size for this grid
   */
  float getCellSize() { return cell_size_; }

  /**
   * \brief Set all the parameters of the grid
   * @param cell_count The number of cells
   * @param cell_size The size of each cell
   * @param color The color
   */
  void set( uint32_t cell_count, float cell_size, const Color& color );
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
  /**
   * \brief Set the color
   */
  void setColor( const Color& color );
  const Color& getColor() { return color_; }

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged() {}
  virtual void createProperties();

  static const char* getTypeStatic() { return "Grid"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();

protected:
  /**
   * \brief Creates the grid with the currently-set parameters
   */
  void create();

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  float cell_size_;                   ///< The size of each cell drawn.  Cells are square.
  uint32_t cell_count_;               ///< The number of rows/columns to draw.
  Color color_;
  ogre_tools::Grid* grid_;            ///< Handles actually drawing the grid

  IntProperty* cellcount_property_;
  FloatProperty* cellsize_property_;
  ColorProperty* color_property_;
};

} // namespace ogre_vis

 #endif
