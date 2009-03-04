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

#include "grid_display.h"
#include "common.h"
#include "visualization_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_tools/grid.h"

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

GridDisplay::GridDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 0.5f, 0.5f, 0.5f )
, alpha_( 0.5f )
, plane_(XY)
, cell_count_property_( NULL )
, height_property_( NULL )
, cell_size_property_( NULL )
, color_property_( NULL )
, alpha_property_(NULL)
, offset_property_(NULL)
{
  grid_ = new ogre_tools::Grid( scene_manager_, manager->getTargetRelativeNode(), ogre_tools::Grid::Lines, 10, 1.0f, 0.03f, Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_) );
}

GridDisplay::~GridDisplay()
{
  delete grid_;
}

void GridDisplay::onEnable()
{
  grid_->getSceneNode()->setVisible( true );
}

void GridDisplay::onDisable()
{
  grid_->getSceneNode()->setVisible( false );
}

void GridDisplay::setCellSize( float size )
{
  grid_->setCellLength(size);

  if (cell_size_property_)
  {
    cell_size_property_->changed();
  }

  causeRender();
}

void GridDisplay::setCellCount( uint32_t count )
{
  grid_->setCellCount(count);

  if (cell_count_property_)
  {
    cell_count_property_->changed();
  }

  causeRender();
}

void GridDisplay::setColor( const Color& color )
{
  color_ = color;
  grid_->setColor(Ogre::ColourValue(color.r_, color.g_, color.b_, alpha_));

  if (color_property_)
  {
    color_property_->changed();
  }

  causeRender();
}

void GridDisplay::setLineWidth( float width )
{
  grid_->setLineWidth(width);

  if (line_width_property_)
  {
    line_width_property_->changed();
  }

  causeRender();
}

void GridDisplay::setStyle( int style )
{
  grid_->setStyle((ogre_tools::Grid::Style)style);

  if (style_property_)
  {
    style_property_->changed();
  }

  causeRender();
}

void GridDisplay::setAlpha( float a )
{
  alpha_ = a;

  grid_->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));

  if (alpha_property_)
  {
    alpha_property_->changed();
  }

  causeRender();
}

void GridDisplay::setHeight( uint32_t height )
{
  grid_->setHeight( height );

  if (height_property_)
  {
    height_property_->changed();
  }

  causeRender();
}

void GridDisplay::setOffset( const Ogre::Vector3& offset )
{
  offset_ = offset;

  Ogre::Vector3 robot_offset = offset;
  robotToOgre(robot_offset);

  grid_->getSceneNode()->setPosition(robot_offset);

  if (offset_property_)
  {
    offset_property_->changed();
  }

  causeRender();
}

void GridDisplay::setPlane(int plane)
{
  plane_ = (Plane)plane;

  if (plane_ == XY)
  {
    grid_->getSceneNode()->setOrientation(1.0f, 0.0f, 0.0f, 0.0f);
  }
  else if (plane_ == XZ)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(0.0f, -1.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, 1.0f), Ogre::Vector3(1.0f, 0.0f, 0.0f)));
  }
  else if (plane_ == YZ)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(1.0f, 0.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, -1.0f), Ogre::Vector3(0.0f, 1.0f, 0.0f)));
  }

  causeRender();
}

void GridDisplay::createProperties()
{
  cell_count_property_ = property_manager_->createProperty<IntProperty>( "Plane Cell Count", property_prefix_, boost::bind( &ogre_tools::Grid::getCellCount, grid_),
                                                           boost::bind( &GridDisplay::setCellCount, this, _1 ), parent_category_, this );
  cell_count_property_->setMin( 1 );
  cell_count_property_->addLegacyName("Cell Count");
  height_property_ = property_manager_->createProperty<IntProperty>( "Normal Cell Count", property_prefix_, boost::bind( &ogre_tools::Grid::getHeight, grid_),
                                                           boost::bind( &GridDisplay::setHeight, this, _1 ), parent_category_, this );
  height_property_->setMin( 0 );

  cell_size_property_ = property_manager_->createProperty<FloatProperty>( "Cell Size", property_prefix_, boost::bind( &ogre_tools::Grid::getCellLength, grid_ ),
                                                             boost::bind( &GridDisplay::setCellSize, this, _1 ), parent_category_, this );
  cell_size_property_->setMin( 0.0001 );

  style_property_ = property_manager_->createProperty<EnumProperty>( "Line Style", property_prefix_, boost::bind( &ogre_tools::Grid::getStyle, grid_ ),
                                                                   boost::bind( &GridDisplay::setStyle, this, _1 ), parent_category_, this );
  style_property_->addOption("Lines", ogre_tools::Grid::Lines);
  style_property_->addOption("Billboards", ogre_tools::Grid::Billboards);

  line_width_property_ = property_manager_->createProperty<FloatProperty>( "Line Width", property_prefix_, boost::bind( &ogre_tools::Grid::getLineWidth, grid_ ),
                                                               boost::bind( &GridDisplay::setLineWidth, this, _1 ), parent_category_, this );
  line_width_property_->setMin( 0.001 );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridDisplay::getColor, this ),
                                                                      boost::bind( &GridDisplay::setColor, this, _1 ), parent_category_, this );
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &GridDisplay::getAlpha, this ),
                                                                      boost::bind( &GridDisplay::setAlpha, this, _1 ), parent_category_, this );
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0f );

  plane_property_ = property_manager_->createProperty<EnumProperty>( "Plane", property_prefix_, boost::bind( &GridDisplay::getPlane, this ),
                                                                     boost::bind( &GridDisplay::setPlane, this, _1 ), parent_category_, this );
  plane_property_->addOption("XY", XY);
  plane_property_->addOption("XZ", XZ);
  plane_property_->addOption("YZ", YZ);

  offset_property_ = property_manager_->createProperty<Vector3Property>( "Offset", property_prefix_, boost::bind( &GridDisplay::getOffset, this ),
                                                                         boost::bind( &GridDisplay::setOffset, this, _1 ), parent_category_, this );
}

const char* GridDisplay::getDescription()
{
  return "Displays a grid along the ground plane, centered at the origin of the target frame of reference.";
}

} // namespace rviz
