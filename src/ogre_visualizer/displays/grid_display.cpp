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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace ogre_vis
{

GridDisplay::GridDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, cell_size_( 1.0f )
, cell_count_( 10 )
, color_( 0.5, 0.5, 0.5 )
, cellcount_property_( NULL )
, cellsize_property_( NULL )
, color_property_( NULL )
{
  grid_ = new ogre_tools::Grid( scene_manager_, manager->getTargetRelativeNode(), cell_count_, cell_size_, color_.r_, color_.g_, color_.b_ );

  /*Ogre::Quaternion orient( Ogre::Quaternion::IDENTITY );
  ogreToRobot( orient );
  grid_->getSceneNode()->setOrientation( orient );*/
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

void GridDisplay::create()
{
  grid_->set( cell_count_, cell_size_, color_.r_, color_.g_, color_.b_ );

  causeRender();
}

void GridDisplay::set( uint32_t cell_count, float cell_size, const Color& color )
{
  cell_count_ = cell_count;
  cell_size_ = cell_size;
  color_ = color;

  create();

  if ( cellcount_property_ )
  {
    cellcount_property_->changed();
  }

  if ( cellsize_property_ )
  {
    cellsize_property_->changed();
  }

  if ( color_property_ )
  {
    color_property_->changed();
  }
}

void GridDisplay::setCellSize( float size )
{
  set( cell_count_, size, color_ );
}

void GridDisplay::setCellCount( uint32_t count )
{
  set( count, cell_size_, color_ );
}

void GridDisplay::setColor( const Color& color )
{
  set( cell_count_, cell_size_, color );
}

void GridDisplay::createProperties()
{
  cellcount_property_ = property_manager_->createProperty<IntProperty>( "Cell Count", property_prefix_, boost::bind( &GridDisplay::getCellCount, this ),
                                                           boost::bind( &GridDisplay::setCellCount, this, _1 ), parent_category_, this );
  cellcount_property_->setMin( 1 );

  cellsize_property_ = property_manager_->createProperty<FloatProperty>( "Cell Size", property_prefix_, boost::bind( &GridDisplay::getCellSize, this ),
                                                             boost::bind( &GridDisplay::setCellSize, this, _1 ), parent_category_, this );
  cellsize_property_->setMin( 0.0001 );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridDisplay::getColor, this ),
                                                                      boost::bind( &GridDisplay::setColor, this, _1 ), parent_category_, this );
}

const char* GridDisplay::getDescription()
{
  return "Displays a grid along the ground plane, centered at the origin of the target frame of reference.";
}

} // namespace ogre_vis
