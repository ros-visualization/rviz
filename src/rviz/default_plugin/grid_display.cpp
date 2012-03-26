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

#include <stdint.h>
#include "grid_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"

#include "rviz/ogre_helpers/grid.h"

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

GridDisplay::GridDisplay()
: Display()
, color_( 0.5f, 0.5f, 0.5f )
, alpha_( 0.5f )
, plane_(XY)
, scene_node_( 0 )
{
  offset_ = Ogre::Vector3::ZERO;
}

GridDisplay::~GridDisplay()
{
  delete grid_;
  scene_manager_->destroySceneNode(scene_node_);
}

void GridDisplay::onInitialize()
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  grid_ = new Grid( scene_manager_, scene_node_, Grid::Lines, 10, 1.0f, 0.03f,
                                Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_) );
  grid_->getSceneNode()->setVisible( false );
  setStyle(Grid::Lines);
  setFrame(FIXED_FRAME_STRING);
  setPlane( XY );
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

  propertyChanged(cell_size_property_);

  causeRender();
}

void GridDisplay::setCellCount( uint32_t count )
{
  grid_->setCellCount(count);

  propertyChanged(cell_count_property_);

  causeRender();
}

void GridDisplay::setColor( const Color& color )
{
  color_ = color;
  grid_->setColor(Ogre::ColourValue(color.r_, color.g_, color.b_, alpha_));

  propertyChanged(color_property_);

  causeRender();
}

void GridDisplay::setLineWidth( float width )
{
  grid_->setLineWidth(width);

  propertyChanged(line_width_property_);

  causeRender();
}

void GridDisplay::setStyle( int style )
{
  grid_->setStyle((Grid::Style)style);

  switch (style)
  {
  case Grid::Billboards:
    showProperty(line_width_property_);
    break;
  case Grid::Lines:
    hideProperty(line_width_property_);
    break;
  }

  propertyChanged(style_property_);

  causeRender();
}

void GridDisplay::setAlpha( float a )
{
  alpha_ = a;

  grid_->setColor(Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_));

  propertyChanged(alpha_property_);

  causeRender();
}

void GridDisplay::setHeight( uint32_t height )
{
  grid_->setHeight( height );

  propertyChanged(height_property_);

  causeRender();
}

void GridDisplay::setOffset( const Ogre::Vector3& offset )
{
  offset_ = offset;

  Ogre::Vector3 ogre_offset = offset;

  grid_->getSceneNode()->setPosition(ogre_offset);

  propertyChanged(offset_property_);

  causeRender();
}

void GridDisplay::setPlane(int plane)
{
  plane_ = (Plane)plane;

  if (plane_ == XY)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(1.0f, 0.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, -1.0f), Ogre::Vector3(0.0f, 1.0f, 0.0f)));
  }
  else if (plane_ == XZ)
  {
    grid_->getSceneNode()->setOrientation(1.0f, 0.0f, 0.0f, 0.0f);
  }
  else if (plane_ == YZ)
  {
    grid_->getSceneNode()->setOrientation(Ogre::Quaternion(Ogre::Vector3(0.0f, -1.0f, 0.0f), Ogre::Vector3(0.0f, 0.0f, 1.0f), Ogre::Vector3(1.0f, 0.0f, 0.0f)));
  }

  propertyChanged(plane_property_);

  causeRender();
}

void GridDisplay::setFrame(const std::string& frame)
{
  frame_ = frame;
  propertyChanged(frame_property_);
}

void GridDisplay::update(float dt, float ros_dt)
{
  std::string frame = frame_;
  if (frame == FIXED_FRAME_STRING)
  {
    frame = fixed_frame_;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (vis_manager_->getFrameManager()->getTransform(frame, ros::Time(), position, orientation))
  {
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    setStatus(status_levels::Ok, "Transform", "Transform OK");
  }
  else
  {
    std::string error;
    if (vis_manager_->getFrameManager()->transformHasProblems(frame, ros::Time(), error))
    {
      setStatus(status_levels::Error, "Transform", error);
    }
    else
    {
      std::stringstream ss;
      ss << "Could not transform from [" << frame << "] to [" << fixed_frame_ << "]";
      setStatus(status_levels::Error, "Transform", ss.str());
    }
  }
}

void GridDisplay::createProperties()
{
  frame_property_ = property_manager_->createProperty<TFFrameProperty>("Reference Frame", property_prefix_, boost::bind(&GridDisplay::getFrame, this),
                                                                       boost::bind(&GridDisplay::setFrame, this, _1), parent_category_, this);
  setPropertyHelpText(frame_property_, "The TF frame this grid will use for its origin.");

  cell_count_property_ = property_manager_->createProperty<IntProperty>( "Plane Cell Count", property_prefix_, boost::bind( &Grid::getCellCount, grid_),
                                                           boost::bind( &GridDisplay::setCellCount, this, _1 ), parent_category_, this );
  setPropertyHelpText(cell_count_property_, "The number of cells to draw in the plane of the grid.");
  IntPropertyPtr int_prop = cell_count_property_.lock();
  int_prop->setMin( 1 );
  int_prop->addLegacyName("Cell Count");

  height_property_ = property_manager_->createProperty<IntProperty>( "Normal Cell Count", property_prefix_, boost::bind( &Grid::getHeight, grid_),
                                                           boost::bind( &GridDisplay::setHeight, this, _1 ), parent_category_, this );
  setPropertyHelpText(height_property_, "The number of cells to draw along the normal vector of the grid.  Setting to anything but 0 makes the grid 3D.");
  int_prop = height_property_.lock();
  int_prop->setMin( 0 );

  cell_size_property_ = property_manager_->createProperty<FloatProperty>( "Cell Size", property_prefix_, boost::bind( &Grid::getCellLength, grid_ ),
                                                             boost::bind( &GridDisplay::setCellSize, this, _1 ), parent_category_, this );
  setPropertyHelpText(cell_size_property_, "The length, in meters, of the side of each cell.");
  FloatPropertyPtr float_prop = cell_size_property_.lock();
  float_prop->setMin( 0.0001 );

  style_property_ = property_manager_->createProperty<EnumProperty>( "Line Style", property_prefix_, boost::bind( &Grid::getStyle, grid_ ),
                                                                   boost::bind( &GridDisplay::setStyle, this, _1 ), parent_category_, this );
  setPropertyHelpText(style_property_, "The rendering operation to use to draw the grid lines.");
  EnumPropertyPtr enum_prop = style_property_.lock();
  enum_prop->addOption("Lines", Grid::Lines);
  enum_prop->addOption("Billboards", Grid::Billboards);

  line_width_property_ = property_manager_->createProperty<FloatProperty>( "Line Width", property_prefix_, boost::bind( &Grid::getLineWidth, grid_ ),
                                                               boost::bind( &GridDisplay::setLineWidth, this, _1 ), parent_category_, this );
  setPropertyHelpText(line_width_property_, "The width, in meters, of each grid line.");
  float_prop = line_width_property_.lock();
  float_prop->setMin( 0.001 );
  float_prop->hide();

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridDisplay::getColor, this ),
                                                                      boost::bind( &GridDisplay::setColor, this, _1 ), parent_category_, this );
  setPropertyHelpText(color_property_, "The color of the grid lines.");
  alpha_property_ = property_manager_->createProperty<FloatProperty>( "Alpha", property_prefix_, boost::bind( &GridDisplay::getAlpha, this ),
                                                                      boost::bind( &GridDisplay::setAlpha, this, _1 ), parent_category_, this );
  setPropertyHelpText(alpha_property_, "The amount of transparency to apply to the grid lines.");
  float_prop = alpha_property_.lock();
  float_prop->setMin( 0.0 );
  float_prop->setMax( 1.0f );

  plane_property_ = property_manager_->createProperty<EnumProperty>( "Plane", property_prefix_, boost::bind( &GridDisplay::getPlane, this ),
                                                                     boost::bind( &GridDisplay::setPlane, this, _1 ), parent_category_, this );
  setPropertyHelpText(plane_property_, "The plane to draw the grid along.");
  enum_prop = plane_property_.lock();
  enum_prop->addOption("XY", XY);
  enum_prop->addOption("XZ", XZ);
  enum_prop->addOption("YZ", YZ);

  offset_property_ = property_manager_->createProperty<Vector3Property>( "Offset", property_prefix_, boost::bind( &GridDisplay::getOffset, this ),
                                                                         boost::bind( &GridDisplay::setOffset, this, _1 ), parent_category_, this );
  setPropertyHelpText(offset_property_, "Allows you to offset the grid from the origin of the reference frame.  In meters.");
}

} // namespace rviz
