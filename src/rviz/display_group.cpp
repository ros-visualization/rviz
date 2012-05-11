/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <stdio.h> // for debug-write printf

#include <QColor>

#include <yaml-cpp/emitter.h>

#include "rviz/display_context.h"
#include "rviz/display_factory.h"
#include "rviz/failed_display.h"
#include "rviz/properties/yaml_helpers.h"
#include "rviz/properties/property_tree_model.h"

#include "display_group.h"

namespace rviz
{

DisplayGroup::DisplayGroup()
{
}

DisplayGroup::~DisplayGroup()
{
  for( int i = displays_.size() - 1; i >= 0; i-- )
  {
    delete displays_.takeAt( i );
  }
}

QVariant DisplayGroup::getViewData( int column, int role ) const
{
  if( column == 0 )
  {
    switch( role )
    {
    case Qt::BackgroundRole: return QColor( 40, 120, 197 );
    case Qt::ForegroundRole: return QColor( Qt::white );
    }
  }
  return Display::getViewData( column, role );
}

Qt::ItemFlags DisplayGroup::getViewFlags( int column ) const
{
  return Display::getViewFlags( column ) | Qt::ItemIsDropEnabled;
}

void DisplayGroup::loadChildren( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "DisplayGroup::load() TODO: error handling - unexpected non-Map YAML type.\n" );
    return;
  }
  removeAllDisplays(); // Only remove Display children, property children must stay.

  // Load Property values, plus name and enabled/disabled.
  Display::loadChildren( yaml_node );

  // Now load Displays.
  const YAML::Node& displays_node = yaml_node[ "Displays" ];
  if( displays_node.Type() != YAML::NodeType::Sequence )
  {
    printf( "DisplayGroup::load() TODO: error handling - unexpected non-Sequence YAML type.\n" );
    return;
  }

  if( model_ )
  {
    model_->beginInsert( this, Display::numChildren(), displays_node.size() );
  }

  for( YAML::Iterator it = displays_node.begin(); it != displays_node.end(); ++it )
  {
    const YAML::Node& display_node = *it;
    QString display_class;
    display_node[ "Class" ] >> display_class;
    Display* disp = createDisplay( display_class );
    addDisplayWithoutSignallingModel( disp );
    disp->initialize( context_ );
    disp->load( display_node );
  }

  if( model_ )
  {
    model_->endInsert();
  }
}

Display* DisplayGroup::createDisplay( const QString& class_id )
{
  DisplayFactory* factory = context_->getDisplayFactory();
  Display* disp = factory->make( class_id );
  if( !disp )
  {
    disp = new FailedDisplay();
    disp->setClassId( class_id );
  }
  return disp;
}

void DisplayGroup::saveChildren( YAML::Emitter& emitter )
{
  Display::saveChildren( emitter );

  // Save Displays in a sequence under the key "Displays".
  emitter << YAML::Key << "Displays";
  emitter << YAML::Value;
  emitter << YAML::BeginSeq;
  int num_displays = displays_.size();
  for( int i = 0; i < num_displays; i++ )
  {
    displays_.at( i )->save( emitter );
  }
  emitter << YAML::EndSeq;
}

void DisplayGroup::removeAllDisplays()
{
  int num_non_display_children = Display::numChildren();

  if( model_ )
  {
    model_->beginRemove( this, num_non_display_children, displays_.size() );
  }
  for( int i = displays_.size() - 1; i >= 0; i-- )
  {
    delete displays_.takeAt( i );
  }
  if( model_ )
  {
    model_->endRemove();
  }
}

Display* DisplayGroup::takeDisplay( Display* child )
{
  Display* result = NULL;
  int num_displays = displays_.size();
  for( int i = 0; i < num_displays; i++ )
  {
    if( displays_.at( i ) == child )
    {
      if( model_ )
      {
        model_->beginRemove( this, Display::numChildren() + i, 1 );
      }
      result = displays_.takeAt( i );
      result->setParent( NULL );
      result->setModel( NULL );
      child_indexes_valid_ = false;
      if( model_ )
      {
        model_->endRemove();
      }
      break;
    }
  }
  return result;
}

Display* DisplayGroup::getDisplayAt( int index ) const
{
  if( 0 <= index && index < displays_.size() )
  {
    return displays_.at( index );
  }
  return NULL;
}

void DisplayGroup::fixedFrameChanged()
{
  int num_children = displays_.size();
  for( int i = 0; i < num_children; i++ )
  {
    displays_.at( i )->setFixedFrame( fixed_frame_ );
  }  
}

void DisplayGroup::update( float wall_dt, float ros_dt )
{
  int num_children = displays_.size();
  for( int i = 0; i < num_children; i++ )
  {
    displays_.at( i )->update( wall_dt, ros_dt );
  }  
}

void DisplayGroup::reset()
{
  Display::reset();

  int num_children = displays_.size();
  for( int i = 0; i < num_children; i++ )
  {
    displays_.at( i )->reset();
  }  
}

void DisplayGroup::addDisplayWithoutSignallingModel( Display* child )
{
  displays_.append( child );
  child_indexes_valid_ = false;
  child->setModel( model_ );
  child->setParent( this );
}

void DisplayGroup::addDisplay( Display* child )
{
  if( model_ )
  {
    model_->beginInsert( this, numChildren(), 1 );
  }
  addDisplayWithoutSignallingModel( child );
  if( model_ )
  {
    model_->endInsert();
  }
}

void DisplayGroup::addChild( Property* child, int index )
{
  Display* display = qobject_cast<Display*>( child );
  if( !display )
  {
    Display::addChild( child, index );
    return;
  }
  if( index < 0 || index > numChildren() )
  {
    index = numChildren();
  }
  int disp_index = index - Display::numChildren();
  if( disp_index < 0 )
  {
    disp_index = 0;
  }
  if( model_ )
  {
    model_->beginInsert( this, index );
  }

  displays_.insert( disp_index, display );
  child_indexes_valid_ = false;
  display->setModel( model_ );
  display->setParent( this );

  if( model_ )
  {
    model_->endInsert();
  }
}

Property* DisplayGroup::takeChildAt( int index )
{
  if( index < Display::numChildren() )
  {
    return Display::takeChildAt( index );
  }
  int disp_index = index - Display::numChildren();
  if( model_ )
  {
    model_->beginRemove( this, index, 1 );
  }
  Property* child = displays_.takeAt( disp_index );
  child->setModel( NULL );
  child->setParent( NULL );
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
  }
  return child;
}

int DisplayGroup::numDisplays() const
{
  return displays_.size();
}

int DisplayGroup::numChildren() const
{
  return Display::numChildren() + displays_.size();
}

Property* DisplayGroup::childAtUnchecked( int index ) const
{
  int first_child_count = Display::numChildren();
  if( index < first_child_count )
  {
    return Display::childAtUnchecked( index );
  }
  index -= first_child_count;
  return displays_.at( index );
}

} // end namespace rviz
