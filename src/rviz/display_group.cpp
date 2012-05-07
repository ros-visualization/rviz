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

void DisplayGroup::load( const YAML::Node& yaml_node )
{
  clear();

  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "DisplayGroup::load() TODO: error handling - unexpected non-Map YAML type.\n" );
    return;
  }
  loadDisplays( yaml_node );
  Display::load( yaml_node );
}

void DisplayGroup::loadDisplays( const YAML::Node& yaml_node )
{
  const YAML::Node& displays_node = yaml_node[ "Displays" ];
  
  if( displays_node.Type() != YAML::NodeType::Sequence )
  {
    printf( "DisplayGroup::load() TODO: error handling - unexpected non-Sequence YAML type.\n" );
    return;
  }

  DisplayFactory* factory = context_->getDisplayFactory();

  if( model_ )
  {
    model_->beginInsert( this, children().size(), displays_node.size() );
  }

  for( YAML::Iterator it = displays_node.begin(); it != displays_node.end(); ++it )
  {
    const YAML::Node& display_node = *it;
    QString display_class;
    display_node[ "Class" ] >> display_class;
    Display* disp = factory->createDisplay( display_class );
    disp->setParentProperty( this );
    disp->initialize( context_ );
    disp->load( display_node );
  }

  if( model_ )
  {
    model_->endInsert();
  }
}

void DisplayGroup::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;
  saveCommonDisplayData( emitter );

  // Save non-display children
  int num_children = children().size();
  for( int i = 0; i < num_children; i++ )
  {
    Property* child = childAt( i );
    if( child && !qobject_cast<Display*>( child ))
    {
      emitter << YAML::Key << child->getName();
      emitter << YAML::Value;
      child->save( emitter );
    }
  }
  saveDisplays( emitter );
  emitter << YAML::EndMap;
}

void DisplayGroup::saveDisplays( YAML::Emitter& emitter )
{
  // Save Display children in a sequence under the key "Displays".
  emitter << YAML::Key << "Displays";
  emitter << YAML::Value;
  emitter << YAML::BeginSeq;
  int num_children = children().size();
  for( int i = 0; i < num_children; i++ )
  {
    Display* child = qobject_cast<Display*>( childAt( i ));
    if( child )
    {
      child->save( emitter );
    }
  }
  emitter << YAML::EndSeq;
}

void DisplayGroup::clear()
{
  int num_children = children().size();
  if( model_ )
  {
    model_->beginRemove( this, 0, num_children );
  }
  for( int i = num_children - 1; i >= 0; i-- )
  {
    delete children().at( i );
  }
  if( model_ )
  {
    model_->endRemove();
  }
}

Display* DisplayGroup::getDisplayAt( int index ) const
{
  if( 0 <= index && index < children().size() )
  {
    return dynamic_cast<Display*>( children().at( index ));
  }
  return NULL;
}

void DisplayGroup::fixedFrameChanged()
{
  int num_children = children().size();
  for( int i = 0; i < num_children; i++ )
  {
    Display* child = qobject_cast<Display*>( childAt( i ));
    if( child )
    {
      child->setFixedFrame( fixed_frame_ );
    }
  }  
}

void DisplayGroup::update( float wall_dt, float ros_dt )
{
  int num_children = children().size();
  for( int i = 0; i < num_children; i++ )
  {
    Display* child = qobject_cast<Display*>( childAt( i ));
    if( child )
    {
      child->update( wall_dt, ros_dt );
    }
  }  
}

void DisplayGroup::reset()
{
  Display::reset();

  int num_children = children().size();
  for( int i = 0; i < num_children; i++ )
  {
    Display* child = qobject_cast<Display*>( childAt( i ));
    if( child )
    {
      child->reset();
    }
  }  
}

} // end namespace rviz
