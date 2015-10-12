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

#include "rviz/display_context.h"
#include "rviz/display_factory.h"
#include "rviz/failed_display.h"
#include "rviz/properties/property_tree_model.h"

#include "display_group.h"

namespace rviz
{

DisplayGroup::DisplayGroup()
{
}

DisplayGroup::~DisplayGroup()
{
  removeAllDisplays();
}

Qt::ItemFlags DisplayGroup::getViewFlags( int column ) const
{
  return Display::getViewFlags( column ) | Qt::ItemIsDropEnabled;
}

void DisplayGroup::load( const Config& config )
{
  removeAllDisplays(); // Only remove Display children, property children must stay.

  // Load Property values, plus name and enabled/disabled.
  Display::load( config );

  // Now load Displays.
  Config display_list_config = config.mapGetChild( "Displays" );
  int num_displays = display_list_config.listLength();

  if( num_displays == 0 )
    return;

  if( model_ )
  {
    model_->beginInsert( this, Display::numChildren(), num_displays );
  }

  std::map<Display*,Config> display_config_map;

  // The following two-step loading procedure was motivated by the
  // 'display group visibility' property, which needs all other displays
  // to be created and named before it can load its settings.

  // hersh says: Is this really necessary?  Can't we make
  // DisplayGroupVisibilityProperty self-sufficient in this regard?
  // Also, does it really work?  What about saving and loading a
  // hierarchy of Displays, will they really all have the right
  // visibility settings?

  // first, create all displays and set their names
  for( int i = 0; i < num_displays; i++ )
  {
    Config display_config = display_list_config.listChildAt( i );
    QString display_class = "(no class name found)";
    display_config.mapGetString( "Class", &display_class );
    Display* disp = createDisplay( display_class );
    addDisplayWithoutSignallingModel( disp );
    QString display_name;
    display_config.mapGetString( "Name", &display_name );
    disp->setObjectName( display_name );

    display_config_map[ disp ] = display_config;
  }

  // now, initialize all displays and load their properties.
  for( std::map<Display*,Config>::iterator it = display_config_map.begin(); it != display_config_map.end(); ++it )
  {
    Config display_config = it->second;
    Display* disp = it->first;
    disp->initialize( context_ );
    disp->load( display_config );
  }

  if( model_ )
  {
    model_->endInsert();
  }
}

Display* DisplayGroup::createDisplay( const QString& class_id )
{
  DisplayFactory* factory = context_->getDisplayFactory();
  QString error;
  Display* disp = factory->make( class_id, &error );
  if( !disp )
  {
    return new FailedDisplay( class_id, error );
  }
  return disp;
}

void DisplayGroup::onEnableChanged()
{
  Display::onEnableChanged();
  for( int i = displays_.size() - 1; i >= 0; i-- )
  {
    displays_[ i ]->onEnableChanged();
  }
}

void DisplayGroup::save( Config config ) const
{
  Display::save( config );

  // Save Displays in a sequence under the key "Displays".
  Config display_list_config = config.mapMakeChild( "Displays" );

  int num_displays = displays_.size();
  for( int i = 0; i < num_displays; i++ )
  {
    displays_.at( i )->save( display_list_config.listAppendNew() );
  }
}

void DisplayGroup::removeAllDisplays()
{
  if(displays_.size() == 0)
    return;

  int num_non_display_children = Display::numChildren();

  if( model_ )
  {
    model_->beginRemove( this, num_non_display_children, displays_.size() );
  }
  for( int i = displays_.size() - 1; i >= 0; i-- )
  {
//    printf("  displaygroup2 displays_.takeAt( %d )\n", i );
    Display* child = displays_.takeAt( i );
    Q_EMIT displayRemoved( child );
    child->setParent( NULL ); // prevent child destructor from calling getParent()->takeChild().
    delete child;
  }
  if( model_ )
  {
    model_->endRemove();
  }
  Q_EMIT childListChanged( this );
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
//      printf("  displaygroup3 displays_.takeAt( %d )\n", i );
      result = displays_.takeAt( i );
      Q_EMIT displayRemoved( result );
      result->setParent( NULL );
      result->setModel( NULL );
      child_indexes_valid_ = false;
      if( model_ )
      {
        model_->endRemove();
      }
      Q_EMIT childListChanged( this );
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

DisplayGroup* DisplayGroup::getGroupAt( int index ) const
{
  return qobject_cast<DisplayGroup*>( getDisplayAt( index ));
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
    Display* display = displays_.at( i );
    if( display->isEnabled() )
    {
      display->update( wall_dt, ros_dt );
    }
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
//  printf("  displaygroup4 displays_.append( child )\n" );
  displays_.append( child );
  child_indexes_valid_ = false;
  child->setModel( model_ );
  child->setParent( this );
  Q_EMIT displayAdded( child );
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
  Q_EMIT childListChanged( this );
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
  Q_EMIT displayAdded( display );
  child_indexes_valid_ = false;
  display->setModel( model_ );
  display->setParent( this );

  if( model_ )
  {
    model_->endInsert();
  }
  Q_EMIT childListChanged( this );
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
//  printf("  displaygroup5 displays_.takeAt( %d ) ( index = %d )\n", disp_index, index );
  Display* child = displays_.takeAt( disp_index );
  Q_EMIT displayRemoved( child );
  child->setModel( NULL );
  child->setParent( NULL );
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
  }
  Q_EMIT childListChanged( this );
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
