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

#include <stdio.h>

#include <QColor>
#include <QApplication>
#include <QFont>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>

#include "rviz/display_context.h"
#include "rviz/properties/yaml_helpers.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"

#include "display.h"

namespace rviz
{

Display::Display()
  : status_( 0 )
{
  // Make the display-enable checkbox show up, and make it unchecked by default.
  setValue( false );

  connect( this, SIGNAL( changed() ), this, SLOT( onEnableChanged() ));
}

void Display::initialize( DisplayContext* context )
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();
  update_nh_.setCallbackQueue( context_->getUpdateQueue() );
  threaded_nh_.setCallbackQueue( context_->getThreadedQueue() );

  onInitialize();
}

QVariant Display::getViewData( int column, int role ) const
{
  if( column == 0 )
  {
    switch( role )
    {
    case Qt::BackgroundRole:
    {
      QColor status_color = StatusProperty::statusColor( status_ ? status_->getLevel() : StatusProperty::Ok );
      return status_color.isValid() ? status_color : QColor( 4, 89, 127 );
    }
    case Qt::ForegroundRole: return QColor( Qt::white );
    case Qt::FontRole:
    {
      QFont font = QApplication::font( "PropertyTreeWidget" );
      font.setBold( true );
      return font;
    }
    }
  }
  return Property::getViewData( column, role );
}

Qt::ItemFlags Display::getViewFlags( int column ) const
{
  return Property::getViewFlags( column ) | Qt::ItemIsDragEnabled;
}

void Display::setStatus( StatusProperty::Level level, const QString& name, const QString& text )
{
  if( !status_ )
  {
    status_ = new StatusList( "Status" );
    addChild( status_, 0 );
  }
  StatusProperty::Level old_level = status_->getLevel();
  status_->setStatus( level, name, text );
  if( old_level != status_->getLevel() )
  {
    model_->emitDataChanged( this );
  }
}

void Display::clearStatuses()
{
  if( status_ )
  {
    StatusProperty::Level old_level = status_->getLevel();
    status_->clear();
    if( old_level != StatusProperty::Ok )
    {
      model_->emitDataChanged( this );
    }
  }
}

void Display::load( const YAML::Node& yaml_node )
{
  loadChildren( yaml_node );
}

void Display::loadChildren( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "Display::load() TODO: error handling - unexpected YAML type.\n" );
    return;
  }

  // Load the name and enabled state by hand, because they don't get
  // stored in sub-properties.
  if( const YAML::Node *name_node = yaml_node.FindValue( "Name" ))
  {
    QString name;
    *name_node >> name;
    setName( name );
  }

  if( const YAML::Node *enabled_node = yaml_node.FindValue( "Enabled" ))
  {
    bool enabled;
    *enabled_node >> enabled;
    setEnabled( enabled );
  }

  // Load all sub-properties the same way the base class does.
  Property::loadChildren( yaml_node );
}

void Display::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;
  saveChildren( emitter );
  emitter << YAML::EndMap;
}

void Display::saveChildren( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Class";
  emitter << YAML::Value << getClassId();

  emitter << YAML::Key << "Name";
  emitter << YAML::Value << getName();

  emitter << YAML::Key << "Enabled";
  emitter << YAML::Value << getEnabled();

  Property::saveChildren( emitter );
}

void Display::setEnabled( bool enabled )
{
  setValue( enabled );
}

bool Display::getEnabled() const
{
  return getValue().toBool();
}

void Display::setFixedFrame( const QString& fixed_frame )
{
  fixed_frame_ = fixed_frame;
  fixedFrameChanged();
}

void Display::reset()
{
  clearStatuses();
}

void Display::onEnableChanged()
{
  if( getEnabled() )
  {
    onEnable();
  }
  else
  {
    onDisable();
  }
}

} // end namespace rviz
