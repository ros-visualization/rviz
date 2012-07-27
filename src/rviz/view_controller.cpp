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

#include <QApplication>
#include <QColor>
#include <QFont>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/yaml_helpers.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/view_manager.h"

#include "rviz/view_controller.h"

namespace rviz
{

ViewController::ViewController()
  : context_( NULL )
  , camera_( NULL )
  , is_active_( false )
  , type_property_( NULL )
{}

void ViewController::initialize( DisplayContext* context )
{
  context_ = context;

  std::stringstream ss;
  static int count = 0;
  ss << "ViewControllerCamera" << count++;
  camera_ = context_->getSceneManager()->createCamera( ss.str() );
  camera_->setNearClipDistance(0.01f);
  context_->getSceneManager()->getRootSceneNode()->attachObject( camera_ );

  setValue( formatClassId( getClassId() ));
  setReadOnly( true );

  // Do subclass initialization.
  onInitialize();
}

ViewController::~ViewController()
{
  context_->getSceneManager()->destroyCamera( camera_ );
}

QString ViewController::formatClassId( const QString& class_id )
{
  QStringList id_parts = class_id.split( "/" );
  if( id_parts.size() != 2 )
  {
    // Should never happen with pluginlib class ids, which are
    // formatted like "package_name/class_name".  Not worth crashing
    // over though.
    return class_id;
  }
  else
  {
    return id_parts[ 1 ] + " (" + id_parts[ 0 ] + ")";
  }
}

QVariant ViewController::getViewData( int column, int role ) const
{
  if( is_active_ )
  {
    switch( role )
    {
    case Qt::BackgroundRole:
    {
      return QColor( 0xba, 0xad, 0xa4 );
    }
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

Qt::ItemFlags ViewController::getViewFlags( int column ) const
{
  if( is_active_ )
  {
    return Property::getViewFlags( column );
  }
  else
  {
    return Property::getViewFlags( column ) | Qt::ItemIsDragEnabled;
  }
}

void ViewController::activate()
{
  is_active_ = true;
  onActivate();
}

void ViewController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void ViewController::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "ViewController::load() TODO: error handling - unexpected non-map YAML type at line %d column %d.\n",
            yaml_node.GetMark().line, yaml_node.GetMark().column );
    return;
  }

  // Load the name by hand.
  if( const YAML::Node *name_node = yaml_node.FindValue( "Name" ))
  {
    QString name;
    *name_node >> name;
    setName( name );
  }
  // Load all sub-properties the same way the base class does.
  Property::loadChildren( yaml_node );
}

void ViewController::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;
  saveChildren( emitter );
  emitter << YAML::EndMap;
}

void ViewController::saveChildren( YAML::Emitter& emitter )
{
  emitter << YAML::Key << "Class";
  emitter << YAML::Value << getClassId();

  emitter << YAML::Key << "Name";
  emitter << YAML::Value << getName();

  Property::saveChildren( emitter );
}

} // end namespace rviz
