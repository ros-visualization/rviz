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
  , target_scene_node_( NULL )
  , is_active_( false )
  , type_property_( NULL )
{}

void ViewController::initialize( DisplayContext* context, Ogre::SceneNode* target_scene_node )
{
  context_ = context;
  target_scene_node_ = target_scene_node;

  std::stringstream ss;
  static int count = 0;
  ss << "ViewControllerCamera" << count++;
  camera_ = context_->getSceneManager()->createCamera( ss.str() );
  camera_->setNearClipDistance(0.01f);
  target_scene_node_->attachObject( camera_ );

  setValue( getClassId() );
  setReadOnly( true );

  // Do subclass initialization.
  onInitialize();
}

ViewController::~ViewController()
{
  context_->getSceneManager()->destroyCamera( camera_ );
}

void ViewController::addTypeSelector( const QStringList& class_ids )
{
  type_property_ = new EnumProperty( "Type", getClassId(), "Type of view controller", NULL, SLOT( updateType() ), this );
  for( int i = 0; i < class_ids.size(); i++ )
  {
    type_property_->addOption( class_ids[ i ]);
  }
  type_property_->setShouldBeSaved( false );
  addChild( type_property_, 0 );
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

void ViewController::activate( const std::string& reference_frame )
{
  is_active_ = true;
  reference_frame_ = reference_frame;
  updateTargetSceneNode();

  onActivate();
}

void ViewController::update(float dt, float ros_dt)
{
  updateTargetSceneNode();
  onUpdate(dt, ros_dt);
}

void ViewController::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void ViewController::setTargetFrame(const std::string& reference_frame)
{
  Ogre::Vector3 old_position;
  Ogre::Quaternion old_orientation;
  context_->getFrameManager()->getTransform(reference_frame_, ros::Time(), old_position, old_orientation);

  reference_frame_ = reference_frame;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  context_->getFrameManager()->getTransform(reference_frame_, ros::Time(), position, orientation);

  reference_position_ = position;
  reference_orientation_ = orientation;

  onTargetFrameChanged( old_position, old_orientation );
}

void ViewController::updateTargetSceneNode()
{
  Ogre::Vector3 new_reference_position;
  Ogre::Quaternion new_reference_orientation;

  if (context_->getFrameManager()->getTransform(reference_frame_, ros::Time(), new_reference_position, new_reference_orientation) )
  {
    target_scene_node_->setPosition( new_reference_position );

    reference_position_ = new_reference_position;
    reference_orientation_ = new_reference_orientation;

    context_->queueRender();
  }
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
  emitter << YAML::Key << "Name";
  emitter << YAML::Value << getName();

  Property::saveChildren( emitter );
}


} // end namespace rviz
