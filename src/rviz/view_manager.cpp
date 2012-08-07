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

#include <sstream>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/parser.h>

#include "rviz/display_context.h"
#include "rviz/failed_view_controller.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/yaml_helpers.h"
#include "rviz/render_panel.h"
#include "rviz/view_controller.h"

#include "rviz/view_manager.h"

namespace rviz
{

ViewManager::ViewManager( DisplayContext* context )
  : context_( context )
  , root_property_( new ViewControllerContainer )
  , property_model_( new PropertyTreeModel( root_property_ ))
  , factory_( new PluginlibFactory<ViewController>( "rviz", "rviz::ViewController" ))
  , current_( NULL )
{
  property_model_->setDragDropClass( "view-controller" );
  connect( property_model_, SIGNAL( configChanged() ), this, SIGNAL( configChanged() ));
}

ViewManager::~ViewManager()
{
  delete property_model_;
  delete factory_;
}

void ViewManager::initialize()
{
  setCurrent( create( "rviz/Orbit" ), false );
}

void ViewManager::update( float wall_dt, float ros_dt )
{
  if( getCurrent() )
  {
    getCurrent()->update( wall_dt, ros_dt );
  }
}

ViewController* ViewManager::create( const QString& class_id )
{
  QString error;
  bool failed = false;
  ViewController* view = factory_->make( class_id, &error );
  if( !view )
  {
    view = new FailedViewController( class_id, error );
    failed = true;
  }
  view->initialize( context_ );

  return view;
}

ViewController* ViewManager::getCurrent() const
{
  return current_;
}

void ViewManager::setCurrentFrom( ViewController* source_view )
{
  if( source_view == NULL )
  {
    return;
  }

  ViewController* previous = getCurrent();
  if( source_view != previous )
  {
    ViewController* new_current = copy( source_view );

    setCurrent( new_current, false );
    Q_EMIT configChanged();
  }
}

void ViewManager::onCurrentDestroyed( QObject* obj )
{
  if( obj == current_ )
  {
    current_ = NULL;
  }
}

void ViewManager::setCurrent( ViewController* new_current, bool mimic_view )
{
  ViewController* previous = getCurrent();
  if( previous )
  {
    if( mimic_view )
    {
      new_current->mimic( previous );
    }
    else
    {
      new_current->transitionFrom( previous );
    }
    disconnect( previous, SIGNAL( destroyed( QObject* )), this, SLOT( onCurrentDestroyed( QObject* )));
  }
  new_current->setName( "Current View" );
  connect( new_current, SIGNAL( destroyed( QObject* )), this, SLOT( onCurrentDestroyed( QObject* )));
  current_ = new_current;
  root_property_->addChildToFront( new_current );
  delete previous;

  // This setViewController() can indirectly call
  // ViewManager::update(), so make sure getCurrent() will return the
  // new one by this point.
  context_->getRenderPanel()->setViewController( new_current );

  Q_EMIT currentChanged();
}

void ViewManager::setCurrentViewControllerType( const QString& new_class_id )
{
  setCurrent( create( new_class_id ), true );
}

void ViewManager::copyCurrentToList()
{
  ViewController* current = getCurrent();
  if( current )
  {
    ViewController* new_copy = copy( current );
    new_copy->setName( factory_->getClassName( new_copy->getClassId() ));
    root_property_->addChild( new_copy );
  }
}

ViewController* ViewManager::getViewAt( int index ) const
{
  if( index < 0 )
  {
    index = 0;
  }
  return qobject_cast<ViewController*>( root_property_->childAt( index + 1 ));
}

int ViewManager::getNumViews() const
{
  int count = root_property_->numChildren();
  if( count <= 0 )
  {
    return 0;
  }
  else
  {
    return count-1;
  }
}

void ViewManager::add( ViewController* view, int index )
{
  if( index < 0 )
  {
    index = root_property_->numChildren();
  }
  else
  {
    index++;
  }
  property_model_->getRoot()->addChild( view, index );
}

ViewController* ViewManager::take( ViewController* view )
{
  for( int i = 0; i < getNumViews(); i++ )
  {
    if( getViewAt( i ) == view )
    {
      return qobject_cast<ViewController*>( root_property_->takeChildAt( i + 1 ));
    }
  }
  return NULL;
}

ViewController* ViewManager::takeAt( int index )
{
  if( index < 0 )
  {
    return NULL;
  }
  return qobject_cast<ViewController*>( root_property_->takeChildAt( index + 1 ));
}

void ViewManager::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "ViewManager::load()1 TODO: error handling - unexpected YAML type (not a Map) at line %d, column %d.\n",
            yaml_node.GetMark().line, yaml_node.GetMark().column );
    return;
  }

  if( const YAML::Node *current_node = yaml_node.FindValue( "Current" ))
  {
    if( current_node->Type() != YAML::NodeType::Map )
    {
      printf( "ViewManager::load()2 TODO: error handling - unexpected YAML type (not a Map) at line %d, column %d.\n",
              current_node->GetMark().line, current_node->GetMark().column );
      return;
    }
    QString class_id;
    (*current_node)[ "Class" ] >> class_id;

    ViewController* new_current = create( class_id );
    new_current->load( *current_node );
    setCurrent( new_current, false );
  }

  if( const YAML::Node *saved_node = yaml_node.FindValue( "Saved" ))
  {
    if( saved_node->Type() != YAML::NodeType::Sequence )
    {
      printf( "ViewManager::load() TODO: error handling - unexpected YAML type (not a Sequence) at line %d, column %d.\n",
              saved_node->GetMark().line, saved_node->GetMark().column );
      return;
    }

    root_property_->removeChildren( 1 );

    for( YAML::Iterator it = saved_node->begin(); it != saved_node->end(); ++it )
    {
      const YAML::Node& view_node = *it;

      if( view_node.Type() != YAML::NodeType::Map )
      {
        printf( "ViewManager::load()3 TODO: error handling - unexpected YAML type (not a Map) at line %d, column %d.\n",
                view_node.GetMark().line, view_node.GetMark().column );
        return;
      }

      QString class_id;
      view_node[ "Class" ] >> class_id;
      ViewController* view = create( class_id );
      view->load( view_node );
      add( view );
    }
  }
}

void ViewManager::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "Current";
  emitter << YAML::Value;
  {
    getCurrent()->save( emitter );
  }

  emitter << YAML::Key << "Saved";
  emitter << YAML::Value;
  {
    emitter << YAML::BeginSeq;
    for( int i = 0; i < getNumViews(); i++ )
    {
      getViewAt( i )->save( emitter );
    }
    emitter << YAML::EndSeq;
  }

  emitter << YAML::EndMap;
}

ViewController* ViewManager::copy( ViewController* source )
{
  ViewController* copy_of_source = create( source->getClassId() );

  YAML::Emitter emitter;
  source->save( emitter );

  std::string yaml_string( emitter.c_str() );
  std::stringstream ss( yaml_string ); // make a stream with the output doc.
  YAML::Parser parser( ss );
  YAML::Node yaml_node;
  if( parser.GetNextDocument( yaml_node ))
  {
    copy_of_source->load( yaml_node );
  }
  else
  {
    ROS_ERROR( "ViewManager::copy() failed to get a valid YAML document from source ViewController (type %s).",
               qPrintable( source->getClassId() ));
  }
  return copy_of_source;
}

Qt::ItemFlags ViewControllerContainer::getViewFlags( int column ) const
{
  return Property::getViewFlags( column ) | Qt::ItemIsDropEnabled;
}

void ViewControllerContainer::addChild( Property* child, int index )
{
  if( index == 0 )
  {
    index = 1;
  }
  Property::addChild( child, index );
}

void ViewControllerContainer::addChildToFront( Property* child )
{
  Property::addChild( child, 0 );
}

} // end namespace rviz
