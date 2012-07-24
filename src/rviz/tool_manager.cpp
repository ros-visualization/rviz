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

#include <QKeyEvent>
#include <QRegExp>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>

#include <ros/assert.h>

#include "rviz/failed_tool.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/yaml_helpers.h"

#include "rviz/tool_manager.h"

namespace rviz
{

QString addSpaceToCamelCase( QString input )
{
  QRegExp re = QRegExp( "([A-Z])([a-z]*)" );
  input.replace( re, " \\1\\2" );
  return input.trimmed();
}

ToolManager::ToolManager( DisplayContext* context )
  : factory_( new PluginlibFactory<Tool>( "rviz", "rviz::Tool" ))
  , property_tree_model_( new PropertyTreeModel( new Property() ))
  , context_( context )
  , current_tool_( NULL )
{
}

ToolManager::~ToolManager()
{
  removeAll();
  delete factory_;
  delete property_tree_model_;
}

void ToolManager::initialize()
{
  // Possibly this should be done with a loop over
  // factory_->getDeclaredClassIds(), but then I couldn't control the
  // order.
  addTool( "rviz/MoveCamera" );
  addTool( "rviz/Interact" );
  addTool( "rviz/Select" );
  addTool( "rviz/SetInitialPose" );
  addTool( "rviz/SetGoal" );
}

void ToolManager::removeAll()
{
  while( !tools_.isEmpty() )
  {
    removeTool( 0 );
  }
}

void ToolManager::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Sequence )
  {
    printf( "ToolManager::load() TODO: error handling - unexpected YAML type (not a Sequence) at line %d, column %d.\n",
            yaml_node.GetMark().line, yaml_node.GetMark().column );
    return;
  }

  removeAll();

  for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
  {
    const YAML::Node& tool_node = *it;

    if( tool_node.Type() != YAML::NodeType::Map )
    {
      printf( "ToolManager::load() TODO: error handling - unexpected YAML type (not a Map) at line %d, column %d.\n",
              tool_node.GetMark().line, tool_node.GetMark().column );
      return;
    }

    QString class_id;
    tool_node[ "Class" ] >> class_id;
    Tool* tool = addTool( class_id );
    tool->load( tool_node );
  }
}

void ToolManager::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginSeq;
  for( int i = 0; i < tools_.size(); i++ )
  {
    Tool* tool = tools_[ i ];
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "Class" << YAML::Value << tool->getClassId();
    tool->save( emitter );
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;
}

void ToolManager::handleChar( QKeyEvent* event, RenderPanel* panel )
{
  if( event->key() == Qt::Key_Escape )
  {
    setCurrentTool( getDefaultTool() );
    return;
  }
  if( current_tool_ )
  {
    current_tool_->processKeyEvent( event, panel );
  }
}

void ToolManager::setCurrentTool( Tool* tool )
{
  if( current_tool_ )
  {
    current_tool_->deactivate();
  }
  current_tool_ = tool;

  if( current_tool_ )
  {
    current_tool_->activate();
  }

  Q_EMIT toolChanged( tool );
}

void ToolManager::setDefaultTool( Tool* tool )
{
  default_tool_ = tool;
}

Tool* ToolManager::getTool( int index )
{
  ROS_ASSERT( index >= 0 );
  ROS_ASSERT( index < (int)tools_.size() );

  return tools_[ index ];
}

Tool* ToolManager::addTool( const QString& class_id )
{
  QString error;
  bool failed = false;
  Tool* tool = factory_->make( class_id, &error );
  if( !tool )
  {
    tool = new FailedTool( class_id, error );
    failed = true;
  }
  else
  {
    tool->setIcon( factory_->getIcon( class_id ) );
    assert( !tool->getIcon().isNull() );
  }
  tools_.append( tool );
  tool->setName( addSpaceToCamelCase( factory_->getClassName( class_id )));
  tool->initialize( context_ );
  Property* container = tool->getPropertyContainer();
  if( container->numChildren() > 0 )
  {
    property_tree_model_->getRoot()->addChild( container );
    container->expand();
  }

  Q_EMIT toolAdded( tool );

  // If the default tool is unset and this tool loaded correctly, set
  // it as the default and current.
  if( default_tool_ == NULL && !failed )
  {
    setDefaultTool( tool );
    setCurrentTool( tool );
  }

  return tool;
}

void ToolManager::removeTool( int index )
{
  Tool* tool = tools_.takeAt( index );
  if( tool == current_tool_ )
  {
    current_tool_ = NULL;
  }
  if( tool == default_tool_ )
  {
    default_tool_ = NULL;
  }
  Q_EMIT toolRemoved( tool );
  delete tool;
}

QStringList ToolManager::getToolClasses()
{
  QStringList class_names;
  for( int i = 0; i < tools_.size(); i++ )
  {
    class_names.append( tools_[ i ]->getClassId() );
  }
  return class_names;
}

} // end namespace rviz
