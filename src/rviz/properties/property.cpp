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

#include <stdio.h> // for printf()
#include <limits.h> // for INT_MIN and INT_MAX

#include <QLineEdit>
#include <QSpinBox>

#include <yaml-cpp/node.h>
#include <yaml-cpp/emitter.h>

#include "rviz/properties/yaml_helpers.h"
#include "rviz/properties/float_edit.h"
#include "rviz/properties/property_tree_model.h"

#include "rviz/properties/property.h"

namespace rviz
{

class FailureProperty: public Property
{
public:
  virtual Property* subProp( const QString& sub_name ) { return this; }
};

/** @brief The property returned by subProp() when the requested
 * name is not found. */
Property* Property::failprop_ = new FailureProperty;

Property::Property( const QString& name,
                    const QVariant default_value,
                    const QString& description,
                    Property* parent,
                    const char *changed_slot,
                    QObject* receiver )
  : QObject( parent )
  , value_( default_value )
  , model_( 0 )
  , description_( description )
  , child_indexes_valid_( false )
{
  if( parent )
  {
    model_ = parent->getModel();
  }
  setName( name );
  if( receiver == 0 )
  {
    receiver = parent;
  }
  if( receiver && changed_slot )
  {
    connect( this, SIGNAL( changed() ), receiver, changed_slot );
  }
}

bool Property::setValue( const QVariant& new_value )
{
  if( new_value != value_ ) {
    Q_EMIT aboutToChange();
    value_ = new_value;
    Q_EMIT changed();
    if( model_ )
    {
      model_->emitDataChanged( this );
    }
    return true;
  }
  return false;
}

QVariant Property::getValue() const
{
  return value_;
}

void Property::setName( const QString& name )
{
  setObjectName( name );
  if( model_ )
  {
    model_->emitDataChanged( this );
  }
}

QString Property::getName() const
{
  return objectName();
}

void Property::setDescription( const QString& description )
{
  description_ = description;
}

QString Property::getDescription() const
{
  return description_;
}

Property* Property::subProp( const QString& sub_name )
{
  const QList<QObject*>& subs = children();
  int size = subs.size();
  for( int i = 0; i < size; i++ )
  {
    QObject* sub = subs.at( i );
    if( sub && sub->objectName() == sub_name )
    {
      Property* prop = dynamic_cast<Property*>( sub );
      if( prop )
      {
        return prop;
      }
    }
  }

  // Print a useful error message showing the whole ancestry of this
  // property, but don't crash.
  QString ancestry = "";
  for( QObject* obj = this; obj != NULL; obj = obj->parent() )
  {
    ancestry = "\"" + obj->objectName() + "\"->" + ancestry;
  }
  printf( "ERROR: Undefined property %s \"%s\" accessed.\n", qPrintable( ancestry ), qPrintable( sub_name ));
  return failprop_;
}

Property* Property::childAt( int index ) const
{
  if( 0 <= index && index < children().size() )
  {
    return dynamic_cast<Property*>( children().at( index ));
  }
  return NULL;
}

Property* Property::parentProperty() const
{
  return dynamic_cast<Property*>( parent() );
}

QVariant Property::getViewData( int column, int role ) const
{
  switch( column )
  {
  case 0: // left column: names
    switch( role )
    {
    case Qt::DisplayRole: return getName();
    default: return QVariant();
    }
    break;
  case 1: // right column: values
    switch( role )
    {
    case Qt::DisplayRole:
    case Qt::EditRole: return (value_.type() == QVariant::Bool ? QVariant() : getValue());
    case Qt::CheckStateRole:
      if( value_.type() == QVariant::Bool )
        return (value_.toBool() ? Qt::Checked : Qt::Unchecked);
      else
        return QVariant();
    default: return QVariant();
    }
    break;
  default: return QVariant();
  }
}

Qt::ItemFlags Property::getViewFlags( int column ) const
{
  if( column == 0 )
  {
    return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  }
  if( value_.isValid() )
  {
    if( value_.type() == QVariant::Bool )
    {
      return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
    }
    return Qt::ItemIsEditable | Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  }
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

bool Property::isAncestorOf( Property* possible_child ) const
{
  QObject* obj = possible_child->parent();
  while( obj != NULL && obj != this )
  {
    obj = obj->parent();
  }
  return obj == this;
}

void Property::addChildAt( Property* child, int index )
{
  int num_children = children().size();
  if( index < 0 )
  {
    index = 0;
  }
  if( index > num_children )
  {
    index = num_children;
  }
  int index_upon_arrival = num_children;

  child->setParentProperty( this );
  num_children++;

  moveChild( index_upon_arrival, index );
}

void Property::setParentProperty( Property* new_parent )
{
  setParent( new_parent );
  if( new_parent )
  {
    setModel( new_parent->getModel() );
  }
  else
  {
    setModel( 0 );
  }
}

void Property::setModel( PropertyTreeModel* model )
{
  model_ = model;
  for( int i = 0; i < children().size(); i++ )
  {
    Property* child = childAt( i );
    if( child )
    {
      child->setModel( model );
    }
  }
}

void Property::childEvent( QChildEvent* event )
{
  child_indexes_valid_ = false;
}

void Property::reindexChildren()
{
  const QList<QObject*>& childs = children();
  int num_children = childs.size();
  for( int index = 0; index < num_children; index++ )
  {
    Property* prop = dynamic_cast<Property*>( childs.at( index ));
    if( prop )
    {
      prop->row_number_within_parent_ = index;
    }
  }
  child_indexes_valid_ = true;
}

int Property::rowNumberInParent() const
{
  Property* parent = parentProperty();
  if( !parent )
  {
    return -1;
  }
  if( !parent->child_indexes_valid_ )
  {
    parent->reindexChildren();
  }
  return row_number_within_parent_;
}

void Property::moveChild( int from_index, int to_index )
{
  QList<QObject*>& non_const_children = const_cast<QList<QObject*>&>( children() );
  non_const_children.move( from_index, to_index );
  child_indexes_valid_ = false;
}

void Property::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() == YAML::NodeType::Scalar )
  {
    switch( value_.type() )
    {
    case QVariant::Int:
    {
      int new_value;
      yaml_node >> new_value;
      setValue( new_value );
    }
    break;
    case QMetaType::Float:
    case QVariant::Double:
    {
      double new_value;
      yaml_node >> new_value;
      setValue( new_value );
    }
    break;
    case QVariant::String:
    {
      std::string new_value;
      yaml_node >> new_value;
      setValue( QString::fromStdString( new_value ));
    }
    break;
    default:
      printf( "Property::load() TODO: error handling - unexpected QVariant type.\n" );
      break;
    }
  }
  else
  {
    printf( "Property::load() TODO: error handling - unexpected YAML type.\n" );
  }
}

void Property::save( YAML::Emitter& emitter )
{
  int num_children = children().size();
  // If there are child properties, save them in a map from names to children.
  if( num_children > 0 )
  {
    emitter << YAML::BeginMap;
    for( int i = 0; i < num_children; i++ )
    {
      Property* child = childAt( i );
      if( child )
      {
        emitter << YAML::Key << child->getName();
        emitter << YAML::Value;
        child->save( emitter );
      }
    }
    emitter << YAML::EndMap;
  }
  else // Else there are no child properties, so just save the value itself.
  {
    switch( value_.type() )
    {
    case QVariant::Int:    emitter << getValue().toInt(); break;
    case QMetaType::Float:
    case QVariant::Double: emitter << getValue().toDouble(); break;
    case QVariant::String: emitter << getValue().toString().toStdString(); break;
    default:
      printf( "Property::save() TODO: error handling - unexpected QVariant type.\n" );
    }
  }
}

QWidget* Property::createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option,
                                 const QModelIndex& index )
{
  switch( value_.type() )
  {
  case QVariant::Int:
  {
    QSpinBox* editor = new QSpinBox( parent );
    editor->setFrame( false );
    editor->setRange( INT_MIN, INT_MAX );
    return editor;
  }
  case QMetaType::Float:
  case QVariant::Double:
  {
    FloatEdit* editor = new FloatEdit( parent );
    return editor;
  }
  case QVariant::String:
  default:
  {
    QLineEdit* editor = new QLineEdit( parent );
    editor->setFrame( false );
    return editor;
  }
  }
}

} // end namespace rviz
