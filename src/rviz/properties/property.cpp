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
  : value_( default_value )
  , model_( 0 )
  , child_indexes_valid_( false )
  , parent_( 0 )
  , description_( description )
  , hidden_( false )
  , is_read_only_( false )
  , save_( true )
{
  setName( name );
  if( parent )
  {
    parent->addChild( this );
  }
  if( receiver == 0 )
  {
    receiver = parent;
  }
  if( receiver && changed_slot )
  {
    connect( this, SIGNAL( changed() ), receiver, changed_slot );
  }
}

Property::~Property()
{
  // Disconnect myself from my parent.
  if( getParent() )
  {
    getParent()->takeChild( this );
  }
  // Destroy my children.
  for( int i = children_.size() - 1; i >= 0; i-- )
  {
//    printf("  property1 children_.takeAt( %d )\n", i );
    Property* child = children_.takeAt( i );
    child->setParent( NULL );
    delete child;
  }
}

void Property::removeChildren( int start_index, int count )
{
  if( count < 0 )
  {
    count = children_.size() - start_index;
  }

  if( model_ )
  {
    model_->beginRemove( this, start_index, count );
  }
  // Destroy my children.
  for( int i = start_index; i < start_index + count; i++ )
  {
    Property* child = children_.at( i );
    child->setParent( NULL ); // prevent child destructor from calling getParent()->takeChild().
    delete child;
  }
//  printf("  property2 children_.clear()\n" );
  children_.erase( children_.begin() + start_index, children_.begin() + start_index + count );
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
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
  int size = numChildren();
  for( int i = 0; i < size; i++ )
  {
    Property* prop = childAtUnchecked( i );
    if( prop->getName() == sub_name )
    {
      return prop;
    }
  }

  // Print a useful error message showing the whole ancestry of this
  // property, but don't crash.
  QString ancestry = "";
  for( Property* prop = this; prop != NULL; prop = prop->getParent() )
  {
    ancestry = "\"" + prop->getName() + "\"->" + ancestry;
  }
  printf( "ERROR: Undefined property %s \"%s\" accessed.\n", qPrintable( ancestry ), qPrintable( sub_name ));
  return failprop_;
}

Property* Property::childAt( int index ) const
{
  // numChildren() and childAtUnchecked() can both be overridden, so
  // call them instead of accessing our children_ list directly.
  if( 0 <= index && index < numChildren() )
  {
    return childAtUnchecked( index );
  }
  return NULL;
}

Property* Property::childAtUnchecked( int index ) const
{
  return children_.at( index );
}

Property* Property::getParent() const
{
  return parent_;
}

void Property::setParent( Property* new_parent )
{
  parent_ = new_parent;
}

QVariant Property::getViewData( int column, int role ) const
{
  switch( column )
  {
  case 0: // left column: names
    switch( role )
    {
    case Qt::DisplayRole: return getName();
    case Qt::DecorationRole: return icon_;
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
  // if the parent propery is a disabled bool property or
  // has its own enabled view flag not set, disable this property as well
  Qt::ItemFlags enabled_flag = Qt::ItemIsEnabled;
  if ( parent_ )
  {
    if( parent_->getValue().type() == QVariant::Bool && !parent_->getValue().toBool() )
    {
      enabled_flag = 0;
    }
    enabled_flag &= parent_->getViewFlags( 0 );
  }

  if( column == 0 || is_read_only_ )
  {
    return enabled_flag | Qt::ItemIsSelectable;
  }
  if( value_.isValid() )
  {
    if( value_.type() == QVariant::Bool )
    {
      return Qt::ItemIsUserCheckable | enabled_flag | Qt::ItemIsSelectable;
    }
    return Qt::ItemIsEditable | enabled_flag | Qt::ItemIsSelectable;
  }
  return enabled_flag | Qt::ItemIsSelectable;
}

bool Property::isAncestorOf( Property* possible_child ) const
{
  Property* prop = possible_child->getParent();
  while( prop != NULL && prop != this )
  {
    prop = prop->getParent();
  }
  return prop == this;
}

Property* Property::takeChild( Property* child )
{
  for( int i = 0; i < numChildren(); i++ )
  {
    if( childAtUnchecked( i ) == child )
    {
      return takeChildAt( i );
    }
  }
  return NULL;
}

Property* Property::takeChildAt( int index )
{
  if( index < 0 || index >= children_.size() )
  {
    return NULL;
  }
  if( model_ )
  {
    model_->beginRemove( this, index, 1 );
  }
//  printf("  property3 children_.takeAt( %d )\n", index );
  Property* child = children_.takeAt( index );
  child->setModel( NULL );
  child->parent_ = NULL;
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
  }
  return child;
}

void Property::addChild( Property* child, int index )
{
  if( !child )
  {
    return;
  }
  int num_children = children_.size();
  if( index < 0 || index > num_children )
  {
    index = num_children;
  }
  if( model_ )
  {
    model_->beginInsert( this, index );
  }

//  printf("  property4 children_.insert( %d, child )\n", index );
  children_.insert( index, child );
  child_indexes_valid_ = false;
  child->setModel( model_ );
  child->parent_ = this;

  if( model_ )
  {
    model_->endInsert();
  }
}

void Property::setModel( PropertyTreeModel* model )
{
  model_ = model;
  if( model_ && hidden_ )
  {
    model_->emitPropertyHiddenChanged( this );
  }
  int num_children = numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    Property* child = childAtUnchecked( i );
    child->setModel( model );
  }
}

void Property::reindexChildren()
{
  int num_children = numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    Property* child = childAtUnchecked( i );
    child->row_number_within_parent_ = i;
  }
  child_indexes_valid_ = true;
}

int Property::rowNumberInParent() const
{
  Property* parent = getParent();
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
//  printf("  property5 children_.move( %d, %d )\n", from_index, to_index );
  children_.move( from_index, to_index );
  child_indexes_valid_ = false;
}

void Property::load( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() == YAML::NodeType::Scalar )
  {
    loadValue( yaml_node );
  }
  else if( yaml_node.Type() == YAML::NodeType::Map )
  {
    loadChildren( yaml_node );
  }
  else
  {
    printf( "Property::load() TODO: error handling - unexpected YAML type (Sequence) at line %d, column %d.\n",
            yaml_node.GetMark().line, yaml_node.GetMark().column );
  }
}

void Property::loadValue( const YAML::Node& yaml_node )
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
  case QVariant::Bool:
  {
    bool new_value;
    yaml_node >> new_value;
    setValue( new_value );
  }
  break;
  default:
    printf( "Property::load() TODO: error handling - unexpected QVariant type.\n" );
    break;
  }
}

void Property::loadChildren( const YAML::Node& yaml_node )
{
  if( yaml_node.Type() != YAML::NodeType::Map )
  {
    printf( "Property::loadChildren() TODO: error handling - unexpected YAML type.\n" );
    return;
  }

  // A special map entry named "Value" means the value of this property, not a child.
  if( const YAML::Node *value_node = yaml_node.FindValue( "Value" ))
  {
    loadValue( *value_node );
  }

  // Yaml-cpp's FindValue() and operator[] functions are order-N,
  // according to the docs, so we don't want to use those.  Instead we
  // make a hash table of the existing property children, then loop
  // over all the yaml key-value pairs, looking up their targets by
  // key (name) in the map.  This should keep this function down to
  // order-N or close, instead of order N squared.

  // First make the hash table of all child properties indexed by name.
  QHash<QString, Property*> child_map;
  int num_property_children = children_.size();
  for( int i = 0; i < num_property_children; i++ )
  {
    Property* child = children_.at( i );
    child_map[ child->getName() ] = child;
  }

  // Next loop over all yaml key/value pairs, calling load() on each
  // child whose name we find.
  for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
  {
    QString key;
    it.first() >> key;
    QHash<QString, Property*>::const_iterator hash_iter = child_map.find( key );
    if( hash_iter != child_map.end() )
    {
      Property* child = hash_iter.value();
      child->load( it.second() );
    }
    else
    {
      std::string key;
      it.first() >> key;
      std::cout << "Could not load property. Key: " << key << std::endl;
    }
  }
}

void Property::save( YAML::Emitter& emitter )
{
  // If there are child properties, save them in a map from names to children.
  if( children_.size() > 0 )
  {
    emitter << YAML::BeginMap;

    // If this property has child properties *and* a value itself,
    // save the value in a special map entry named "Value".
    if( value_.isValid() )
    {
      emitter << YAML::Key << "Value";
      emitter << YAML::Value;
      saveValue( emitter );
    }
    saveChildren( emitter );
    emitter << YAML::EndMap;
  }
  else // Else there are no child properties, so just save the value itself.
  {
    if( value_.isValid() )
    {
      saveValue( emitter );
    }
    else
    {
      emitter << YAML::BeginMap << YAML::EndMap;
    }
  }
}

void Property::saveValue( YAML::Emitter& emitter )
{
  switch( value_.type() )
  {
  case QVariant::Int:     emitter << getValue().toInt(); break;
  case QMetaType::Float:
  case QVariant::Double:  emitter << getValue().toDouble(); break;
  case QVariant::String:  emitter << getValue().toString(); break;
  case QVariant::Bool:    emitter << getValue().toBool(); break;
  default:
    printf( "Property::save() TODO: error handling - unexpected QVariant type %s.\n", getValue().typeName() );
    emitter << ( QString( "Unexpected QVariant type " ) + getValue().typeName() );
  }
}

void Property::saveChildren( YAML::Emitter& emitter )
{
  int num_properties = children_.size();
  for( int i = 0; i < num_properties; i++ )
  {
    Property* prop = children_.at( i );
    if( prop && prop->shouldBeSaved() )
    {
      emitter << YAML::Key << prop->getName();
      emitter << YAML::Value;
      prop->save( emitter );
    }
  }
}

QWidget* Property::createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option )
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

void Property::setHidden( bool hidden )
{
  if( hidden != hidden_ )
  {
    hidden_ = hidden;
    if( model_ )
    {
      model_->emitPropertyHiddenChanged( this );
    }
  }
}

void Property::expand()
{
  if( model_ )
  {
    model_->expandProperty( this );
  }
}

void Property::collapse()
{
  if( model_ )
  {
    model_->collapseProperty( this );
  }
}

} // end namespace rviz
