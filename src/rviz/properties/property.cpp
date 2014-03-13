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
  children_.erase( children_.begin() + start_index, children_.begin() + start_index + count );
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
  }
  Q_EMIT childListChanged( this );
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

bool Property::contains( Property* possible_child ) const
{
  int num_children = numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    if( childAtUnchecked( i ) == possible_child )
    {
      return true;
    }
  }
  return false;
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
  if ( role == Qt::TextColorRole &&
       ( parent_ && parent_->getDisableChildren() ) )
  {
    return Qt::gray;
  }

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

bool Property::getDisableChildren()
{
  // Pass down the disableChildren flag
  if ( parent_ )
  {
    return parent_->getDisableChildren();
  }
  return false;
}

Qt::ItemFlags Property::getViewFlags( int column ) const
{
  // if the parent propery is a disabled bool property or
  // has its own enabled view flag not set, disable this property as well
  Qt::ItemFlags enabled_flag = ( parent_ && parent_->getDisableChildren() ) ? Qt::NoItemFlags : Qt::ItemIsEnabled;  // || is_read_only_

  if( column == 0 )
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
  Property* child = children_.takeAt( index );
  child->setModel( NULL );
  child->parent_ = NULL;
  child_indexes_valid_ = false;
  if( model_ )
  {
    model_->endRemove();
  }
  Q_EMIT childListChanged( this );
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

  children_.insert( index, child );
  child_indexes_valid_ = false;
  child->setModel( model_ );
  child->parent_ = this;

  if( model_ )
  {
    model_->endInsert();
  }

  Q_EMIT childListChanged( this );
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
  children_.move( from_index, to_index );
  child_indexes_valid_ = false;
  Q_EMIT childListChanged( this );
}

void Property::load( const Config& config )
{
  if( config.getType() == Config::Value )
  {
    loadValue( config );
  }
  else if( config.getType() == Config::Map )
  {
    // A special map entry named "Value" means the value of this property, not a child.
    // (If child "Value"does not exist, loadValue() will do nothing.)
    loadValue( config.mapGetChild( "Value" ));

    // Loop over all child Properties.
    int num_property_children = children_.size();
    for( int i = 0; i < num_property_children; i++ )
    {
      Property* child = children_.at( i );
      // Load the child Property with the config under the child property's name.
      child->load( config.mapGetChild( child->getName() ));
    }
  }
}

void Property::loadValue( const Config& config )
{
  if( config.getType() == Config::Value )
  {
    switch( int( value_.type() ))
    {
    case QVariant::Int: setValue( config.getValue().toInt() ); break;
    case QMetaType::Float:
    case QVariant::Double: setValue( config.getValue().toDouble() ); break;
    case QVariant::String: setValue( config.getValue().toString() ); break;
    case QVariant::Bool: setValue( config.getValue().toBool() ); break;
    default:
      printf( "Property::loadValue() TODO: error handling - unexpected QVariant type %d.\n", int( value_.type() ));
      break;
    }
  }
}

void Property::save( Config config ) const
{
  // If there are child properties, save them in a map from names to children.
  if( children_.size() > 0 )
  {
    // If this property has child properties *and* a value itself,
    // save the value in a special map entry named "Value".
    if( value_.isValid() )
    {
      config.mapSetValue( "Value", value_ );
    }
    int num_properties = children_.size();
    for( int i = 0; i < num_properties; i++ )
    {
      Property* prop = children_.at( i );
      if( prop && prop->shouldBeSaved() )
      {
        prop->save( config.mapMakeChild( prop->getName() ));
      }
    }
  }
  else // Else there are no child properties, so just save the value itself.
  {
    if( value_.isValid() )
    {
      config.setValue( value_ );
    }
    else
    {
      // Empty Properties get saved as empty Maps instead of null values.
      config.setType( Config::Map );
    }
  }
}

QWidget* Property::createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option )
{
  switch( int( value_.type() ))
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
