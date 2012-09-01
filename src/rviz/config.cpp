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

#include "rviz/config.h"

namespace rviz
{

////////////////////////////////////////////////////////////////////////////////////
// Config::Node internal data storage class
////////////////////////////////////////////////////////////////////////////////////

class Config::Node
{
public:
  Node();
  ~Node();

  void setType( Config::Type new_type );
  void deleteData();

  typedef QMap<QString, NodePtr> ChildMap;
  typedef QList<NodePtr> ChildList;

  Config::Type type_;
  union
  {
    ChildMap* map;
    ChildList* list;
    QVariant* value;
  } data_;
};

Config::Node::Node()
  : type_( Empty )
{
  data_.map = NULL;
}

Config::Node::~Node()
{
  deleteData();
}

void Config::Node::deleteData()
{
  switch( type_ )
  {
  case Map: delete data_.map; break;
  case List: delete data_.list; break;
  case Value: delete data_.value; break;
  default:
    break;
  }
  data_.map = NULL;
}

void Config::Node::setType( Config::Type new_type )
{
  if( type_ == new_type )
  {
    return;
  }
  deleteData();
  type_ = new_type;
  switch( type_ )
  {
  case Map:   data_.map =   new ChildMap;  break;
  case List:  data_.list =  new ChildList; break;
  case Value: data_.value = new QVariant;  break;
  default:                                    break;
  }
}

////////////////////////////////////////////////////////////////////////////////////
// Config wrapper class
////////////////////////////////////////////////////////////////////////////////////

Config::Config()
  : node_( new Config::Node() )
{}

Config::Config( const Config& source )
  : node_( source.node_ )
{}

Config::Config( QVariant value )
  : node_( new Config::Node() )
{
  setValue( value );
}

Config::Config( NodePtr node )
  : node_( node )
{}

Config Config::invalidConfig()
{
  return Config( NodePtr() );
}

Config::Type Config::getType() const
{
  return isValid() ? node_->type_ : Invalid;
}

void Config::setType( Type new_type )
{
  makeValid();
  node_->setType( new_type );
}

void Config::mapSetValue( const QString& key, QVariant value )
{
  mapSetChild( key, Config( value ));
}

void Config::mapSetChild( const QString& key, const Config& child )
{
  makeValid();
  node_->setType( Map );
  (*node_->data_.map)[ key ] = child.node_;
}

Config Config::mapGetChild( const QString& key )
{
  if( node_.get() == NULL || node_->type_ != Map )
  {
    return invalidConfig();
  }
  Node::ChildMap::const_iterator iter = node_->data_.map->find( key );
  if( iter == node_->data_.map->end() )
  {
    return invalidConfig();
  }
  else
  {
    return Config( iter.value() );
  }
}

void Config::makeValid()
{
  if( node_.get() == NULL )
  {
    node_.reset( new Node() );
  }
}

bool Config::isValid() const
{
  return node_.get() != NULL;
}

void Config::setValue( const QVariant& value )
{
  makeValid();
  node_->setType( Value );
  *node_->data_.value = value;
}

QVariant Config::getValue() const
{
  return ( isValid() && node_->type_ == Value ) ? *node_->data_.value : QVariant();
}

int Config::listLength() const
{
  return ( isValid() && node_->type_ == List ) ? node_->data_.list->size() : 0;
}

Config Config::listChildAt( int i ) const
{
  if( isValid() && node_->type_ == List && i >= 0 && i < node_->data_.list->size() )
  {
    return Config( node_->data_.list->at( i ));
  }
  else
  {
    return invalidConfig();
  }
}

void Config::listAppend( const Config& child )
{
  setType( List );
  node_->data_.list->append( child.node_ );
}

Config::MapIterator Config::mapIterator() const
{
  // Create a new (invalid) iterator.
  Config::MapIterator iter;

  if( node_.get() == NULL || node_->type_ != Map )
  {
    // Force the node to be invalid, since this node does not have a map.
    iter.node_.reset();
  }
  else
  {
    // Copy this config's node reference into the iterator's node reference.
    iter.node_ = node_;
    iter.start();
  }
  return iter;
}

Config::MapIterator::MapIterator()
  : iterator_valid_( false )
{}

void Config::MapIterator::next()
{
  if( node_.get() == NULL || node_->type_ != Config::Map )
  {
    iterator_valid_ = false;
    return;
  }
  if( !iterator_valid_ )
  {
    iterator_ = node_->data_.map->begin();
    iterator_valid_ = true;
  }
  else
  {
    iterator_++;
  }
}

bool Config::MapIterator::hasNext()
{
  if( node_.get() == NULL || node_->type_ != Config::Map )
  {
    iterator_valid_ = false;
    return false;
  }
  if( !iterator_valid_ )
  {
    return false;
  }
  else
  {
    return iterator_ != node_->data_.map->end();
  }
}

void Config::MapIterator::start()
{
  if( node_.get() == NULL || node_->type_ != Config::Map )
  {
    iterator_valid_ = false;
    return;
  }
  iterator_ = node_->data_.map->begin();
  iterator_valid_ = true;
}

QString Config::MapIterator::currentKey()
{
  if( node_.get() == NULL || node_->type_ != Config::Map || !iterator_valid_ )
  {
    iterator_valid_ = false;
    return QString();
  }
  return iterator_.key();
}

Config Config::MapIterator::currentChild()
{
  if( node_.get() == NULL || node_->type_ != Config::Map || !iterator_valid_ )
  {
    iterator_valid_ = false;
    return Config();
  }
  return Config( iterator_.value() );
}

} // end namespace rviz
