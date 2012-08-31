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

  NodePtr makeChild( const QString& name );
  NodePtr getChild( const QString& name ) const;

  void setValue( const QVariant& value );
  QVariant getValue() const;

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
  : type_( Invalid )
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
  case Sequence: delete data_.list; break;
  case Scalar: delete data_.value; break;
  case Invalid:
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
  case Map:      data_.map =   new ChildMap;  break;
  case Sequence: data_.list =  new ChildList; break;
  case Scalar:   data_.value = new QVariant;  break;
  case Invalid:
  default:
    break;
  }
}

Config::NodePtr Config::Node::makeChild( const QString& name )
{
  setType( Map );

  ChildMap::iterator iter = data_.map->find( name );
  if( iter == data_.map->end() )
  {
    NodePtr child = NodePtr( new Config::Node() );
    (*data_.map)[ name ] = child;
    return child;
  }
  else
  {
    return iter.value();
  }
}

Config::NodePtr Config::Node::getChild( const QString& name ) const
{
  if( type_ != Map )
  {
    return NodePtr();
  }
  ChildMap::const_iterator iter = data_.map->find( name );
  if( iter == data_.map->end() )
  {
    return NodePtr();
  }
  else
  {
    return iter.value();
  }
}

void Config::Node::setValue( const QVariant& value )
{
  setType( Scalar );
  *data_.value = value;
}

QVariant Config::Node::getValue() const
{
  if( type_ == Scalar )
  {
    return *data_.value;
  }
  else
  {
    return QVariant();
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

Config::Config( NodePtr node )
  : node_( node )
{
  if( node_.get() == NULL )
  {
    node_.reset( new Node() );
  }
}

Config::Type Config::getType() const
{
  if( node_.get() == NULL )
  {
    return Invalid;
  }
  else
  {
    return node_->type_;
  }
}

Config Config::makeChild( const QString& name )
{
  return Config( node_->makeChild( name ));
}

Config Config::getChild( const QString& name ) const
{
  return Config( node_->getChild( name ));
}

bool Config::isValid() const
{
  return node_->type_ != Invalid;
}

void Config::setValue( const QVariant& value )
{
  node_->setValue( value );
}

QVariant Config::getValue() const
{
  return node_->getValue();
}

ConfigSequence Config::makeSequence()
{
  node_->setType( Sequence );

  // Create a new sequence object with next_child_num_ of 0.
  ConfigSequence seq = ConfigSequence();

  // Copy this config's node reference into the sequence's node reference.
  seq.node_ = node_;

  return seq;
}

ConfigSequence Config::getSequence() const
{
  // Create a new sequence object with next_child_num_ of 0.
  ConfigSequence seq = ConfigSequence();

  if( node_.get() == NULL || node_->type_ != Sequence )
  {
    // Force the sequence to be invalid, since this node does not have a sequence.
    seq.node_.reset();
  }
  else
  {
    // Copy this config's node reference into the sequence's node reference.
    seq.node_ = node_;
  }
  return seq;
}

ConfigMapIterator Config::mapIterator() const
{
  // Create a new (invalid) iterator.
  ConfigMapIterator iter;

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

///////////////////////////////////////////////////////////////////////////////////////////
// Sequence type implementation
///////////////////////////////////////////////////////////////////////////////////////////

ConfigSequence::ConfigSequence()
  : next_child_num_( 0 )
{}

Config ConfigSequence::makeNext()
{
  // Ensure we are talking about a sequence.
  node_->setType( Config::Sequence );

  // Ensure the sequence has a next item.
  Config::Node::ChildList* list = node_->data_.list;
  if( next_child_num_ >= list->size() )
  {
    list->reserve( next_child_num_ + 1 );
    for( int i = next_child_num_ - list->size(); i >= 0; i-- )
    {
      list->append( Config::NodePtr() );
    }
  }

  // Advance the index.
  int current_index = next_child_num_;
  next_child_num_++;

  // Make sure the next item is meaningful.
  if( (*list)[ current_index ].get() == NULL )
  {
    (*list)[ current_index ].reset( new Config::Node() );
  }

  // Return the next item.
  return Config( (*list)[ current_index ]);
}

Config ConfigSequence::getNext()
{
  if( node_->type_ != Config::Sequence )
  {
    return Config();
  }
  Config::Node::ChildList* list = node_->data_.list;
  if( next_child_num_ >= list->size() )
  {
    return Config();
  }
  int current_index = next_child_num_;
  next_child_num_++;
  return Config( list->at( current_index ));
}

void ConfigSequence::start()
{
  next_child_num_ = 0;
}

bool ConfigSequence::hasNext()
{
  if( node_.get() == NULL || node_->type_ != Config::Sequence )
  {
    return false;
  }
  Config::Node::ChildList* list = node_->data_.list;
  return next_child_num_ < list->size();
}

ConfigMapIterator::ConfigMapIterator()
  : iterator_valid_( false )
{}

void ConfigMapIterator::next()
{
  if( node_.get() == NULL || node_->type_ != Config::Map )
  {
    iterator_valid_ = false;
    return;
  }
  if( !iterator_valid_ )
  {
    iterator_ = node_->data_.map->begin();
  }
  else
  {
    iterator_++;
  }
}

bool ConfigMapIterator::hasNext()
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

void ConfigMapIterator::start()
{
  if( node_.get() == NULL || node_->type_ != Config::Map )
  {
    iterator_valid_ = false;
    return;
  }
  iterator_ = node_->data_.map->begin();
  iterator_valid_ = true;
}

QString ConfigMapIterator::currentKey()
{
  if( node_.get() == NULL || node_->type_ != Config::Map || !iterator_valid_ )
  {
    iterator_valid_ = false;
    return QString();
  }
  return iterator_.key();
}

Config ConfigMapIterator::currentChild()
{
  if( node_.get() == NULL || node_->type_ != Config::Map || !iterator_valid_ )
  {
    iterator_valid_ = false;
    return Config();
  }
  return Config( iterator_.value() );
}

} // end namespace rviz
