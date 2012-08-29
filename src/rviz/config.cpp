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
#include "rviz/config_sequence.h"

namespace rviz
{

////////////////////////////////////////////////////////////////////////////////////
// Config::Node internal data storage class
////////////////////////////////////////////////////////////////////////////////////

class Config::Node
{
public:
  Node();

  NodePtr makeChild( const QString& name );
  NodePtr getChild( const QString& name ) const;

  void setValue( const QVariant& value );
  QVariant getValue() const;

  void makeMap();
  void makeSequence();

  typedef QMap<QString, NodePtr> ChildMap;
  typedef QList<NodePtr> ChildList;
  inline ChildMap* childrenAsMap() const { return static_cast<ChildMap*>( children_ ); }
  inline ChildList* childrenAsList() const { return static_cast<ChildList*>( children_ ); }

  bool is_sequence_;
  void *children_; // is this a good idea?? pointer to either QMap or QList??
  QVariant value_;
};

Config::Node::Node()
  : is_sequence_( false )
  , children_( NULL )
{}

void Config::Node::makeMap()
{
  if( is_sequence_ )
  {
    delete childrenAsList();
    children_ = NULL;
    is_sequence_ = false;
  }
  if( children_ == NULL )
  {
    children_ = new ChildMap;
  }
}

void Config::Node::makeSequence()
{
  if( !is_sequence_ )
  {
    delete childrenAsMap();
    children_ = NULL;
    is_sequence_ = true;
  }
  if( children_ == NULL )
  {
    children_ = new ChildList;
  }
}

Config::NodePtr Config::Node::makeChild( const QString& name )
{
  makeMap();

  ChildMap* map = childrenAsMap();
  ChildMap::iterator iter = map->find( name );
  if( iter == map->end() )
  {
    NodePtr child = NodePtr( new Config::Node() );
    (*map)[ name ] = child;
    return child;
  }
  else
  {
    return iter.value();
  }
}

Config::NodePtr Config::Node::getChild( const QString& name ) const
{
  const ChildMap* map = childrenAsMap();
  ChildMap::const_iterator iter = map->find( name );
  if( iter == map->end() )
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
  value_ = value;
}

QVariant Config::Node::getValue() const
{
  return value_;
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
{}

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
  return node_.get() != NULL;
}

void Config::setValue( const QVariant& value )
{
  if( !isValid() )
  {
    node_ = NodePtr( new Config::Node() );
  }
  node_->setValue( value );
}

QVariant Config::getValue() const
{
  if( isValid() )
  {
    return node_->getValue();
  }
  return QVariant();
}

ConfigSequence Config::makeSequence()
{
  // Force this config to be valid.
  if( !isValid() )
  {
    node_ = NodePtr( new Config::Node() );
  }
  // Force it to be a sequence.
  node_->makeSequence();

  // Create a new sequence object with next_child_num_ of 0.
  ConfigSequence seq = ConfigSequence();

  // Copy this config's node reference into the sequence's node reference.
  seq.node_ = node_;

  return seq;
}

ConfigSequence Config::getSequence()
{
  // Create a new sequence object with next_child_num_ of 0.
  ConfigSequence seq = ConfigSequence();

  if( !isValid() || !node_->is_sequence_ )
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

bool Config::isSequence()
{
  if( !isValid() )
  {
    return false;
  }
  return node_->is_sequence_;
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
  node_->makeSequence();

  // Ensure the sequence has a next item.
  Config::Node::ChildList* list = node_->childrenAsList();
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
  if( !node_->is_sequence_ )
  {
    return Config( Config::NodePtr() );
  }
  Config::Node::ChildList* list = node_->childrenAsList();
  if( next_child_num_ >= list->size() )
  {
    return Config( Config::NodePtr() );
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
  if( node_.get() == NULL || !node_->is_sequence_ )
  {
    return false;
  }
  Config::Node::ChildList* list = node_->childrenAsList();
  return next_child_num_ < list->size();
}

} // end namespace rviz
