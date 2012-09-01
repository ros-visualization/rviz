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
#ifndef CONFIG_H
#define CONFIG_H

#include <stdio.h>

#include <boost/shared_ptr.hpp>

#include <QMap>
#include <QString>
#include <QVariant>

namespace rviz
{

class Config
{
private:
  class Node;
  typedef boost::shared_ptr<Node> NodePtr;

public:
  /** @brief Default constructor.  Creates an empty config object. */
  Config();
  /** @brief Copy constructor.  Copies only the reference to the data, so creates a shallow copy. */
  Config( const Config& source );
  /** @brief Convenience constructor, makes a Value type Config object with the given value. */
  Config( QVariant value );

  enum Type { Map, List, Value, Empty, Invalid };
  Type getType() const;
  void setType( Type );

  /** @brief Returns getType() != Invalid. */
  bool isValid() const;

  void mapSetValue( const QString& key, QVariant value );
  void mapSetChild( const QString& key, const Config& child );
  Config mapGetChild( const QString& key );

  /** @brief Ensures this is a valid Config object, sets the type to Value then sets the value. */
  void setValue( const QVariant& value );
  /** @brief If this config object is valid, this returns its value.  If not, it returns an invalid QVariant. */
  QVariant getValue() const;

  int listLength() const;
  Config listChildAt( int i ) const;
  void listAppend( const Config& child );

  class MapIterator
  {
  public:
    /** @brief Advance iterator to next entry. */
    void advance();

    /** @brief Return true if the iterator currently points to a valid entry, false if not. */
    bool isValid();
  
    /** @brief Resets the iterator to the start of the map. */
    void start();

    /** @brief Return the name of the current map entry. */
    QString currentKey();

    /** @brief Return the config reference of the current map entry. */
    Config currentChild();

  private:
    MapIterator();
    Config::NodePtr node_;
    QMap<QString, Config::NodePtr>::const_iterator iterator_;
    bool iterator_valid_;
    friend class Config;
  };

  /** @brief Return a new iterator for looping over key/value pairs. */
  MapIterator mapIterator() const;

private:
  Config( NodePtr node );
  static Config invalidConfig();

  /** @brief If the node pointer is NULL, this sets it to a new empty node. */
  void makeValid();

  NodePtr node_;

  friend class MapIterator;
};

} // end namespace rviz

#endif // CONFIG_H
