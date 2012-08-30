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

#include <boost/shared_ptr.hpp>

#include <QMap>
#include <QString>
#include <QVariant>

namespace rviz
{

class ConfigSequence;
class ConfigMapIterator;

class Config
{
private:
  class Node;
  typedef boost::shared_ptr<Node> NodePtr;

public:
  /** @brief Default constructor.  Creates a valid but empty configuration. */
  Config();
  /** @brief Copy constructor.  Copies only the reference to the data, so creates a shallow copy. */
  Config( const Config& source );

  Config makeChild( const QString& name );
  Config getChild( const QString& name ) const;
  bool isValid() const;

  /** @brief Ensures this is a valid Config object then sets the value. */
  void setValue( const QVariant& value );

  /** @brief If this config object is valid, this returns its value.  If not, it returns an invalid QVariant. */
  QVariant getValue() const;

  /** @brief Makes this Config element a sequence container and returns a reference to the sequence interface. */
  ConfigSequence makeSequence();

  /** @brief If this Config element is a sequence container, this
   * returns a reference to the sequence interface.  If it is not,
   * returns an invalid reference. */
  ConfigSequence getSequence();

  /** @brief Returns true if this Config element is a sequence container. */
  bool isSequence();

  /** @brief Return a new iterator for looping over key/value pairs. */
  ConfigMapIterator mapIterator();

  /** @brief Return an invalid Config object. */
  static Config invalidConfig() { return Config( NodePtr() ); }

private:
  Config( NodePtr node );

  NodePtr node_;

  friend class ConfigSequence;
  friend class ConfigMapIterator;
};

} // end namespace rviz

#include "rviz/config_sequence.h"
#include "rviz/config_map_iterator.h"

#endif // CONFIG_H
