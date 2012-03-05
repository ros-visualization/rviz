/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef RVIZ_CONFIG_H
#define RVIZ_CONFIG_H

#include <string>
#include <iostream>
#include <map>

namespace rviz
{

/** Configuration data structure.  Inspired by rviz's use of
 * wxConfigBase.  Current implementation uses a flat ordered map, but
 * in the long run it may more sense to use a recursive data
 * structure.
 */
class Config
{
public:
  /** @return success or failure of the initial file-open. */
  bool readFromFile( const std::string& filename );

  /** @return success or failure of the initial file-open. */
  bool writeToFile( const std::string& filename );

  /** Read settings from a stream.  Invalid lines are ignored. */
  void read( std::istream& input );

  /** Write settings to a stream. */
  void write( std::ostream& output );

  void set( const std::string& key, const std::string& value );
  void set( const std::string& key, float value );
  void set( const std::string& key, int value );

  /** Remove a key/value pair.  This will invalidate iterators
   * pointing at the given element. */
  void unset( const std::string& key ) { map_.erase( key ); }

  bool get( const std::string& key, std::string* output, const std::string& default_value = "" );
  bool get( const std::string& key, float* output, float default_value = 0 );
  bool get( const std::string& key, int* output, int default_value = 0 );

  class DirectoryCompare
  {
  public:
    bool operator() (const std::string& lhs, const std::string& rhs) const;
  };

  typedef std::map<std::string, std::string, DirectoryCompare> Map;
  typedef Map::iterator Iterator;

  Iterator begin() { return map_.begin(); }
  Iterator end() { return map_.end(); }

  void clear() { map_.clear(); }

private:
  /** If the first char of str is a '/', return everything but the
   * first char in a new string.  If it is not, return str. */
  const std::string stripFirstSlash( const std::string& str );

  /** Write a directory description of new_dir to output, given that
   * we were previously "in" prev_dir. */
  void writeDirectory( std::ostream& output, const std::string& new_dir, const std::string& prev_dir );

  /** Escape ' ' and ':' characters with '\'. */
  std::string escapeKey( const std::string& raw_key );

  /** Un-escape ' ' and ':' characters with '\'. */
  std::string unescapeKey( const std::string& cooked_key );

  Map map_;
};

} // end namespace rviz

#endif // RVIZ_CONFIG_H
