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

#include <fstream>
#include <sstream>

#include "rviz/config.h"
#include "rviz/uniform_string_stream.h"

namespace rviz
{

bool Config::readFromFile( const std::string& filename )
{
  std::ifstream in( filename.c_str() );
  if( in )
  {
    read( in );
    return true;
  }
  else
  {
    std::cerr << "Config file '" << filename << "' could not be opened for reading." << std::endl;
    return false;
  }
}

bool Config::writeToFile( const std::string& filename )
{
  std::ofstream out( filename.c_str() );
  if( out )
  {
    write( out );
    return true;
  }
  else
  {
    std::cerr << "Config file '" << filename << "' could not be opened for writing." << std::endl;
    return false;
  }
}

void Config::read( std::istream& input )
{
  size_t equals_sign_index;
  std::string line;
  std::string current_dir;
  std::string key, value;

  // Loop over all lines.
  while( !input.eof() && !input.fail() )
  {
    // Read the next line.
    line.clear();
    std::getline( input, line );

    if( line.size() > 0 ) // Ignore empty lines.
    {
      if( line[0] == '[' ) // Keep track of the current "directory" to prepend to the keys.
      {
        current_dir = line.substr( 1, line.size() - 2 );
      }
      else
      {
        // Parse a key=value line.
        equals_sign_index = line.find_first_of( '=' );
        key = line.substr( 0, equals_sign_index );
        key = unescapeKey( key );
        value = line.substr( equals_sign_index + 1 );

        // Store the key,value pair if the key is not empty.
        if( key.size() > 0 )
        {
          if( current_dir.size() > 0 )
          {
            key = current_dir + '/' + key;
          }
          set( key, value );
        }
      }
    }
  }
}

void Config::write( std::ostream& output )
{
  std::string last_prefix;
  std::string key_tail;
  std::string key_prefix;

  for( Iterator it = begin(); it != end(); it++ )
  {
    const std::string& key = (*it).first;
    const std::string& value = (*it).second;
    size_t last_slash_index = key.find_last_of( '/' );
    if( last_slash_index == std::string::npos )
    {
      key_tail = key;
      key_prefix = "";
    }
    else
    {
      key_tail = key.substr( last_slash_index + 1 );
      key_prefix = key.substr( 0, last_slash_index );
    }
    if( key_prefix != last_prefix )
    {
      writeDirectory( output, key_prefix, last_prefix );
    }
    key_tail = escapeKey( key_tail );
    output << key_tail << "=" << value << std::endl;
    last_prefix = key_prefix;
  }
}

// Write a directory description of new_dir to output, given that
// we were previously "in" prev_dir.  For example:
//
// new_dir = a/c, prev_dir = a/b/z, output:
//   [a/c]
//
// new_dir = a/c, prev_dir = a, output:
//   [a/c]
//
// new_dir = a/c, prev_dir = "", output:
//   [a]
//   [a/c]
//
// new_dir = a/c, prev_dir = "b", output:
//   [a]
//   [a/c]
//
// new_dir = a/b/c, prev_dir = "a", output:
//   [a/b]
//   [a/b/c]
//
// new_dir = a/b/c, prev_dir = "a/z/y", output:
//   [a/b]
//   [a/b/c]
void Config::writeDirectory( std::ostream& output, const std::string& new_dir, const std::string& prev_dir )
{
  // Find common initial substring between new_dir and prev_dir
  size_t min_size = new_dir.size() < prev_dir.size() ? new_dir.size() : prev_dir.size();

  size_t index = 0; // index of first non-matching char.
  for( ; index < min_size; index++ )
  {
    if( new_dir[ index ] != prev_dir[ index ] )
    {
      break;
    }
  }

  // If we are pointed at a '/' move just past it.
  if( index < new_dir.size() && new_dir[ index ] == '/' )
  {
    index++;
  }

  // Search forward for first '/' after the matching part.  That will
  // be the end of the first string we need to print.
  bool done = false;
  while( !done )
  {
    index = new_dir.find_first_of( '/', index );
    if( index == std::string::npos )
    {
      index = new_dir.size();
      done = true;
    }
    output << '[' << new_dir.substr( 0, index ) << ']' << std::endl;
    index++; // Skip the '/'
  }
}

std::string Config::escapeKey( const std::string& raw_key )
{
  std::istringstream in( raw_key );
  std::ostringstream out;
  char c;
  while( in.good() )
  {
    c = in.get();
    if( in )
    {
      switch( c )
      {
      case ':':
      case ' ':
      case '\\':
        out << '\\';
      }
      out << c;
    }
  }
  return out.str();
}

std::string Config::unescapeKey( const std::string& cooked_key )
{
  std::istringstream in( cooked_key );
  std::ostringstream out;
  char c;
  while( in.good() )
  {
    c = in.get();
    if( in.good() )
    {
      if( c == '\\' )
      {
        c = in.get();
        if( in.good() )
        {
          out << c;
        }
      }
      else
      {
        out << c;
      }
    }
  }
  return out.str();
}

void Config::set( const std::string& key, const std::string& value )
{
  map_[ stripFirstSlash( key )] = value;
}

void Config::set( const std::string& key, float value )
{
  UniformStringStream ss;
  ss << value;
  map_[ stripFirstSlash( key )] = ss.str();
}

void Config::set( const std::string& key, int value )
{
  UniformStringStream ss;
  ss << value;
  map_[ stripFirstSlash( key )] = ss.str();
}

bool Config::get( const std::string& key, std::string* output, const std::string& default_value )
{
  Iterator it = map_.find( stripFirstSlash( key ));
  if( it != map_.end() )
  {
    *output = (*it).second;
    return true;
  }
  *output = default_value;
  return false;
}

bool Config::get( const std::string& key, float* output, float default_value )
{
  Iterator it = map_.find( stripFirstSlash( key ));
  if( it != map_.end() )
  {
    UniformStringStream ss;
    ss.str( (*it).second );
    ss.parseFloat( *output );
    if( !ss.fail() )
    {
      return true;
    }
  }
  *output = default_value;
  return false;
}

bool Config::get( const std::string& key, int* output, int default_value )
{
  Iterator it = map_.find( stripFirstSlash( key ));
  if( it != map_.end() )
  {
    UniformStringStream ss;
    ss.str( (*it).second );
    ss >> *output;
    if( !ss.fail() )
    {
      return true;
    }
  }
  *output = default_value;
  return false;
}

const std::string Config::stripFirstSlash( const std::string& str )
{
  if( str[0] == '/' )
  {
    return str.substr( 1 );
  }
  else
  {
    return str;
  }
}

bool Config::DirectoryCompare::operator() (const std::string& lhs, const std::string& rhs) const
{
  int start = 0;
  int count;
  int rhs_count;
  size_t l_slash_index, r_slash_index;
  bool l_on_last, r_on_last;

  // Loop once for each section of the strings (where sections are
  // delimited by '/' characters) as long as the corresponding
  // sections are equal.  When we come to a section in which lhs !=
  // rhs, return either true or false.
  while( true )
  {
    // Find the index of the next slash in each string, if there is one.
    l_slash_index = lhs.find_first_of( '/', start );
    r_slash_index = rhs.find_first_of( '/', start );
    l_on_last = ( l_slash_index == std::string::npos );
    if( l_on_last )
    {
      l_slash_index = lhs.size();
    }
    r_on_last = ( r_slash_index == std::string::npos );
    if( r_on_last )
    {
      r_slash_index = rhs.size();
    }

    // If one but not both of the strings is on the last section, we
    // know the result.  A string which is on its last element is
    // "less" than a string which has more elements coming, regardless
    // of the current-element comparison.
    if( !l_on_last && r_on_last )
    {
      return false;
    }
    if( l_on_last && !r_on_last )
    {
      return true;
    }

    // Update the comparison lengths.
    count = l_slash_index - start;
    rhs_count = r_slash_index - start;

    // Compare the current section of each string.
    int result = lhs.compare( start, count, rhs, start, rhs_count );

    // If the sections differ, return true or false according to the
    // direction of difference.
    if( result != 0 )
    {
      return result < 0;
    }

    // The sections are equal, so if the rhs is ending, then the rhs
    // is less, so return false.
    if( start + rhs_count >= (int)rhs.size() )
    {
      return false;
    }

    // Move start index to the next section, skipping over the '/'
    // character with the +1.
    start += count + 1;

    // The sections are equal, so if the lhs is ending, it is less, so
    // return true.
    if( start > (int)lhs.size() )
    {
      return true;
    }
  }
}

} // end namespace rviz
