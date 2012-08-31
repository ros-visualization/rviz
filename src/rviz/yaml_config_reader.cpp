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

#include <fstream>
#include <sstream>

#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>

#include "rviz/properties/yaml_helpers.h"

#include "rviz/yaml_config_reader.h"

namespace rviz
{

YamlConfigReader::YamlConfigReader()
  : message_( "No config data read yet." )
  , error_( true )
{}

void YamlConfigReader::readFile( const QString& filename )
{
  std::ifstream in( qPrintable( filename ));
  readStream( in, filename );
}

void YamlConfigReader::readString( const QString& data, const QString& filename )
{
  std::stringstream ss( data.toStdString() );
  readStream( ss, filename );
}

void YamlConfigReader::readStream( std::istream& in, const QString& filename )
{
  try
  {
    YAML::Parser parser( in );
    YAML::Node yaml_node;
    parser.GetNextDocument( yaml_node );
    config_ = Config();
    error_ = false;
    message_ = "Read config from " + filename;
    fillConfigNode( config_, yaml_node );
  }
  catch( YAML::ParserException& ex )
  {
    message_ = ex.what();
    error_ = true;
    config_ = Config();
  }
}

void YamlConfigReader::fillConfigNode( Config& config, const YAML::Node& yaml_node )
{
  switch( yaml_node.Type() )
  {
  case YAML::NodeType::Map:
  {
    for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
    {
      std::string key;
      it.first() >> key;
      Config config_child;
      fillConfigNode( config_child, it.second() );
      config.mapSetChild( QString::fromStdString( key ), config_child );
    }
    break;
  }
  case YAML::NodeType::Sequence:
  {
    for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
    {
      Config config_child;
      fillConfigNode( config_child, *it );
      config.listAppend( config_child );
    }
    break;
  }
  case YAML::NodeType::Scalar:
  {
    std::string s;
    yaml_node >> s;
    config.setValue( QString::fromStdString( s ));
    break;
  }
  case YAML::NodeType::Null:
  default:
    break;
  }
}

bool YamlConfigReader::error()
{
  return error_;
}

QString YamlConfigReader::statusMessage()
{
  return message_;
}

Config YamlConfigReader::config()
{
  return config_;
}

} // end namespace rviz
