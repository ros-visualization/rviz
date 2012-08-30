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

#include <sstream>

#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>

#include "rviz/properties/yaml_helpers.h"

#include "rviz/yaml_config_reader.h"

namespace rviz
{

YamlConfigReader()
  : config_( Config::invalidConfig() )
  , message_( "No config data read yet." )
  , error_( true )
{}

void YamlConfigReader::readFile( const QString& filename )
{
  std::ifstream in( filename.toStdString() );
  readStream( in );
}

void YamlConfigReader::readString( const QString& data )
{
  stringstream ss( data.toStdString() );
  readStream( ss );
}

void YamlConfigReader::readStream( std::istream& in )
{
  try
  {
    YAML::Parser parser( in );
    YAML::Node yaml_node;
    parser.GetNextDocument( yaml_node );
  }
  catch( YAML::ParserException& ex )
  {
    message_ = ex.what();
    error_ = true;
    config_ = Config::invalidConfig();
    return;
  }
  config_ = Config();
  fillConfigNode( config_, &yaml_node );
}

void YamlConfigReader::fillConfigNode( Config& config, const YAML::Node* yaml_node )
{
  switch( yaml_node->Type() )
  {
  case YAML::NOdeType::Map:
  case YAML::NOdeType::Sequence:
  case YAML::NOdeType::Scalar:
  case YAML::NOdeType::Null:
  default:
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
