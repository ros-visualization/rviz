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

#ifdef RVIZ_HAVE_YAMLCPP_05
#include <yaml-cpp/yaml.h>
#else
#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>
#endif

#include "rviz/yaml_config_reader.h"

namespace rviz
{

YamlConfigReader::YamlConfigReader()
  : error_( false )
{}

void YamlConfigReader::readFile( Config& config, const QString& filename )
{
  std::ifstream in( qPrintable( filename ));
  readStream( config, in, filename );
}

void YamlConfigReader::readString( Config& config, const QString& data, const QString& filename )
{
  std::stringstream ss( data.toStdString() );
  readStream( config, ss, filename );
}

void YamlConfigReader::readStream( Config& config, std::istream& in, const QString& filename )
{
  try
  {
    YAML::Node yaml_node;
#ifdef RVIZ_HAVE_YAMLCPP_05
    yaml_node = YAML::Load(in);
#else
    YAML::Parser parser( in );
    parser.GetNextDocument( yaml_node );
#endif
    error_ = false;
    message_ = "";
    readYamlNode( config, yaml_node );
  }
  catch( YAML::ParserException& ex )
  {
    message_ = ex.what();
    error_ = true;
  }
}

void YamlConfigReader::readYamlNode( Config& config, const YAML::Node& yaml_node )
{
  switch( yaml_node.Type() )
  {
  case YAML::NodeType::Map:
  {
#ifdef RVIZ_HAVE_YAMLCPP_05
    for( YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
#else
    for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
#endif
    {
      std::string key;
#ifdef RVIZ_HAVE_YAMLCPP_05
      key = it->first.as<std::string>();
#else
      it.first() >> key;
#endif
      Config child = config.mapMakeChild( QString::fromStdString( key ));
#ifdef RVIZ_HAVE_YAMLCPP_05
      readYamlNode( child, it->second );
#else
      readYamlNode( child, it.second() );
#endif
    }
    break;
  }
  case YAML::NodeType::Sequence:
  {
#ifdef RVIZ_HAVE_YAMLCPP_05
    for( YAML::const_iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
#else
    for( YAML::Iterator it = yaml_node.begin(); it != yaml_node.end(); ++it )
#endif
    {
      Config child = config.listAppendNew();
      readYamlNode( child, *it );
    }
    break;
  }
  case YAML::NodeType::Scalar:
  {
    std::string s;
#ifdef RVIZ_HAVE_YAMLCPP_05
    s = yaml_node.as<std::string>();
#else
    yaml_node >> s;
#endif
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

QString YamlConfigReader::errorMessage()
{
  return message_;
}

} // end namespace rviz
