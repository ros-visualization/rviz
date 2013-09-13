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

#include "rviz/display_group.h"

#include "rviz/display_factory.h"

#include <tinyxml.h>

namespace rviz
{

static Display* newDisplayGroup()
{
  return new DisplayGroup();
}

DisplayFactory::DisplayFactory()
  : PluginlibFactory<Display>( "rviz", "rviz::Display" )
{
  addBuiltInClass( "rviz", "Group", "A container for Displays", &newDisplayGroup );
}

Display* DisplayFactory::makeRaw( const QString& class_id, QString* error_return )
{
  Display* display = PluginlibFactory<Display>::makeRaw( class_id, error_return );
  if ( display )
  {
    display->setIcon( getIcon( class_id ));
  }
  return display;
}

/** @brief Get all supported message types for the  */
QSet<QString> DisplayFactory::getTopicTypes( const QString& class_id ) const
{
  QSet<QString> topic_types;

  QString xml_file = getPluginManifestPath( class_id );

  if ( !xml_file.isEmpty() )
  {
    TiXmlDocument document;
    document.LoadFile(xml_file.toStdString());
    TiXmlElement * config = document.RootElement();
    if (config == NULL)
    {
      ROS_ERROR_NAMED("rviz.DisplayFactory","Skipping XML Document \"%s\" which had no Root Element.  This likely means the XML is malformed or missing.", xml_file.toStdString().c_str());
      return topic_types;
    }
    if (config->ValueStr() != "library" &&
        config->ValueStr() != "class_libraries")
    {
      ROS_ERROR_NAMED("rviz.DisplayFactory","The XML document \"%s\" given to add must have either \"library\" or \
          \"class_libraries\" as the root tag", xml_file.toStdString().c_str());
      return topic_types;
    }
    //Step into the filter list if necessary
    if (config->ValueStr() == "class_libraries")
    {
      config = config->FirstChildElement("library");
    }

    TiXmlElement* library = config;
    while ( library != NULL)
    {
      TiXmlElement* class_element = library->FirstChildElement("class");
      while (class_element)
      {
        std::string derived_class = class_element->Attribute("type");

        std::string lookup_name;
        if(class_element->Attribute("name") != NULL)
        {
          lookup_name = class_element->Attribute("name");
          ROS_DEBUG_NAMED("rviz.DisplayFactory","XML file specifies lookup name (i.e. magic name) = %s.", lookup_name.c_str());
        }
        else
        {
          ROS_DEBUG_NAMED("rviz.DisplayFactory","XML file has no lookup name (i.e. magic name) for class %s, assuming lookup_name == real class name.", derived_class.c_str());
          lookup_name = derived_class;
        }

        std::cout << lookup_name << std::endl;

        if ( lookup_name == class_id.toStdString() )
        {
          TiXmlElement* message_type = class_element->FirstChildElement("message_type");

          while ( message_type )
          {
            if ( message_type->GetText() )
            {
              const char* message_type_str = message_type->GetText();
              std::cout << class_id.toStdString() << " supports message type " << message_type_str << std::endl;
              topic_types.insert( QString::fromAscii( message_type_str ) );
            }
            message_type = message_type->NextSiblingElement("message_type");
          }
        }

        //step to next class_element
        class_element = class_element->NextSiblingElement( "class" );
      }
      library = library->NextSiblingElement( "library" );
    }
  }

  return topic_types;
}


} // end namespace rviz
