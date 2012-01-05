/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "display_wrapper.h"
#include "display.h"
#include "visualization_manager.h"
// #include "plugin/display_type_info.h"
// #include "plugin/plugin.h"
// #include "plugin/display_creator.h"
#include "properties/property_manager.h"
#include "properties/property.h"

#include <ros/assert.h>

#include "config.h"

namespace rviz
{

DisplayWrapper::DisplayWrapper( const std::string& class_lookup_name,
                                pluginlib::ClassLoader<Display>* class_loader,
                                const std::string& name,
                                VisualizationManager* manager )
: manager_(manager)
, class_loader_( class_loader )
, name_(name)
, class_lookup_name_(class_lookup_name)
, display_(0)
, property_manager_(0)
, enabled_(true)
{
  connect( manager, SIGNAL( displaysConfigLoaded( const boost::shared_ptr<Config>& )),
           this, SLOT( onDisplaysConfigLoaded( const boost::shared_ptr<Config>& )));
  connect( manager, SIGNAL( displaysConfigSaved( const boost::shared_ptr<Config>& )),
           this, SLOT( onDisplaysConfigSaved( const boost::shared_ptr<Config>& )));
}

DisplayWrapper::~DisplayWrapper()
{
  destroyDisplay();

  if (property_manager_)
  {
    property_manager_->deleteProperty(category_.lock());
  }
}

void DisplayWrapper::setName( const std::string& name )
{
  std::string old_name = name_;
  name_ = name;
  M_string new_props;
  if (display_)
  {
    display_->setName(name);

    if (config_)
    {
      for( Config::Iterator ci = config_->begin(); ci != config_->end(); )
      {
        std::string key = (*ci).first;
        std::string value = (*ci).second;

        ci++; // Advance the iterator before possibly removing the item it points to.

        if( key.find( old_name + "." ) == 0 )
        {
          std::string new_key = name + key.substr( old_name.size() + 1 );
          config_->set( new_key, value );
          config_->unset( key );

          new_props[ key ] = value;
        }
      }
    }

    if (property_manager_)
    {
      property_manager_->changePrefix(old_name + ".", name + ".");
    }
  }
  else
  {
    M_string::iterator it = properties_.begin();
    M_string::iterator end = properties_.end();
    for (; it != end; ++it)
    {
      const std::string& key = it->first;
      const std::string& val = it->second;

      std::string new_key = name + "." + key.substr( old_name.size() + 1 );
      new_props[new_key] = val;

      if (config_)
      {
        config_->unset( key );
        config_->set( new_key, val );
      }
    }
  }

  properties_ = new_props;
}

void DisplayWrapper::loadProperties()
{
  if (!config_)
  {
    return;
  }

  properties_.clear();

  for( Config::Iterator ci = config_->begin(); ci != config_->end(); ci++ )
  {
    std::string name = (*ci).first;
    std::string value = (*ci).second;

    if( name.find( name_ + "." ) == 0 )
    {
      properties_[ name ] = value;
    }
  }
}

void DisplayWrapper::onDisplaysConfigLoaded(const boost::shared_ptr<Config>& config)
{
  config_ = config;

  loadProperties();
}

void DisplayWrapper::onDisplaysConfigSaved(const boost::shared_ptr<Config>& config)
{
  if (display_)
  {
    return;
  }

  M_string::iterator it = properties_.begin();
  M_string::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    const std::string& name = it->first;
    const std::string& value = it->second;
    config->set(name, value);
  }
}

bool DisplayWrapper::isLoaded() const
{
  return display_ != 0;
}

void DisplayWrapper::createDisplay()
{
  if( display_ )
  {
    return;
  }

  Q_EMIT displayCreating( this );

  try
  {
    display_ = class_loader_->createClassInstance( class_lookup_name_ );
  }
  catch( pluginlib::PluginlibException& ex )
  {
    ROS_ERROR( "The plugin for class '%s' failed to load.  Error: %s",
               class_lookup_name_.c_str(), ex.what() );
  }

  if (display_)
  {
    display_->initialize( name_, manager_ );
    if (property_manager_)
    {
      display_->setPropertyManager( property_manager_, category_ );
    }

    Q_EMIT displayCreated( this );
  }
}

void DisplayWrapper::destroyDisplay()
{
  if (display_)
  {
    Q_EMIT displayDestroying( this );

    display_->disable(false);
    delete display_;
    display_ = 0;

    Q_EMIT displayDestroyed( this );
  }
}

//void DisplayWrapper::onPluginLoaded(const PluginStatus& st)
//{
//  ROS_ASSERT(st.plugin == plugin_.get());
//  ROS_ASSERT(display_ == 0);
//
//  createDisplay();
//
//  if (display_)
//  {
//    display_->setEnabled(enabled_, true);
//  }
//}
//
//void DisplayWrapper::onPluginUnloading(const PluginStatus& st)
//{
//  ROS_ASSERT(st.plugin == plugin_.get());
//  ROS_ASSERT(display_ != 0);
//
//  loadProperties();
//  destroyDisplay();
//}

bool DisplayWrapper::isEnabled()
{
  if (display_)
  {
    return display_->isEnabled();
  }

  return enabled_;
}

void DisplayWrapper::setEnabled(bool enabled)
{
  if (display_)
  {
    display_->setEnabled(enabled, false);
  }

  enabled_ = enabled;

  propertyChanged(category_);
}

void DisplayWrapper::setPropertyManager(PropertyManager* property_manager, const CategoryPropertyWPtr& parent)
{
  ROS_ASSERT(!property_manager_);

  property_manager_ = property_manager;

  category_ = property_manager_->createCheckboxCategory( getName(), "Enabled", getName() + ".", boost::bind( &DisplayWrapper::isEnabled, this ),
                                                         boost::bind( &DisplayWrapper::setEnabled, this, _1 ), parent, this );

  std::string help_description = class_loader_->getClassDescription( class_lookup_name_ );
  setPropertyHelpText(category_, help_description);

  if (display_)
  {
    display_->setPropertyManager(property_manager, category_);
  }
}

std::string DisplayWrapper::getClassDisplayName() const
{
  return class_loader_->getName( class_lookup_name_ );
}

} // namespace rviz
