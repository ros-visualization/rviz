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

#include "plugin_manager.h"
#include "plugin.h"

#include <ros/console.h>
#include <ros/package.h>

#include <wx/confbase.h>

namespace rviz
{

PluginManager::PluginManager()
{

}

PluginManager::~PluginManager()
{

}

void PluginManager::loadDescriptions()
{
  ros::package::V_string plugins;
  ros::package::getPlugins("rviz", "plugin", plugins);

  ros::package::V_string::iterator it = plugins.begin();
  ros::package::V_string::iterator end = plugins.end();
  for (; it != end; ++it)
  {
    const std::string& plugin = *it;
    loadDescription(plugin);
  }
}

PluginPtr PluginManager::getPlugin(const std::string& description_file)
{
  L_Plugin::iterator it = plugins_.begin();
  L_Plugin::iterator end = plugins_.end();
  for (; it != end; ++it)
  {
    const PluginPtr& plugin = *it;

    if (plugin->getDescriptionPath() == description_file)
    {
      return plugin;
    }
  }

  return PluginPtr();
}

PluginPtr PluginManager::getPluginByPackage(const std::string& package)
{
  L_Plugin::iterator it = plugins_.begin();
  L_Plugin::iterator end = plugins_.end();
  for (; it != end; ++it)
  {
    const PluginPtr& plugin = *it;

    if (plugin->getPackageName() == package)
    {
      return plugin;
    }
  }

  return PluginPtr();
}

PluginPtr PluginManager::getPluginByDisplayName(const std::string& display_name)
{
  L_Plugin::iterator it = plugins_.begin();
  L_Plugin::iterator end = plugins_.end();
  for (; it != end; ++it)
  {
    const PluginPtr& plugin = *it;

    if (plugin->getDisplayTypeInfoByDisplayName(display_name))
    {
      return plugin;
    }
  }

  return PluginPtr();
}

PluginPtr PluginManager::loadDescription(const std::string& description_path)
{
  PluginPtr plugin = getPlugin(description_path);
  if (plugin)
  {
    return plugin;
  }

  plugin.reset(new Plugin);

  try
  {
    plugin->loadDescription(description_path);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("Error loading plugin from description [%s]: %s", description_path.c_str(), e.what());
  }

  plugins_.push_back(plugin);

  return plugin;
}

void PluginManager::loadConfig(const boost::shared_ptr<wxConfigBase>& config)
{
  int i = 0;
  while (1)
  {
    wxString package_key, autoload_key;
    package_key.Printf( wxT("Plugins/Plugin%d/Package"), i );
    autoload_key.Printf( wxT("Plugins/Plugin%d/AutoLoad"), i );

    wxString package;
    bool autoload;

    if (!config->Read(package_key, &package))
    {
      break;
    }

    if (!config->Read(autoload_key, &autoload))
    {
      break;
    }

    PluginPtr plugin = getPluginByPackage((const char*)package.char_str());
    if (plugin)
    {
      plugin->setAutoLoad(autoload);
      plugin->autoLoad();
    }

    ++i;
  }
}

void PluginManager::saveConfig(const boost::shared_ptr<wxConfigBase>& config)
{
  int i = 0;
  L_Plugin::iterator it = plugins_.begin();
  L_Plugin::iterator end = plugins_.end();
  for (; it != end; ++it, ++i)
  {
    const PluginPtr& plugin = *it;

    wxString package_key, autoload_key;
    package_key.Printf( wxT("Plugins/Plugin%d/Package"), i );
    autoload_key.Printf( wxT("Plugins/Plugin%d/AutoLoad"), i );

    config->Write(package_key, wxString::FromAscii(plugin->getPackageName().c_str()));
    config->Write(autoload_key, plugin->isAutoLoad());
  }
}

}
