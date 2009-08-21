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
#include "plugin/display_type_info.h"
#include "plugin/plugin.h"
#include "plugin/display_creator.h"
#include "properties/property_manager.h"
#include "properties/property.h"

#include <ros/assert.h>

#include <wx/confbase.h>

namespace rviz
{

DisplayWrapper::DisplayWrapper(const std::string& package, const std::string& class_name, const PluginPtr& plugin, const std::string& name, VisualizationManager* manager)
: manager_(manager)
, name_(name)
, package_(package)
, class_name_(class_name)
, display_(0)
, property_manager_(0)
{
  manager->getDisplaysConfigLoadedSignal().connect(boost::bind(&DisplayWrapper::onDisplaysConfigLoaded, this, _1));
  manager->getDisplaysConfigSavingSignal().connect(boost::bind(&DisplayWrapper::onDisplaysConfigSaved, this, _1));

  if (plugin)
  {
    setPlugin(plugin);
  }
}

DisplayWrapper::~DisplayWrapper()
{
  destroyDisplay();

  if (property_manager_)
  {
    property_manager_->deleteProperty(category_.lock());
  }
}

void DisplayWrapper::setPlugin(const PluginPtr& plugin)
{
  ROS_ASSERT(!plugin_);

  plugin_ = plugin;

  if (plugin_)
  {
    plugin_->getLoadedSignal().connect(boost::bind(&DisplayWrapper::onPluginLoaded, this, _1));
    plugin_->getUnloadingSignal().connect(boost::bind(&DisplayWrapper::onPluginUnloading, this, _1));
  }

  plugin_->autoLoad();
  typeinfo_ = plugin_->getDisplayTypeInfo(class_name_);
}

void DisplayWrapper::loadProperties()
{
  if (!config_)
  {
    return;
  }

  properties_.clear();

  wxString name;
  long index;
  bool cont = config_->GetFirstEntry(name, index);
  while (cont)
  {
    wxString value;
    config_->Read(name, &value);

    if (name.StartsWith(wxString::FromAscii((name_ + ".").c_str())))
    {
      properties_[(const char*)name.mb_str()] = (const char*)value.mb_str();
    }

    cont = config_->GetNextEntry(name, index);
  }
}

void DisplayWrapper::onDisplaysConfigLoaded(const boost::shared_ptr<wxConfigBase>& config)
{
  config_ = config;

  loadProperties();
}

void DisplayWrapper::onDisplaysConfigSaved(const boost::shared_ptr<wxConfigBase>& config)
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
    config->Write(wxString::FromAscii(name.c_str()), wxString::FromAscii(value.c_str()));
  }
}

bool DisplayWrapper::isLoaded() const
{
  return display_ != 0;
}

void DisplayWrapper::createDisplay()
{
  if (!typeinfo_ || !typeinfo_->creator)
  {
    return;
  }

  if (display_)
  {
    return;
  }

  display_creating_(this);

  display_ = plugin_->createDisplay(class_name_, name_, manager_);
  if (property_manager_)
  {
    display_->setPropertyManager(property_manager_, category_);
  }

  if (display_)
  {
    display_created_(this);
  }
}

void DisplayWrapper::destroyDisplay()
{
  if (display_)
  {
    display_destroying_(this);

    display_->disable(false);
    delete display_;
    display_ = 0;

    display_destroyed_(this);
  }
}

void DisplayWrapper::onPluginLoaded(const PluginStatus& st)
{
  ROS_ASSERT(st.plugin == plugin_.get());
  ROS_ASSERT(display_ == 0);

  createDisplay();
}

void DisplayWrapper::onPluginUnloading(const PluginStatus& st)
{
  ROS_ASSERT(st.plugin == plugin_.get());
  ROS_ASSERT(display_ != 0);

  loadProperties();
  destroyDisplay();
}

void DisplayWrapper::setPropertyManager(PropertyManager* property_manager, const CategoryPropertyWPtr& parent)
{
  ROS_ASSERT(!property_manager_);

  property_manager_ = property_manager;

  category_ = property_manager_->createCategory( getName(), "", parent );
  CategoryPropertyPtr cat_prop = category_.lock();
  cat_prop->setUserData(this);

  if (display_)
  {
    display_->setPropertyManager(property_manager, category_);
  }
}

} // namespace rviz
