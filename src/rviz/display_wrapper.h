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

#ifndef RVIZ_DISPLAY_WRAPPER_H
#define RVIZ_DISPLAY_WRAPPER_H

#include <QObject>

#include <pluginlib/class_loader.h>

#include "properties/forwards.h"

namespace rviz
{

class Config;
class Display;
class VisualizationManager;
//struct PluginStatus;

class DisplayTypeInfo;
typedef boost::shared_ptr<DisplayTypeInfo> DisplayTypeInfoPtr;
class Plugin;
typedef boost::shared_ptr<Plugin> PluginPtr;

class DisplayWrapper: public QObject
{
Q_OBJECT
public:
  DisplayWrapper( const std::string& class_lookup_name,
                  pluginlib::ClassLoader<Display>* class_loader,
                  const std::string& name,
                  VisualizationManager* manager );
  ~DisplayWrapper();

  bool isLoaded() const;
  const DisplayTypeInfoPtr& getTypeInfo() const { return typeinfo_; }
  const PluginPtr& getPlugin() const { return plugin_; }
  Display* getDisplay() const { return display_; }
  const std::string& getName() const { return name_; }
  const std::string& getClassLookupName() const { return class_lookup_name_; }
  std::string getClassDisplayName() const;

  void setName(const std::string& name);

  void setEnabled(bool enabled);
  bool isEnabled();

  void setPlugin(const PluginPtr& plugin);
  void createDisplay();
  void destroyDisplay();

  void setPropertyManager(PropertyManager* property_manager, const CategoryPropertyWPtr& parent);
  const CategoryPropertyWPtr& getCategory() const { return category_; }

Q_SIGNALS:
  void displayCreating( DisplayWrapper* );
  void displayCreated( DisplayWrapper* );
  void displayDestroying( DisplayWrapper* );
  void displayDestroyed( DisplayWrapper* );

protected Q_SLOTS:
  void onDisplaysConfigLoaded( const boost::shared_ptr<Config>& config );
  void onDisplaysConfigSaved( const boost::shared_ptr<Config>& config );
//  void onPluginLoaded(const PluginStatus&);
//  void onPluginUnloading(const PluginStatus&);

protected:
  void loadProperties();

  VisualizationManager* manager_;

  pluginlib::ClassLoader<Display>* class_loader_;

  std::string name_;
  std::string class_lookup_name_;

  Display* display_;
  DisplayTypeInfoPtr typeinfo_;
  PluginPtr plugin_;

  boost::shared_ptr<Config> config_;

  typedef std::map<std::string, std::string> M_string;
  M_string properties_;

  PropertyManager* property_manager_;
  CategoryPropertyWPtr category_;

  bool enabled_;
};

} // namespace rviz

#endif // RVIZ_DISPLAY_WRAPPER_H
