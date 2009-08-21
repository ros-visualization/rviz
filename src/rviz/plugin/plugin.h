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

#ifndef RVIZ_PLUGIN_H
#define RVIZ_PLUGIN_H

#include "display_type_info.h"

#include <string>
#include <list>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/signals.hpp>

#include <wx/dynlib.h>

namespace rviz
{
class VisualizationManager;
class Display;
class DisplayCreator;
typedef boost::shared_ptr<DisplayCreator> DisplayCreatorPtr;

class Plugin;
struct PluginStatus
{
  PluginStatus(Plugin* p)
  : plugin(p)
  {}
  Plugin* plugin;
};
typedef boost::signal<void(const PluginStatus&)> PluginStatusSignal;

class PluginParseException : public std::runtime_error
{
public:
  PluginParseException(const std::string& file, const std::string& error)
  : std::runtime_error("Unable to parse plugin description file [" + file + "]: " + error)
  {}
};

class UnableToLoadLibraryException : public std::runtime_error
{
public:
  UnableToLoadLibraryException(const std::string& library_name)
  : std::runtime_error("Unable to load library [" + library_name + "]")
  {}
};

class NoPluginInitFunctionException : public std::runtime_error
{
public:
  NoPluginInitFunctionException(const std::string& library_name)
  : std::runtime_error("Library [" + library_name + "] does not have an rvizPluginInit(rviz::TypeRegistry*) function!")
  {}
};

class Plugin
{
public:
  Plugin();
  ~Plugin();

  void loadDescription(const std::string& description_path);

  void load();
  void unload();

  bool isLoaded();
  bool isAutoLoad();
  void setAutoLoad(bool autoload);
  void autoLoad();

  const L_DisplayTypeInfo& getDisplayTypeInfoList() const { return display_info_; }
  DisplayTypeInfoPtr getDisplayTypeInfo(const std::string& class_name) const;
  DisplayTypeInfoPtr getDisplayTypeInfoByDisplayName(const std::string& display_name) const;

  Display* createDisplay(const std::string& class_name, const std::string& name, VisualizationManager* manager);

  const std::string& getPackageName() const { return package_name_; }
  const std::string& getDescriptionPath() const { return description_path_; }
  const std::string& getName() { return name_; }

  PluginStatusSignal& getLoadingSignal() { return loading_signal_; }
  PluginStatusSignal& getLoadedSignal() { return loaded_signal_; }
  PluginStatusSignal& getUnloadingSignal() { return unloading_signal_; }
  PluginStatusSignal& getUnloadedSignal() { return unloaded_signal_; }

private:
  std::string description_path_;
  std::string package_name_;
  std::string library_path_;
  std::string name_;

  L_DisplayTypeInfo display_info_;

  wxDynamicLibrary library_;

  bool loaded_;
  bool auto_load_;
  bool auto_load_tried_;

  PluginStatusSignal loading_signal_;
  PluginStatusSignal loaded_signal_;
  PluginStatusSignal unloading_signal_;
  PluginStatusSignal unloaded_signal_;
};
typedef boost::shared_ptr<Plugin> PluginPtr;
typedef std::list<PluginPtr> L_Plugin;

}

#endif // RVIZ_PLUGIN_MANAGER_H

