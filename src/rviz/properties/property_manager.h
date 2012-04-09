/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_PROPERTY_MANAGER_H
#define RVIZ_PROPERTY_MANAGER_H

#include <QObject>

#include "forwards.h"
#include "ros/assert.h"

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <map>
#include <set>

namespace rviz
{

class Config;
class PropertyTreeWidget;
class PropertyBase;
class CategoryProperty;

/**
 * \brief Manages a set of properties
 *
 * The PropertyManager manages a set of properties, allowing you to create, delete, and load/save them from disk
 */
class PropertyManager: public QObject
{
Q_OBJECT
public:
  /**
   * \brief Constructor
   * @param grid The property grid to be associated with these properties
   */
  PropertyManager();
  /**
   * \brief Destructor
   */
  ~PropertyManager();

  /**
   * \brief Create a property.
   * @param name Name of the property (eg, "Color")
   * @param prefix Prefix for the property (eg, "Head Laser Scan")
   * @param getter Getter object (see boost::function and boost::bind).  Must be compatible with Property<T>::Getter
   * @param setter Setter object (see boost::function and boost::bind).  Must be compatible with Property<T>::Setter
   * @param parent Parent category
   * @param user_data User data to associate with the created property
   * @return The new property
   */
  template<typename T, typename G, typename S>
  boost::weak_ptr<T> createProperty(const std::string& name, const std::string& prefix, const G& getter, const S& setter, const CategoryPropertyWPtr& parent, void* user_data = NULL)
  {
    boost::shared_ptr<T> property(new T( name, prefix, parent, getter, setter ));
    addProperty(property, name, prefix, user_data);

    return boost::weak_ptr<T>(property);
  }

  void addProperty(const PropertyBasePtr& property, const std::string& name, const std::string& prefix, void* user_data);

  /**
   * \brief Create a category property
   * @param name Name of the category
   * @param parent Parent category (may be NULL)
   * @return The new category property
   */
  CategoryPropertyWPtr createCategory(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent = CategoryPropertyWPtr(), void* user_data = NULL);

  CategoryPropertyWPtr createCheckboxCategory(const std::string& label, const std::string& name, const std::string& prefix, const boost::function<bool(void)>& getter,
                                              const boost::function<void(bool)>& setter, const CategoryPropertyWPtr& parent = CategoryPropertyWPtr(), void* user_data = NULL);

  StatusPropertyWPtr createStatus(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent = CategoryPropertyWPtr(), void* user_data = NULL);

  bool hasProperty(const std::string& name, const std::string& prefix) { return properties_.find(std::make_pair(prefix, name)) != properties_.end(); }

  /**
   * \brief Delete a property
   * @param property The property to delete
   */
  void deleteProperty( const PropertyBasePtr& property );
  /**
   * \brief Delete a property, by name/prefix
   * @param name Name of the property
   * @param prefix Prefix of the property
   */
  void deleteProperty( const std::string& name, const std::string& prefix );
  /**
   * \brief Delete all properties that have a given user data
   * @param user_data The user data to compare against
   */
  void deleteByUserData( void* user_data );
  /**
   * \brief Delete all the children of a property
   * @param property The property whose children to delete
   */
  void deleteChildren( const PropertyBasePtr& property );

  /**
   * \brief Called when a property has been set (ie, Property::changed() has been called)
   * @param property The property that was set
   */
  void propertySet( const PropertyBasePtr& property );

  /**
   * \brief Save all properties into a Config
   * @param config The config to save to
   */
  void save(const boost::shared_ptr<Config>& config);
  /**
   * \brief Load all existing properties' values from a Config
   * @param config The config to load from
   */
  void load(const boost::shared_ptr<Config>& config, const StatusCallback& cb = StatusCallback());

  /**
   * \brief Get the property grid used by this manager
   * @return A pointer to the property grid
   */
  PropertyTreeWidget* getPropertyTreeWidget() { return grid_; }

  /**
   * \brief rename a property
   */
  void changePrefix(const std::string& old_prefix, const std::string& new_prefix);

  void setPropertyTreeWidget(PropertyTreeWidget* grid);

  void setDefaultUserData(void* data) { default_user_data_ = data; }

  void refreshAll();
  void clear();
  void update();

  /** @brief Emit the configChanged() signal. */
  void emitConfigChanged();

Q_SIGNALS:
  /** @brief Emitted when changes occur which would show up in a config file. */
  void configChanged();

protected:
  PropertyTreeWidget* grid_;        //< The property grid associated with our properties

  typedef std::map< std::pair<std::string, std::string>, PropertyBasePtr > M_Property;
  M_Property properties_;       //< The properties, mapped by name + prefix

  boost::mutex changed_mutex_;
  typedef std::set<PropertyBaseWPtr> S_PropertyBaseWPtr;
  S_PropertyBaseWPtr changed_properties_;

  void* default_user_data_;

  boost::shared_ptr<Config> config_;
};

} // namespace rviz

#endif
