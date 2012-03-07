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

#include "property_manager.h"
#include "property.h"
#include "rviz/properties/property_tree_widget.h"

#include <ros/console.h>

#include "config.h"

namespace rviz
{

PropertyManager::PropertyManager()
: grid_(0)
, default_user_data_(0)
{
}

PropertyManager::~PropertyManager()
{
  clear();
}

void PropertyManager::addProperty(const PropertyBasePtr& property, const std::string& name, const std::string& prefix, void* user_data)
{
  bool inserted = properties_.insert( std::make_pair( std::make_pair(prefix, name), property ) ).second;
  ROS_ASSERT(inserted);

  if (!user_data)
  {
    user_data = default_user_data_;
  }

  property->setUserData( user_data );

  // "connect" the property's changed() callback to this->propertySet().
  property->manager_ = this;

  if (config_ && property->getSave())
  {
    property->loadFromConfig(config_.get());
  }

  if (grid_)
  {
    property->setPropertyTreeWidget(grid_);
    property->writeToGrid();
  }

  if( property->getSave() )
  {
    Q_EMIT configChanged();
  }
}

StatusPropertyWPtr PropertyManager::createStatus(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data)
{
  StatusPropertyPtr prop(new StatusProperty(name, prefix, parent, user_data));
  addProperty(prop, name, prefix, user_data);

  return StatusPropertyWPtr(prop);
}

CategoryPropertyWPtr PropertyManager::createCategory(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data)
{
  CategoryPropertyPtr category(new CategoryProperty(name, name, prefix, parent, CategoryProperty::Getter(), CategoryProperty::Setter(), false));
  category->setSave( false );
  addProperty(category, name, prefix, user_data);

  return CategoryPropertyWPtr(category);
}

CategoryPropertyWPtr PropertyManager::createCheckboxCategory(const std::string& label, const std::string& name, const std::string& prefix, const boost::function<bool(void)>& getter,
                                                             const boost::function<void(bool)>& setter, const CategoryPropertyWPtr& parent, void* user_data)
{
  CategoryPropertyPtr category(new CategoryProperty(label, name, prefix, parent, getter, setter, true));
  addProperty(category, name, prefix, user_data);

  return CategoryPropertyWPtr(category);
}

void PropertyManager::update()
{
  S_PropertyBaseWPtr local_props;
  {
    boost::mutex::scoped_lock lock(changed_mutex_);

    local_props.swap(changed_properties_);
  }

  if (!local_props.empty())
  {
    S_PropertyBaseWPtr::iterator it = local_props.begin();
    S_PropertyBaseWPtr::iterator end = local_props.end();
    for (; it != end; ++it)
    {
      PropertyBasePtr property = it->lock();
      if (property)
      {
        if (grid_)
        {
          property->writeToGrid();
        }

        if (config_ && property->getSave())
        {
          property->saveToConfig(config_.get());
        }
      }
    }

    if (grid_)
    {
      grid_->update();
    }
  }
}

void PropertyManager::deleteProperty( const PropertyBasePtr& property )
{
  if ( !property )
  {
    return;
  }

  // "disconnect" from the property's changed() callback.
  property->manager_ = 0;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    if (it->second == property)
    {
      // search for any children of this property, and delete them as well
      deleteChildren( it->second );

      properties_.erase( it );

      break;
    }
  }
  if( property->getSave() )
  {
    Q_EMIT configChanged();
  }
}

void PropertyManager::deleteProperty( const std::string& name, const std::string& prefix )
{
  M_Property::iterator found_it = properties_.find( std::make_pair( prefix, name ) );
  ROS_ASSERT( found_it != properties_.end() );

  // search for any children of this property, and delete them as well
  deleteChildren( found_it->second );

  if( found_it->second )
  {
    // "disconnect" from the property's changed() callback.
    found_it->second->manager_ = 0;
  }

  properties_.erase( found_it );

  if( found_it->second->getSave() )
  {
    Q_EMIT configChanged();
  }
}

void PropertyManager::changePrefix(const std::string& old_prefix, const std::string& new_prefix)
{
  // This kind of sucks... because properties are split into name + prefix, can't just lookup based on the prefix
  // so we have to iterate through
  M_Property to_add;
  std::vector<M_Property::iterator> to_delete;
  bool savable_changed = false;
  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    const std::pair<std::string, std::string>& key = it->first;
    const PropertyBasePtr& prop = it->second;

    // We want to get everything that started with the old prefix, not just those that are an exact match
    size_t pos = key.first.find(old_prefix);
    if (pos == 0)
    {
      std::string np = new_prefix + key.first.substr(old_prefix.size());
      prop->setPrefix(np);
      to_add[std::make_pair(np, key.second)] = prop;
      to_delete.push_back(it);
      if( prop->getSave() )
      {
        savable_changed = true;
      }
    }
  }

  for (size_t i = 0; i < to_delete.size(); ++i)
  {
    properties_.erase(to_delete[i]);
  }

  properties_.insert(to_add.begin(), to_add.end());

  if( savable_changed )
  {
    Q_EMIT configChanged();
  }
}

void PropertyManager::deleteChildren( const PropertyBasePtr& property )
{
  if (!property)
  {
    return;
  }

  std::set<PropertyBasePtr> to_delete;

  M_Property::iterator prop_it = properties_.begin();
  M_Property::iterator prop_end = properties_.end();
  for ( ; prop_it != prop_end; ++prop_it )
  {
    const PropertyBasePtr& child = prop_it->second;

    PropertyBaseWPtr parent = child->getParent();
    if ( parent.lock() == property )
    {
      to_delete.insert( child );
    }
  }

  std::set<PropertyBasePtr>::iterator del_it = to_delete.begin();
  std::set<PropertyBasePtr>::iterator del_end = to_delete.end();
  for ( ; del_it != del_end; ++del_it )
  {
    deleteProperty( *del_it );
  }

  to_delete.clear();
}

void PropertyManager::deleteByUserData( void* user_data )
{
  std::set<PropertyBasePtr> to_delete;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for ( ; it != end; ++it )
  {
    const PropertyBasePtr& property = it->second;

    if ( property->getUserData() == user_data )
    {
      PropertyBasePtr parent = property->getParent().lock();
      if ( !parent || parent->getUserData() != user_data )
      {
        to_delete.insert( property );
      }
    }
  }

  std::set<PropertyBasePtr>::iterator prop_it = to_delete.begin();
  std::set<PropertyBasePtr>::iterator prop_end = to_delete.end();
  for ( ; prop_it != prop_end; ++prop_it )
  {
    deleteProperty( *prop_it );
  }
}

void PropertyManager::propertySet( const PropertyBasePtr& property )
{
  boost::mutex::scoped_lock lock(changed_mutex_);

  changed_properties_.insert(property);
}

void PropertyManager::emitConfigChanged()
{
  Q_EMIT configChanged();
}

void PropertyManager::save(const boost::shared_ptr<Config>& config)
{
  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for ( ; it != end; ++it )
  {
    const PropertyBasePtr& property = it->second;

    if ( property->getSave() )
    {
      property->saveToConfig( config.get() );
    }
  }
}

void PropertyManager::load(const boost::shared_ptr<Config>& config, const StatusCallback& cb)
{
  config_ = config;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for ( ; it != end; ++it )
  {
    const PropertyBasePtr& property = it->second;

    if ( property->getSave() )
    {
      std::stringstream ss;
      ss << "Loading property [" << property->getPrefix() + property->getName() << "]";
      ROS_DEBUG_STREAM_NAMED("properties", ss.str());

      if (cb)
      {
        cb(ss.str());
      }

      property->loadFromConfig( config.get() );
    }
  }

  if( grid_ )
  {
    grid_->update();
  }
}

void PropertyManager::setPropertyTreeWidget(PropertyTreeWidget* grid)
{
  ROS_ASSERT(!grid_);
  ROS_ASSERT(grid);

  grid_ = grid;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    const PropertyBasePtr& property = it->second;
    property->setPropertyTreeWidget(grid_);
    property->writeToGrid();
  }
}

void PropertyManager::refreshAll()
{
  ROS_ASSERT(grid_);

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    propertySet(it->second);
  }

  update();
}

void PropertyManager::clear()
{
  properties_.clear();
}

} // end namespace rviz
