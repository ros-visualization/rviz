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

#include <ros/console.h>

#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>

namespace rviz
{

PropertyManager::PropertyManager()
: grid_(NULL)
{
}

PropertyManager::~PropertyManager()
{
  clear();
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
  }
}

CategoryPropertyWPtr PropertyManager::createCategory(const std::string& name, const std::string& prefix, const CategoryPropertyWPtr& parent, void* user_data)
{
  CategoryPropertyWPtr category = createProperty<CategoryProperty>(name, prefix, CategoryProperty::Getter(), CategoryProperty::Setter(), parent, user_data);
  CategoryPropertyPtr category_real = category.lock();
  category_real->setSave( false );

  return category;
}

void PropertyManager::deleteProperty( const PropertyBasePtr& property )
{
  if ( !property )
  {
    return;
  }

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    if (it->second == property)
    {
      // search for any children of this property, and delete them as well
      deleteChildren( it->second );

      grid_->Freeze();

      properties_.erase( it );

      grid_->Thaw();

      break;
    }
  }
}

void PropertyManager::deleteProperty( const std::string& name, const std::string& prefix )
{
  M_Property::iterator found_it = properties_.find( std::make_pair( prefix, name ) );
  ROS_ASSERT( found_it != properties_.end() );

  // search for any children of this property, and delete them as well
  deleteChildren( found_it->second );

  if (grid_)
  {
    grid_->Freeze();
  }

  properties_.erase( found_it );

  if (grid_)
  {
    grid_->Thaw();
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

  if (grid_)
  {
    grid_->Freeze();
  }

  std::set<PropertyBasePtr>::iterator del_it = to_delete.begin();
  std::set<PropertyBasePtr>::iterator del_end = to_delete.end();
  for ( ; del_it != del_end; ++del_it )
  {
    deleteProperty( *del_it );
  }

  to_delete.clear();

  if (grid_)
  {
    grid_->Thaw();
  }
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

  if (grid_)
  {
    grid_->Freeze();
  }

  std::set<PropertyBasePtr>::iterator prop_it = to_delete.begin();
  std::set<PropertyBasePtr>::iterator prop_end = to_delete.end();
  for ( ; prop_it != prop_end; ++prop_it )
  {
    deleteProperty( *prop_it );
  }

  if (grid_)
  {
    grid_->Thaw();
  }
}

void PropertyManager::propertyChanging( wxPropertyGridEvent& event )
{

}

void PropertyManager::propertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  void* client_data = property->GetClientData();
  if ( client_data )
  {
    PropertyBase* property = reinterpret_cast<PropertyBase*>(client_data);

    property->readFromGrid();
  }
}

void PropertyManager::propertySet( const PropertyBasePtr& property )
{
  boost::mutex::scoped_lock lock(changed_mutex_);

  changed_properties_.insert(property);
}

void PropertyManager::save(const boost::shared_ptr<wxConfigBase>& config)
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

void PropertyManager::load(const boost::shared_ptr<wxConfigBase>& config)
{
  config_ = config;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for ( ; it != end; ++it )
  {
    const PropertyBasePtr& property = it->second;

    if ( property->getSave() )
    {
      ROS_DEBUG_NAMED("properties", "Loading property [%s]", (property->getPrefix() + property->getName()).c_str());
      property->loadFromConfig( config.get() );
    }
  }
}

void PropertyManager::setPropertyGrid(wxPropertyGrid* grid)
{
  ROS_ASSERT(!grid_);
  ROS_ASSERT(grid);

  grid_ = grid;

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    const PropertyBasePtr& property = it->second;
    property->setPropertyGrid(grid_);
    property->writeToGrid();
    property->setPGClientData();
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
  if (grid_)
  {
    grid_->Freeze();
  }

  M_Property::iterator it = properties_.begin();
  M_Property::iterator end = properties_.end();
  for (; it != end; ++it)
  {
    it->second->reset();
  }
  properties_.clear();

  if (grid_)
  {
    grid_->Clear();
    grid_->Thaw();
  }
}

}
