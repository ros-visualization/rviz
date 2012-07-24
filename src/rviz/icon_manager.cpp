/*
 * icon_manager.cpp
 *
 *  Created on: Jul 24, 2012
 *      Author: gossow
 */

#include "icon_manager.h"

#include <boost/filesystem.hpp>
#include <ros/package.h>

namespace rviz
{


IconManager::IconManager()
{
}

IconManager::~IconManager()
{
}

QIcon IconManager::getIcon( std::string package_name, std::string icon_path )
{
  boost::mutex::scoped_lock lock(cache_mutex_);

  std::string cache_key = package_name+"/"+icon_path;

  // look up icon in cache
  std::map< std::string, QIcon >::iterator it = icon_cache_.find( cache_key );
  if ( it != icon_cache_.end() )
  {
    return it->second;
  }

  std::map< std::string, std::string >::iterator path_it = package_path_cache_.find( package_name );
  if ( path_it == package_path_cache_.end() )
  {
    path_it = package_path_cache_.insert( std::make_pair( package_name, ros::package::getPath(package_name) ) ).first;
  }

  boost::filesystem::path path = path_it->second;
  path = path / "icons" / icon_path;
  return icon_cache_[cache_key] = QIcon( QString::fromStdString( path.string() ) );
}

QIcon IconManager::getIcon( std::string package_name, std::string icon_path ) const
{
  boost::mutex::scoped_lock lock(cache_mutex_);

  std::string cache_key = package_name+"/"+icon_path;

  // look up icon in cache
  std::map< std::string, QIcon >::const_iterator it = icon_cache_.find( cache_key );
  if ( it != icon_cache_.end() )
  {
    return it->second;
  }

  boost::filesystem::path path;
  std::map< std::string, std::string >::const_iterator path_it = package_path_cache_.find( package_name );
  if ( path_it == package_path_cache_.end() )
  {
    path = ros::package::getPath(package_name);
  }
  else
  {
    path = path_it->second;
  }

  path = path / "icons" / icon_path;
  return QIcon( QString::fromStdString( path.string() ) );
}

}

