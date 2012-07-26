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

#include "load_resource.h"

#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <ros/ros.h>

#include <QPixmapCache>

namespace rviz
{

QPixmap loadPixmap( QString url, bool fill_cache )
{
  QPixmap pixmap;

  // if it's in the cache, no need to locate
  if ( QPixmapCache::find( url, &pixmap ) )
  {
    return pixmap;
  }

  boost::filesystem::path path;

  if ( url.indexOf("package://", 0, Qt::CaseInsensitive) == 0 )
  {
    QString package_name = url.section('/',2,2);
    QString file_name = url.section('/',3);
    path = ros::package::getPath(package_name.toStdString());
    path = path / file_name.toStdString();
  }
  else if ( url.indexOf("file://", 0, Qt::CaseInsensitive) == 0 )
  {
    path = url.section('/',2).toStdString();
  }
  else
  {
    ROS_ERROR( "Invalid or unsupported URL: '%s'", url.toStdString().c_str() );
  }

  // If something goes wrong here, we go on and store the empty pixmap,
  // so the error won't appear again anytime soon.
  if ( boost::filesystem::exists( path ) )
  {
    if ( !pixmap.load( QString::fromStdString( path.string() ) ) )
    {
      ROS_ERROR( "Could not load pixmap '%s'", path.c_str() );
    }
  }

  if ( fill_cache )
  {
    QPixmapCache::insert( url, pixmap );
  }

  return pixmap;
}


}
