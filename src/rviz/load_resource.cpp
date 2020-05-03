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
#include <QPainter>

namespace rviz
{
boost::filesystem::path getPath(const QString& url)
{
  boost::filesystem::path path;

  if (url.indexOf("package://", 0, Qt::CaseInsensitive) == 0)
  {
    QString package_name = url.section('/', 2, 2);
    QString file_name = url.section('/', 3);
    path = ros::package::getPath(package_name.toStdString());
    path = path / file_name.toStdString();
  }
  else if (url.indexOf("file://", 0, Qt::CaseInsensitive) == 0)
  {
    path = url.section('/', 2).toStdString();
  }
  else
  {
    ROS_ERROR("Invalid or unsupported URL: '%s'", url.toStdString().c_str());
  }

  return path;
}


QPixmap loadPixmap(const QString& url, bool fill_cache)
{
  QPixmap pixmap;

  // if it's in the cache, no need to locate
  if (QPixmapCache::find(url, &pixmap))
  {
    return pixmap;
  }

  boost::filesystem::path path = getPath(url);

  // If something goes wrong here, we go on and store the empty pixmap,
  // so the error won't appear again anytime soon.
  if (boost::filesystem::exists(path))
  {
    ROS_DEBUG_NAMED("load_resource", "Loading '%s'", path.string().c_str());
    if (!pixmap.load(QString::fromStdString(path.string())))
    {
      ROS_ERROR("Could not load pixmap '%s'", path.string().c_str());
    }
  }

  if (fill_cache)
  {
    QPixmapCache::insert(url, pixmap);
  }

  return pixmap;
}

QCursor getDefaultCursor(bool /*fill_cache*/)
{
  return QCursor(Qt::ArrowCursor);
}

QCursor makeIconCursor(const QString& url, bool fill_cache)
{
  QPixmap icon = loadPixmap(url, fill_cache);
  if (icon.width() == 0 || icon.height() == 0)
  {
    ROS_ERROR("Could not load pixmap '%s' -- using default cursor instead.", url.toStdString().c_str());
    return getDefaultCursor();
  }
  QString cache_key = url + ".cursor";
  return makeIconCursor(icon, cache_key, fill_cache);
}

QCursor makeIconCursor(const QPixmap& icon, const QString& cache_key, bool fill_cache)
{
  // if it's in the cache, no need to locate
  QPixmap cursor_img;
  if (QPixmapCache::find(cache_key, &cursor_img))
  {
    return QCursor(cursor_img, 0, 0);
  }

  QPixmap base_cursor = loadPixmap("package://rviz/icons/cursor.svg", fill_cache);

  const int cursor_size = 32;

  cursor_img = QPixmap(cursor_size, cursor_size);
  cursor_img.fill(QColor(0, 0, 0, 0));

  // copy base cursor & icon into one image
  QPainter painter(&cursor_img);

  int draw_x = 12;
  int draw_y = 16;

  // if the icon is too large, move it to the left
  if (draw_x + icon.width() > cursor_size)
  {
    draw_x = cursor_size - icon.width();
  }
  if (draw_y + icon.height() > cursor_size)
  {
    draw_y = cursor_size - icon.height();
  }

  painter.drawPixmap(0, 0, base_cursor);
  painter.drawPixmap(draw_x, draw_y, icon);

  if (fill_cache)
  {
    QPixmapCache::insert(cache_key, cursor_img);
  }

  return QCursor(cursor_img, 1, 1);
}


} // namespace rviz
