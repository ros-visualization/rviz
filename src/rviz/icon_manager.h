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

#ifndef ICON_MANAGER_H_
#define ICON_MANAGER_H_

#include <boost/thread/mutex.hpp>

#include <QIcon>

namespace rviz
{

/*
 * Lets you retrieve icons based on the name of a package and file.
 * Icons are internally cached, so retrieval is fast after they've
 * been accessed once
 */
class IconManager
{
public:
  IconManager();
  virtual
  ~IconManager();

  // Returns the icon corresponding to the file <package path>/icons/icon_path
  // and updates the cache.
  // To determine if the icon was actually found, use QIcon::isNull()
  QIcon getIcon( std::string package_name, std::string icon_path );

  // Returns the icon corresponding to the file <package path>/icons/icon_path.
  // Does not insert new elements into the cache.
  // To determine if the icon was actually found, use QIcon::isNull()
  QIcon getIcon( std::string package_name, std::string icon_path ) const;

private:

  std::map< std::string, std::string > package_path_cache_;
  std::map< std::string, QIcon > icon_cache_;
  mutable boost::mutex cache_mutex_;
};

}

#endif /* ICON_MANAGER_H_ */
