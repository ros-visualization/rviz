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
#ifndef PLUGINLIB_FACTORY_H
#define PLUGINLIB_FACTORY_H

#include <QString>
#include <QList>

#include <string>
#include <vector>

#include <pluginlib/class_loader.h>

#include "rviz/class_id_recording_factory.h"

namespace rviz
{

template<class Type>
class PluginlibFactory: public ClassIdRecordingFactory<Type>
{
public:
  PluginlibFactory( const QString& package, const QString& base_class_type )
    {
      class_loader_ = new pluginlib::ClassLoader<Type>( package.toStdString(), base_class_type.toStdString() );
    }
  virtual ~PluginlibFactory()
    {
      delete class_loader_;
    }

  virtual QList<QString> getDeclaredClassIds()
    {
      QList<QString> ids;
      std::vector<std::string> std_ids = class_loader_->getDeclaredClasses();
      for( size_t i = 0; i < std_ids.size(); i++ )
      {
        ids.push_back( QString::fromStdString( std_ids[ i ]));
      }
      return ids;
    }

  virtual QString getClassDescription( const QString& class_id ) const
    {
      return QString::fromStdString( class_loader_->getClassDescription( class_id.toStdString() ));
    }

  virtual QString getClassName( const QString& class_id ) const
    {
      return QString::fromStdString( class_loader_->getName( class_id.toStdString() ));
    }

  virtual QString getClassPackage( const QString& class_id ) const
    {
      return QString::fromStdString( class_loader_->getClassPackage( class_id.toStdString() ));
    }

protected:
  /** @brief Instantiate and return a instance of a subclass of Type using our
   * pluginlib::ClassLoader. */
  virtual Type* makeRaw( const QString& class_id )
    {
      return class_loader_->createUnmanagedInstance( class_id.toStdString() );
    }

private:
  pluginlib::ClassLoader<Type>* class_loader_;  
};

} // end namespace rviz

#endif // PLUGINLIB_FACTORY_H
