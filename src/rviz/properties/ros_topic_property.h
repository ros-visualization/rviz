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
#ifndef ROS_TOPIC_PROPERTY_H
#define ROS_TOPIC_PROPERTY_H

#include <string>

#include <rviz/properties/editable_enum_property.h>
#include <rviz/rviz_export.h>

namespace rviz
{
class RVIZ_EXPORT RosTopicProperty : public EditableEnumProperty
{
  Q_OBJECT
public:
  RosTopicProperty(const QString& name = QString(),
                   const QString& default_value = QString(),
                   const QString& message_type = QString(),
                   const QString& description = QString(),
                   Property* parent = nullptr);

  template <typename Func, typename R>
  RosTopicProperty(const QString& name,
                   const QString& default_value,
                   const QString& message_type,
                   const QString& description,
                   Property* parent,
                   Func&& changed_slot,
                   const R* receiver)
    : RosTopicProperty(name, default_value, message_type, description, parent)
  {
    connect(receiver, std::forward<Func>(changed_slot));
  }

  // this variant is required to allow omitting the receiver argument
  template <typename Func, typename P>
  RosTopicProperty(const QString& name,
                   const QString& default_value,
                   const QString& message_type,
                   const QString& description,
                   P* parent,
                   Func&& changed_slot)
    : RosTopicProperty(name, default_value, message_type, description, parent)
  {
    connect(parent, std::forward<Func>(changed_slot));
  }

  void setMessageType(const QString& message_type);
  QString getMessageType() const
  {
    return message_type_;
  }

  QString getTopic() const
  {
    return getValue().toString();
  }
  std::string getTopicStd() const
  {
    return getValue().toString().toStdString();
  }

protected Q_SLOTS:
  virtual void fillTopicList();

private:
  QString message_type_;
};

} // end namespace rviz

#endif // ROS_TOPIC_PROPERTY_H
