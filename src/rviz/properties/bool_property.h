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
#ifndef BOOL_PROPERTY_H
#define BOOL_PROPERTY_H

#include <rviz/properties/property.h>
#include <rviz/rviz_export.h>

namespace rviz
{
/** @brief Property specialized to provide getter for booleans. */
class RVIZ_EXPORT BoolProperty : public Property
{
  Q_OBJECT
public:
  BoolProperty(const QString& name = QString(),
               bool default_value = false,
               const QString& description = QString(),
               Property* parent = nullptr,
               const char* changed_slot = nullptr,
               QObject* receiver = nullptr);

  ~BoolProperty() override;

  virtual bool getBool() const;

  //* If this is true, will disable it's children when it's own bool value is false */
  void setDisableChildrenIfFalse(bool disable);

  bool getDisableChildrenIfFalse();

  //* Overridden from Property */
  bool getDisableChildren() override;

public Q_SLOTS:
  bool setBool(bool value)
  {
    return setValue(value);
  }

private:
  bool disable_children_if_false_;
};

} // end namespace rviz

#endif // BOOL_PROPERTY_H
