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
#ifndef LABEL_PROPERTY_H
#define LABEL_PROPERTY_H

#include "rviz/properties/bool_property.h"

namespace rviz
{

class ColorProperty;
class IntProperty;
class LabelProperty;
class StringProperty;

class LabelsProperty : public Property
{
Q_OBJECT

public:
    LabelsProperty(
      const QString& name = QString(),
      const QVariant& value = QVariant(),
      const QString& description = QString(),
      Property* parent = 0,
      const char *changed_slot = 0,
      QObject* receiver = 0);

    virtual void addChild(Property* child, int index = -1);

    virtual void load(const Config& config);

    LabelProperty* add(int value, const QColor &color);
    bool remove(int value);

public Q_SLOTS:
    void updateFromChild();
    void emitAboutToChange();
};

class LabelProperty : public BoolProperty
{
Q_OBJECT
public:
    LabelProperty(
      const QString& name,
      bool enabled,
      int value,
      const QColor &color,
      Property* parent = 0,
      const char *changed_slot = 0,
      QObject* receiver = 0);

    virtual bool setInt(int value);
    virtual int getInt() const;

    virtual bool setString(const QString& name);
    virtual QString getString() const;

    virtual bool setColor(const QColor& color);
    virtual QColor getColor() const;

public Q_SLOTS:
    void updateFromChildren();
    void emitAboutToChange();

private:
    void updateName();

    StringProperty* alias_;
    IntProperty* value_;
    ColorProperty* color_;
};

}  // end namespace rviz

#endif  // LABEL_PROPERTY_H
