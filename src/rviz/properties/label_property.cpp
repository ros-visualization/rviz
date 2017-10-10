/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/string_property.h"

#include "label_property.h"

namespace rviz
{

LabelsProperty::LabelsProperty(
  const QString& name,
  const QVariant& value,
  const QString& description,
  Property* parent,
  const char *changed_slot,
  QObject* receiver) :
  Property(name, value, description, parent, changed_slot, receiver)
{
}

void LabelsProperty::addChild(Property* child, int index)
{
  Property::addChild(child, index);
  connect(child, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
  connect(child, SIGNAL(changed()), this, SLOT(updateFromChild()));
}

void LabelsProperty::load(const Config& config)
{
  if (config.getType() == Config::Map)
  {
    // HACK: prevent signals from partially constructed children
    bool prev = blockSignals(true);

    // HACK: must create children here or Property::load does not load
    Config::MapIterator iter = config.mapIterator();
    while (iter.isValid())
    {
      LabelProperty* label = new LabelProperty(iter.currentKey(), true, 0, Qt::white, this);
      iter.advance();
    }

    blockSignals(prev);
  }
  Property::load(config);
}

LabelProperty* LabelsProperty::add(int value, const QColor &color)
{
    std::stringstream ss;
    ss << value;
    QString name = QString::fromStdString(ss.str());
    LabelProperty *label = new LabelProperty(name, true, value, color, this);
    return label;
}

bool LabelsProperty::remove(int value)
{
  const int num_children = numChildren();
  for (int i = 0; i != num_children; i += 1)
  {
    LabelProperty *label = reinterpret_cast<LabelProperty *>(childAt(i));
    if (label->getInt() == value)
    {
      return true;
    }
    removeChildren(i);
  }
  return false;
}

void LabelsProperty::updateFromChild()
{
  Q_EMIT changed();
}

void LabelsProperty::emitAboutToChange()
{
  Q_EMIT aboutToChange();
}

LabelProperty::LabelProperty(
  const QString& name,
  bool enabled,
  int value,
  const QColor &color,
  Property* parent,
  const char *changed_slot,
  QObject* receiver) :
    BoolProperty(name, enabled, "", parent, changed_slot, receiver)
{
    alias_ = new StringProperty("Name", name, "Alias assigned to this label.", this);
    value_ = new IntProperty("Label", value, "Value of this label.", this);
    color_ = new ColorProperty("Color", color, "Color to assign to every point with this label.", this);
    connect(alias_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
    connect(alias_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
    connect(value_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
    connect(value_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
    connect(color_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
    connect(color_, SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
}

bool LabelProperty::setInt(int value)
{
  value_->setInt(value);
  Q_EMIT changed();
  return true;
}

int LabelProperty::getInt() const
{
  return value_->getInt();
}

bool LabelProperty::setString(const QString& alias)
{
  alias_->setString(alias);
  Q_EMIT changed();
  return true;
}

QString LabelProperty::getString() const
{
  return alias_->getString();
}

bool LabelProperty::setColor(const QColor& color)
{
  color_->setColor(color);
  Q_EMIT changed();
  return true;
}

QColor LabelProperty::getColor() const
{
  return color_->getColor();
}

void LabelProperty::updateFromChildren()
{
  if (getString() != getName()) {
      setName(getString());
  }
  Q_EMIT changed();
}

void LabelProperty::emitAboutToChange()
{
  Q_EMIT aboutToChange();
}

} // end namespace rviz
