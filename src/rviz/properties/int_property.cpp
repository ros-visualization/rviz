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

#include <limits.h> // for INT_MAX and INT_MIN

#include <QtGlobal>
#include <QSpinBox>

#include "rviz/properties/int_property.h"

namespace rviz
{
IntProperty::IntProperty(const QString& name,
                         int default_value,
                         const QString& description,
                         Property* parent,
                         const char* changed_slot,
                         QObject* receiver)
  : Property(name, default_value, description, parent, changed_slot, receiver)
  , min_(INT_MIN)
  , max_(INT_MAX)
{
}

bool IntProperty::setValue(const QVariant& new_value)
{
  return Property::setValue(qBound(min_, new_value.toInt(), max_));
}

void IntProperty::setMin(int min)
{
  min_ = min;
  setValue(getValue());
}

void IntProperty::setMax(int max)
{
  max_ = max;
  setValue(getValue());
}

QWidget* IntProperty::createEditor(QWidget* parent, const QStyleOptionViewItem& /*option*/)
{
  QSpinBox* editor = new QSpinBox(parent);
  editor->setFrame(false);
  editor->setRange(min_, max_);
  connect(editor, SIGNAL(valueChanged(int)), this, SLOT(setInt(int)));
  return editor;
}

} // end namespace rviz
