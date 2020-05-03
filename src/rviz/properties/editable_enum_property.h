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
#ifndef EDITABLE_ENUM_PROPERTY_H
#define EDITABLE_ENUM_PROPERTY_H

#include <QStringList>

#include <rviz/properties/string_property.h>

namespace rviz
{
/** @brief Editable Enum property.
 *
 * An editable enum property works like a string property, but with
 * the addition of a drop-down list of predefined choices.
 */
class EditableEnumProperty : public StringProperty
{
  Q_OBJECT
public:
  EditableEnumProperty(const QString& name = QString(),
                       const QString& default_value = QString(),
                       const QString& description = QString(),
                       Property* parent = nullptr,
                       const char* changed_slot = nullptr,
                       QObject* receiver = nullptr);

  virtual void clearOptions();
  virtual void addOption(const QString& option);
  void addOptionStd(const std::string& option)
  {
    addOption(QString::fromStdString(option));
  }

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option) override;

  /** @brief Sort the option strings. */
  void sortOptions()
  {
    strings_.sort();
  }

public Q_SLOTS:
  virtual void setString(const QString& str);

Q_SIGNALS:
  /** @brief requestOptions() is emitted each time createEditor() is
   * called.
   *
   * A connection to this signal should never be made with a queued
   * connection, because then the "emit" would return before the
   * changes to the options in the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or
   * addOption() as needed.  The option list in the EditableEnumProperty will
   * not be cleared before the signal is emitted. */
  void requestOptions(EditableEnumProperty* property_in_need_of_options);

protected:
  QStringList strings_;
};

} // end namespace rviz

#endif // EDITABLE_ENUM_PROPERTY_H
