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
#ifndef ENUM_PROPERTY_H
#define ENUM_PROPERTY_H

#include <QStringList>

#include "rviz/properties/string_property.h"

namespace rviz
{

/** @brief Enum property.
 *
 * An enum property works like a string property all the way through
 * the system property system, except when you get a changed() signal
 * you can call getOptionInt() to get the integer value of the current
 * option.  The integer returned will be that passed to addOption()
 * for with the string that is currently selected.
 */
class EnumProperty: public StringProperty
{
Q_OBJECT
public:
  EnumProperty( const QString& name = QString(),
                const QString& default_value = QString(),
                const QString& description = QString(),
                Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual void clearOptions();
  virtual void addOption( const QString& option, int value = 0 );

  /** @brief Return the int value of the currently-chosen option, or 0
   * if the current option string does not have an int value. */
  virtual int getOptionInt();

  virtual QWidget* createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option );

public Q_SLOTS:
  virtual void setString( const QString& str );

Q_SIGNALS:
  /** @brief requestOptions() is emitted each time createEditor() is
   * called.
   *
   * A connection to this signal should never be made with a queued
   * connection, because then the "emit" would return before the
   * changes to the options in the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or
   * addOption() as needed.  The option list in the EnumProperty will
   * not be cleared before the signal is emitted. */
  void requestOptions( EnumProperty* property_in_need_of_options );

private:
  QStringList strings_;
  QHash<QString, int> ints_;
};

} // end namespace rviz

#endif // ENUM_PROPERTY_H
