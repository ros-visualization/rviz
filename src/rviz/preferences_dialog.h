/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_PREFERENCES_DIALOG_H
#define RVIZ_PREFERENCES_DIALOG_H

#include <QDialog>

#include "rviz/factory.h"

class QTreeWidget;
class QTreeWidgetItem;
class QTextBrowser;
class QLineEdit;
class QDialogButtonBox;
class QLabel;

namespace rviz
{

class Preferences;

class PreferencesDialog : public QDialog
{
Q_OBJECT
public:
  /** Dialog for choosing a new object to load with a pluginlib ClassLoader.
   *
   * @param disallowed_display_names set of display names to prevent
   *        the user from using.
   *
   * @param disallowed_class_lookup_names set of class lookup names to
   *        prevent the user from selecting.  Names found in the class loader
   *        which are in this list will appear disabled.
   *
   * @param lookup_name_output Pointer to a string where dialog will
   *        put the class lookup name chosen.
   *
   * @param display_name_output Pointer to a string where dialog will
   *        put the display name entered, or NULL (default) if display
   *        name entry field should not be shown. */
  PreferencesDialog( Factory* factory,
                   Preferences* preferences,
                   QWidget* parent = 0 );

  virtual QSize sizeHint () const;

public Q_SLOTS:
  virtual void accept();

private:
  /** Returns true if entered display name is non-empty and unique and
   * if lookup name is non-empty. */
  bool isValid();

  /** Display an error message to the user, or clear the previous
   * error message if error_text is empty. */
  void setError( const QString& error_text );

  Factory* factory_;

  QCheckBox* prompt_save_on_exit_checkbox_;
  Preferences* preferences_;

  /** Widget with OK and CANCEL buttons. */
  QDialogButtonBox* button_box_;
};

} //namespace rviz

#endif // RVIZ_NEW_OBJECT_DIALOG_H
