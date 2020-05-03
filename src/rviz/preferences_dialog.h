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

class QCheckBox;
class QDialogButtonBox;

namespace rviz
{
struct Preferences;

class PreferencesDialog : public QDialog
{
  Q_OBJECT
public:
  /** Dialog for setting preferences.
   *
   * @param preferences_output Pointer to Preferences struct where
   *        preferences chosen by the user will be put.
   */
  PreferencesDialog(Factory* factory, Preferences* preferences_output, QWidget* parent = nullptr);

  QSize sizeHint() const override;

public Q_SLOTS:
  void accept() override;

private:
  /** Returns true if entered display name is non-empty and unique and
   * if lookup name is non-empty. */
  bool isValid();

  /** Display an error message to the user, or clear the previous
   * error message if error_text is empty. */
  void setError(const QString& error_text);

  Factory* factory_;

  QCheckBox* prompt_save_on_exit_checkbox_;
  Preferences* preferences_;

  /** Widget with OK and CANCEL buttons. */
  QDialogButtonBox* button_box_;
};

} // namespace rviz

#endif // RVIZ_NEW_OBJECT_DIALOG_H
