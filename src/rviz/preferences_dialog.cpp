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

#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>

#include "preferences_dialog.h"
#include "rviz/preferences.h"

namespace rviz
{
PreferencesDialog::PreferencesDialog(Factory* factory, Preferences* preferences_output, QWidget* parent)
  : QDialog(parent), factory_(factory), preferences_(preferences_output)
{
  //***** Layout

  // Display Type group
  QGroupBox* preferences_box = new QGroupBox("Preferences");

  QVBoxLayout* preferences_layout = new QVBoxLayout;
  preferences_layout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  prompt_save_on_exit_checkbox_ = new QCheckBox;
  prompt_save_on_exit_checkbox_->setChecked(preferences_->prompt_save_on_exit);
  prompt_save_on_exit_checkbox_->setText(QString("Prompt Save on Exit?"));
  preferences_layout->addWidget(prompt_save_on_exit_checkbox_);
  preferences_box->setLayout(preferences_layout);

  // Buttons
  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal);

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget(preferences_box);
  main_layout->addWidget(button_box_);
  setLayout(main_layout);

  //***** Connections
  connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  connect(button_box_, SIGNAL(rejected()), this, SLOT(reject()));
}

QSize PreferencesDialog::sizeHint() const
{
  return (QSize(500, 100));
}

void PreferencesDialog::setError(const QString& error_text)
{
  button_box_->button(QDialogButtonBox::Ok)->setToolTip(error_text);
}

bool PreferencesDialog::isValid()
{
  setError("");
  return true;
}

void PreferencesDialog::accept()
{
  if (isValid())
  {
    preferences_->prompt_save_on_exit = prompt_save_on_exit_checkbox_->isChecked();
    QDialog::accept();
  }
}

} // namespace rviz
