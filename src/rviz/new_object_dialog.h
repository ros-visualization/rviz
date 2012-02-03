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

#ifndef RVIZ_NEW_DISPLAY_DIALOG_H
#define RVIZ_NEW_DISPLAY_DIALOG_H

#include <QDialog>

#include <vector>
#include <set>
#include <string>

#include <pluginlib/class_loader.h>
#include "rviz/display.h"

class QTreeWidget;
class QTreeWidgetItem;
class QTextBrowser;
class QLineEdit;
class QDialogButtonBox;
class QLabel;

namespace rviz
{

typedef std::vector<std::string> V_string;
typedef std::set<std::string> S_string;

class NewObjectDialog : public QDialog
{
Q_OBJECT
public:
  NewObjectDialog( pluginlib::ClassLoaderBase* class_loader,
                   const std::string& object_type,
                    const S_string& current_display_names,
                    std::string* lookup_name_output,
                    std::string* display_name_output,
                    QWidget* parent = 0 );

public Q_SLOTS:
  virtual void accept();

private Q_SLOTS:
  void onDisplaySelected( QTreeWidgetItem* selected_item );
  void onNameChanged();

private:
  /** Fill the tree widget with classes from the class loader. */
  void fillTree( QTreeWidget* tree );

  /** Returns true if entered display name is non-empty and unique and
   * if lookup name is non-empty. */
  bool isValid();

  /** Display an error message to the user, or clear the previous
   * error message if error_text is empty. */
  void setError( const QString& error_text );

  pluginlib::ClassLoaderBase* class_loader_;
  const S_string& current_display_names_;

  std::string* lookup_name_output_;
  std::string* display_name_output_;

  /** Widget showing description of the class. */
  QTextBrowser* description_;

  QLineEdit* name_editor_;

  /** Widget with OK and CANCEL buttons. */
  QDialogButtonBox* button_box_;

  /** Current value of selected class-lookup name.  Copied to
   * *lookup_name_output_ when "ok" is clicked. */
  std::string lookup_name_;
};

} //namespace rviz

#endif
