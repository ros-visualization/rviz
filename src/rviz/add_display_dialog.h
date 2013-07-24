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

#ifndef RVIZ_ADD_DISPLAY_DIALOG_H
#define RVIZ_ADD_DISPLAY_DIALOG_H

#include <QDialog>
#include <QTreeWidget>

#include "rviz/factory.h"

class QTextBrowser;
class QLineEdit;
class QDialogButtonBox;
class QLabel;
class QCheckBox;

namespace rviz
{

class DisplayFactory;

/**
 * Meta-data needed to pick a plugin and optionally a topic.
 */
struct SelectionData {
  QString whats_this;
  QString lookup_name;
  QString display_name;
  QString topic;
  QString datatype;
};

class AddDisplayDialog : public QDialog
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
  AddDisplayDialog( DisplayFactory* factory,
                    const QString& object_type,
                    const QStringList& disallowed_display_names,
                    const QStringList& disallowed_class_lookup_names,
                    QString* lookup_name_output,
                    QString* display_name_output = 0,
                    QString* topic_output = 0,
                    QString* datatype_output = 0,
                    QWidget* parent = 0 );

  virtual QSize sizeHint () const;

public Q_SLOTS:
  virtual void accept();

private Q_SLOTS:
  void onDisplaySelected( SelectionData *data);
  void onTopicSelected( SelectionData *data );
  void onTabChanged( int index );
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

  /** Populate text boxes based on current tab and selection */
  void updateDisplay();

  Factory* factory_;
  const QStringList& disallowed_display_names_;
  const QStringList& disallowed_class_lookup_names_;

  QString* lookup_name_output_;
  QString* display_name_output_;
  QString* topic_output_;
  QString* datatype_output_;

  /** Widget holding tabs */
  QTabWidget *tab_widget_;
  /** Index of tab for selection by topic */
  int topic_tab_;
  /** Index of tab for selection by display */
  int display_tab_;
  /** Current data for display tab */
  SelectionData display_data_;
  /** Current data for topic tab */
  SelectionData topic_data_;

  /** Widget showing description of the class. */
  QTextBrowser* description_;

  QLineEdit* name_editor_;

  /** Widget with OK and CANCEL buttons. */
  QDialogButtonBox* button_box_;

  /** Current value of selected class-lookup name.  Copied to
   * *lookup_name_output_ when "ok" is clicked. */
  QString lookup_name_;
};


/** @brief Widget for selecting a display by display type */
class DisplayTypeTree : public QTreeWidget {
Q_OBJECT
public:
  DisplayTypeTree();

  void fillTree(Factory *factory);

Q_SIGNALS:
  void itemChanged( SelectionData *selection );

private Q_SLOTS:
  void onCurrentItemChanged(QTreeWidgetItem *curr, QTreeWidgetItem *prev);
};

/** @brief Widget for selecting a display by topic */
class TopicDisplayWidget : public QWidget {
Q_OBJECT
public:
  TopicDisplayWidget();
  void fill(DisplayFactory *factory);

Q_SIGNALS:
  void itemChanged( SelectionData *selection );
  void itemActivated( QTreeWidgetItem *item, int column );

private Q_SLOTS:
  void stateChanged(int state);
  void onCurrentItemChanged(QTreeWidgetItem *curr, QTreeWidgetItem *prev);

private:
  void findPlugins( DisplayFactory*, QMap<QString, QString> *datatype_plugins );

  QTreeWidget *tree_;
  QCheckBox *enable_hidden_box_;
};

} //namespace rviz

#endif // RVIZ_ADD_DISPLAY_DIALOG_H