/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef RVIZ_PROPERTY_WIDGET_ITEM_H
#define RVIZ_PROPERTY_WIDGET_ITEM_H

#include <stdio.h>

#include <QTreeWidgetItem>

namespace rviz
{

class PropertyBase;

class PropertyWidgetItem: public QTreeWidgetItem
{
public:
  PropertyWidgetItem( PropertyBase* property,
                      const std::string& label,
                      bool editable = true,
                      bool needs_check_box = false,
                      bool first_column_spans = false );

  /** Add this item to a parent.  If parent_item is given, this item
   * will be a child of that.  If parent_item is not given, it will be
   * added either to the top level of the tree or to the parent
   * category, according to information from the PropertyBase. */
  void addToParent( PropertyWidgetItem* parent_item = 0 );

  PropertyBase* getProperty() { return property_; }

  /** Set the display text for the right column of this item without
   * triggering the data-changed signal. */
  void setRightText( const std::string& text );

  /** Set the display text for the right column of this item without
   * triggering the data-changed signal. */
  void setRightText( const QString& text );

  /** Set the display text for the left column of this item without
   * triggering the data-changed signal. */
  void setLeftText( const std::string& text );

  /** Set cargo data, which goes into the Qt::UserRole of the right
   * column, as well as a string-ified version into the text of the
   * right column. */
  void setUserData( QVariant data );

  /** Return the Qt::UserRole data of the right column. */
  QVariant userData() const;

  /** Do custom painting of the widget item here, and return true; or
   * return false to let the QStyledItemDelegate do the painting. */
  virtual bool paint( QPainter * painter,
                      const QStyleOptionViewItem & option )
  {
    return false; // return false to indicate this call has not done the painting.
  }

  /** Override to create a custom editor widget for this item.  The
   * base implementation returns NULL, which will case the default
   * QLineEdit to be created. */
  virtual QWidget* createEditor( QWidget* parent,
                                 const QStyleOptionViewItem & option )
  {
    return 0;
  }

  /** Override to load an editor with data from the model.  Return
   * true to claim to have done the work, false to fall back on the
   * default behavior. */
  virtual bool setEditorData( QWidget* editor )
  {
    return false;
  }

  /** Override to read data from an editor and store it in the model.
   * Return true to claim to have done the work, false to fall back on
   * the default behavior. */
  virtual bool setModelData( QWidget* editor )
  {
    return false;
  }

  double max_;
  double min_;

private:
  PropertyBase* property_;
  bool first_column_spans_;
};

} // end namespace rviz

#endif // RVIZ_PROPERTY_WIDGET_ITEM_H
