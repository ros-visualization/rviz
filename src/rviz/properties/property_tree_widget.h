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
#ifndef RVIZ_PROPERTY_TREE_WIDGET_H
#define RVIZ_PROPERTY_TREE_WIDGET_H

#include <set>

#include <QTreeWidget>

namespace rviz
{

class PropertyWidgetItem;
class SplitterHandle;

class PropertyTreeWidget: public QTreeWidget
{
Q_OBJECT
public:
  PropertyTreeWidget( QWidget* parent = 0 );

  /** While ignoring changes, this widget will not call
   * Property::readFromGrid() in response to changes in its items.
   * This is useful when you need to make non-data changes to the
   * items, or when the model already has the data and you just need
   * to send it to the widget.  Default is false.
   * @return previous value of ignore_changes_. */
  bool setIgnoreChanges( bool ignore_changes )
  {
    bool old = ignore_changes_;
    ignore_changes_ = ignore_changes;
    return old;
  }
  bool getIgnoreChanges() { return ignore_changes_; }

  /** Get the PropertyWidgetItem* for the given index. */
  PropertyWidgetItem* getItem( const QModelIndex & index );

  /** Return a string storing the expanded-or-not state of each item,
   * and splitter position. */
  std::string saveEditableState();

  /** Restore state from a string previously returned by saveEditableState(). */
  void restoreEditableState( const std::string& state );

Q_SIGNALS:
  void orderChanged();

public Q_SLOTS:
  void startPersistCurrent();
  void endPersistCurrent();

protected:
  virtual void resizeEvent( QResizeEvent* event );
  virtual Qt::DropActions supportedDropActions() const { return Qt::MoveAction; }

  /** Reimplemented from QTreeWidget to send the orderChanged()
   * signal. */
  virtual void dropEvent( QDropEvent* event );

private Q_SLOTS:
  void onItemChanged( QTreeWidgetItem* item, int column_number );

private:
  /** Recursive function to iterate through tree of items, writing
   * output as it goes. */
  void saveExpandedState( std::ostream& output,
                          QTreeWidgetItem* parent_item,
                          bool& first );

  /** Recursive function to iterate through tree of items, checking
   * each for presence in the expanded_entries set. */
  void restoreExpandedState( const std::set<std::string>& expanded_entries,
                             QTreeWidgetItem* parent_item );

  bool ignore_changes_;
  SplitterHandle* splitter_handle_;
  QTreeWidgetItem* persisted_item_;
};

} // end namespace rviz

#endif // RVIZ_PROPERTY_TREE_WIDGET_H
