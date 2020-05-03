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
#ifndef PROPERTY_MODEL_H
#define PROPERTY_MODEL_H

#include <QAbstractItemModel>

namespace rviz
{
class Property;

class PropertyTreeModel : public QAbstractItemModel
{
  Q_OBJECT
public:
  /** @brief Constructor.
   * @param root_property The root of the property tree.
   *        PropertyTreeModel takes ownership of root_property and
   *        deletes it in its destructor.
   * @param parent A QObject to set as the parent. */
  PropertyTreeModel(Property* root_property, QObject* parent = nullptr);

  /** @brief Destructor.  Deletes the root property (and thus the
   * entire property tree). */
  ~PropertyTreeModel() override;

  void setDragDropClass(const QString& drag_drop_class)
  {
    drag_drop_class_ = drag_drop_class;
  }

  // Read-only model functions:
  QVariant data(const QModelIndex& index, int role) const override;
  QVariant
  headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex& index) const override;

  /** @brief Same as parent() but taking a Property pointer instead of
   * an index. */
  QModelIndex parentIndex(const Property* child) const;

  /** @brief Return the number of rows under the given parent index. */
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;

  /** @brief Return the number of columns under the given parent
   * index, which is always 2 for this model. */
  int columnCount(const QModelIndex& /*parent*/ = QModelIndex()) const override
  {
    return 2;
  }

  // Editable model functions:
  Qt::ItemFlags flags(const QModelIndex& index) const override;
  bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

  Qt::DropActions supportedDropActions() const override
  {
    return Qt::MoveAction;
  }

  /** @brief Override from QAbstractItemModel.  Returns a
   * (non-standard) mime-encoded version of the given indexes.
   *
   * Returns the model indexes encoded using pointer values, which
   * means they will only work within the application this is compiled
   * into. */
  QMimeData* mimeData(const QModelIndexList& indexes) const override;

  /** @brief Override from QAbstractItemModel.  Takes a (non-standard)
   * mime-encoded version of an index list and drops it at the
   * destination.
   *
   * The model indexes are encoded using pointer values (by
   * mimeData()), which means they will only work within the
   * application this is compiled into. */
  bool dropMimeData(const QMimeData* data,
                    Qt::DropAction action,
                    int destination_row,
                    int destination_column,
                    const QModelIndex& destination_parent) override;

  /** @brief Returns a list with just "application/x-rviz-" plus
   * drag_drop_class_. */
  QStringList mimeTypes() const override;

  Property* getRoot() const
  {
    return root_property_;
  }

  QModelIndex indexOf(Property* property) const;

  /** emit dataChanged() and configChanged() signals (the latter can be suppressed) */
  void emitDataChanged(Property* property, bool emit_config_changed = true);

  void beginInsert(Property* parent_property, int row_within_parent, int count = 1);
  void endInsert();

  void beginRemove(Property* parent_property, int row_within_parent, int count = 1);
  void endRemove();

  /** @brief return the Property at the given index, or the root
   * property if the index is invalid. */
  Property* getProp(const QModelIndex& index) const;

  /** @brief Emit the propertyHiddenChanged() signal for the given Property. */
  void emitPropertyHiddenChanged(const Property* property)
  {
    Q_EMIT propertyHiddenChanged(property);
  }

  /** @brief Expand (show the children of) the given Property. */
  void expandProperty(Property* property);

  /** @brief Collapse (hide the children of) the given Property. */
  void collapseProperty(Property* property);

  /** @brief For debugging only.  Uses printf() to print the property
   * names of current persistent indices. */
  void printPersistentIndices();

Q_SIGNALS:
  /** @brief Emitted when a property within the model is hidden or shown. */
  void propertyHiddenChanged(const Property* property);

  /** @brief Emitted when a Property which should be saved changes. */
  void configChanged();

  /** @brief Emitted when a Property wants to expand (display its children). */
  void expand(const QModelIndex& index);

  /** @brief Emitted when a Property wants to collapse (hide its children). */
  void collapse(const QModelIndex& index);

private:
  Property* root_property_;
  QString drag_drop_class_; ///< Identifier to add to mimeTypes() entry to keep drag/drops from crossing
                            /// types.
};

} // end namespace rviz

#endif // PROPERTY_MODEL_H
