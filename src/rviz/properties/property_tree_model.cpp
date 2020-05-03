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

#include <stdio.h>

#include <QStringList>
#include <QMimeData>

#include "rviz/properties/property.h"

#include "rviz/properties/property_tree_model.h"

namespace rviz
{
PropertyTreeModel::PropertyTreeModel(Property* root_property, QObject* parent)
  : QAbstractItemModel(parent), root_property_(root_property)
{
  root_property_->setModel(this);
}

PropertyTreeModel::~PropertyTreeModel()
{
  delete root_property_;
}

Property* PropertyTreeModel::getProp(const QModelIndex& index) const
{
  if (index.isValid())
  {
    Property* prop = static_cast<Property*>(index.internalPointer());
    if (prop)
    {
      return prop;
    }
  }
  return root_property_;
}

Qt::ItemFlags PropertyTreeModel::flags(const QModelIndex& index) const
{
  if (!index.isValid())
  {
    root_property_->getViewFlags(0);
  }
  Property* property = getProp(index);
  return property->getViewFlags(index.column());
}

QModelIndex PropertyTreeModel::index(int row, int column, const QModelIndex& parent_index) const
{
  if (parent_index.isValid() && parent_index.column() != 0)
  {
    return QModelIndex();
  }
  Property* parent = getProp(parent_index);

  Property* child = parent->childAt(row);
  if (child)
  {
    return createIndex(row, column, child);
  }
  else
  {
    return QModelIndex();
  }
}

QModelIndex PropertyTreeModel::parent(const QModelIndex& child_index) const
{
  if (!child_index.isValid())
  {
    return QModelIndex();
  }
  Property* child = getProp(child_index);
  return parentIndex(child);
}

QModelIndex PropertyTreeModel::parentIndex(const Property* child) const
{
  if (!child)
  {
    return QModelIndex();
  }
  Property* parent = child->getParent();
  if (parent == root_property_ || !parent)
  {
    return QModelIndex();
  }
  return indexOf(parent);
}

int PropertyTreeModel::rowCount(const QModelIndex& parent_index) const
{
  return getProp(parent_index)->numChildren();
}

QVariant PropertyTreeModel::data(const QModelIndex& index, int role) const
{
  if (!index.isValid())
  {
    return QVariant();
  }

  return getProp(index)->getViewData(index.column(), role);
}

QVariant
PropertyTreeModel::headerData(int /*section*/, Qt::Orientation /*orientation*/, int /*role*/) const
{
  // we don't use headers.
  return QVariant();
}

bool PropertyTreeModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
  Property* property = getProp(index);

  if (property->getValue().type() == QVariant::Bool && role == Qt::CheckStateRole)
  {
    if (property->setValue(value.toInt() != Qt::Unchecked))
    {
      return true;
    }
  }

  if (role != Qt::EditRole)
  {
    return false;
  }

  return property->setValue(value);
}

/** @brief Override from QAbstractItemModel.  Returns a
 * (non-standard) mime-encoded version of the given indexes.
 *
 * Returns the model indexes encoded using pointer values, which
 * means they will only work within the application this is compiled
 * into. */
QMimeData* PropertyTreeModel::mimeData(const QModelIndexList& indexes) const
{
  if (indexes.count() <= 0)
  {
    return nullptr;
  }
  QStringList types = mimeTypes();
  if (types.isEmpty())
  {
    return nullptr;
  }
  QMimeData* data = new QMimeData();
  QString format = types.at(0);
  QByteArray encoded;
  QDataStream stream(&encoded, QIODevice::WriteOnly);

  QModelIndexList::ConstIterator it = indexes.begin();
  for (; it != indexes.end(); ++it)
  {
    if ((*it).column() == 0)
    {
      void* pointer = (*it).internalPointer();
      stream.writeRawData((char*)&pointer, sizeof(void*));
    }
  }

  data->setData(format, encoded);
  return data;
}

/** @brief Override from QAbstractItemModel.  Takes a (non-standard)
 * mime-encoded version of an index list and drops it at the
 * destination.
 *
 * The model indexes are encoded using pointer values (by
 * mimeData()), which means they will only work within the
 * application this is compiled into. */
bool PropertyTreeModel::dropMimeData(const QMimeData* data,
                                     Qt::DropAction action,
                                     int dest_row,
                                     int /*dest_column*/,
                                     const QModelIndex& dest_parent)
{
  if (!data || action != Qt::MoveAction)
  {
    return false;
  }
  QStringList types = mimeTypes();
  if (types.isEmpty())
  {
    return false;
  }
  QString format = types.at(0);
  if (!data->hasFormat(format))
  {
    return false;
  }
  QByteArray encoded = data->data(format);
  QDataStream stream(&encoded, QIODevice::ReadOnly);

  Property* dest_parent_property = getProp(dest_parent);

  QList<Property*> source_properties;

  // Decode the mime data.
  while (!stream.atEnd())
  {
    void* pointer;
    if (sizeof(void*) != stream.readRawData((char*)&pointer, sizeof(void*)))
    {
      printf("ERROR: dropped mime data has invalid pointer data.\n");
      return false;
    }
    Property* prop = static_cast<Property*>(pointer);
    if (prop == dest_parent_property || prop->isAncestorOf(dest_parent_property))
    {
      // Can't drop a row into its own child.
      return false;
    }
    source_properties.append(prop);
  }

  if (dest_row == -1)
  {
    dest_row = dest_parent_property->numChildren();
  }
  for (int i = 0; i < source_properties.size(); i++)
  {
    Property* prop = source_properties.at(i);
    // When moving multiple items, source indices can change.
    // Therefore we ask each property for its row just before we move
    // it.
    int source_row = prop->rowNumberInParent();

    prop->getParent()->takeChildAt(source_row);

    if (dest_parent_property == prop->getParent() && dest_row > source_row)
    {
      dest_row--;
    }

    dest_parent_property->addChild(prop, dest_row);
    dest_row++;
  }

  return true;
}

QStringList PropertyTreeModel::mimeTypes() const
{
  QStringList result;
  result.append("application/x-rviz-" + drag_drop_class_);
  return result;
}

QModelIndex PropertyTreeModel::indexOf(Property* property) const
{
  if (property == root_property_ || !property)
  {
    return QModelIndex();
  }
  return createIndex(property->rowNumberInParent(), 0, property);
}

void PropertyTreeModel::emitDataChanged(Property* property)
{
  if (property->shouldBeSaved())
  {
    Q_EMIT configChanged();
  }
  QModelIndex left_index = indexOf(property);
  QModelIndex right_index = createIndex(left_index.row(), 1, left_index.internalPointer());
  Q_EMIT dataChanged(left_index, right_index);
}

void PropertyTreeModel::beginInsert(Property* parent_property, int row_within_parent, int count)
{
  // printf( "PropertyTreeModel::beginInsert() into %s row %d, %d rows.  Persistent indices:\n",
  //         qPrintable( parent_property->getName()), row_within_parent, count );
  // printPersistentIndices();

  beginInsertRows(indexOf(parent_property), row_within_parent, row_within_parent + count - 1);
}

void PropertyTreeModel::endInsert()
{
  endInsertRows();
  // printf( "PropertyTreeModel::endInsert()\n" );
}

void PropertyTreeModel::beginRemove(Property* parent_property, int row_within_parent, int count)
{
  // printf( "PropertyTreeModel::beginRemove() from %s row %d, %d rows.  Persistent indices:\n",
  //         qPrintable( parent_property->getName()), row_within_parent, count );
  // printPersistentIndices();

  beginRemoveRows(indexOf(parent_property), row_within_parent, row_within_parent + count - 1);
}

void PropertyTreeModel::endRemove()
{
  endRemoveRows();
  //  printf( "PropertyTreeModel::endRemove()\n" );
}

void PropertyTreeModel::expandProperty(Property* property)
{
  Q_EMIT expand(indexOf(property));
}

void PropertyTreeModel::collapseProperty(Property* property)
{
  Q_EMIT collapse(indexOf(property));
}

void PropertyTreeModel::printPersistentIndices()
{
  QModelIndexList indexes = persistentIndexList();
  QModelIndexList::ConstIterator it = indexes.begin();
  for (; it != indexes.end(); ++it)
  {
    if (!(*it).isValid())
    {
      printf("  invalid index\n");
    }
    else
    {
      Property* prop = getProp(*it);
      if (!prop)
      {
        printf("  null property\n");
      }
      else
      {
        printf("  prop name '%s'\n", qPrintable(prop->getName()));
      }
    }
  }
}

} // end namespace rviz
