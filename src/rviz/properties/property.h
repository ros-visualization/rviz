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
#ifndef PROPERTY_H
#define PROPERTY_H

#include <QObject>
#include <QVariant>

namespace YAML
{
class Node;
class Emitter;
}

class QModelIndex;
class QPainter;
class QStyleOptionViewItem;

namespace rviz
{

class PropertyTreeModel;

/** @brief A single element of a property tree, with a name, value,
 *         and possibly children.
 *
 * Property does not support the addition of non-Property objects as
 * children.  This will likely cause a crash. */
class Property: public QObject
{
Q_OBJECT
public:
  Property( const QString& name = QString(),
            const QVariant default_value = QVariant(),
            const QString& description = QString(),
            Property* parent = 0,
            const char *changed_slot = 0,
            QObject* receiver = 0 );

  /** @brief Set the new value for this property.  Returns true if the
   * new value is different from the old value, false if same.
   *
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and changed() after. */
  virtual bool setValue( const QVariant& new_value );
  virtual QVariant getValue() const;

  virtual void setName( const QString& name );
  virtual QString getName() const;

  virtual void setDescription( const QString& description );
  virtual QString getDescription() const;

  /** @brief Return the child Property with given name, or the
   * FailureProperty if it does not exist. */
  virtual Property* subProp( const QString& sub_name );

  /** @brief Return the number of child objects (Property or otherwise). */
  virtual int numChildren() const { return children().size(); }

  /** @brief Return the child Property with the given index, or NULL
   * if the index is out of bounds or if the child at that index is
   * not a Property. */
  virtual Property* childAt( int index ) const;

  /** @brief Return the parent object, if it is a Property, or NULL if not. */
  virtual Property* parentProperty() const;

  /** @brief Return data appropriate for the given column (0 or 1) and role for this Property.
   *
   * When overriding to add new data (like a color for example), check
   * the role for the thing you know about, and if it matches, return
   * your data.  If it does not match, call the parent class version
   * of this function and return its result. */
  virtual QVariant getViewData( int column, int role ) const;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this Property. */
  virtual Qt::ItemFlags getViewFlags( int column ) const;

  virtual bool paint( QPainter* painter,
                      const QStyleOptionViewItem& option ) const { return false; }

  virtual QWidget* createEditor( QWidget* parent,
                                 const QStyleOptionViewItem& option,
                                 const QModelIndex& index );

  bool isAncestorOf( Property* possible_child ) const;

  void addChildAt( Property* child, int index );

  /** @brief Set the model managing this Property and all its child properties, recursively. */
  void setModel( PropertyTreeModel* model );

  PropertyTreeModel* getModel() const { return model_; }

  /** @brief Return the row number of this property within its parent,
   * or -1 if it has no parent. */
  int rowNumberInParent() const;

  void setParentProperty( Property* new_parent );

  void moveChild( int from_index, int to_index );

  /** @brief Load the value of this property and/or its children from
   * the given YAML node. */
  virtual void load( const YAML::Node& yaml_node );

  /** @brief Write the value of this property and/or its children to
   * the given YAML emitter. */
  virtual void save( YAML::Emitter& emitter );

  virtual bool shouldBeSaved() const { return true; }

Q_SIGNALS:
  void aboutToChange();
  void changed();

protected:
  virtual void childEvent( QChildEvent* event );

  QVariant value_;
  PropertyTreeModel* model_;

private:
  void reindexChildren();

  QString description_;

  /** @brief The property returned by subProp() when the requested
   * name is not found. */
  static Property* failprop_;

  bool child_indexes_valid_;
  int row_number_within_parent_;
};

} // end namespace rviz

#endif // PROPERTY_H
