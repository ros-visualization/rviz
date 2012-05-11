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
 *         and possibly children. */
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

  virtual ~Property();

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

  /** @brief Return the number of child objects (Property or otherwise).
   *
   * You can override this in a subclass to implement different child
   * storage. */
  virtual int numChildren() const { return children_.size(); }

  /** @brief Return the child Property with the given index, or NULL
   * if the index is out of bounds or if the child at that index is
   * not a Property.
   *
   * This just checks the index against 0 and numChildren() and then
   * calls childAtUnchecked(), so it does not need to be overridden in
   * a subclass. */
  Property* childAt( int index ) const;

  /** @brief Return the child Property with the given index, without
   * checking whether the index is within bounds.
   *
   * You can override this in a subclass to implement different child
   * storage. */
  virtual Property* childAtUnchecked( int index ) const;

  /** @brief Return the parent Property. */
  Property* getParent() const;

  /** @brief Set parent property.  This does not have any side
   * effects, like adding itself to be a child of the parent.  Should
   * only be used by implementations of addChild() and takeChild() and
   * such. */
  void setParent( Property* new_parent );

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

  /** @brief Remove a given child object and return a pointer to it.
   * @return If child is contained here, it is returned; otherwise NULL.
   *
   * This uses only virtual functions, numChildren(),
   * childAtUnchecked(), and takeChildAt(), so it does not need to be
   * virtual itself.  */
  Property* takeChild( Property* child );

  /** @brief Take a child out of the child list, but don't destroy it.
   * @return Returns the child property at the given index, or NULL if the index is out of bounds.
   *
   * This notifies the model about the removal. */
  virtual Property* takeChildAt( int index );

  /** @brief Add a child property.
   * @param child The child property to add.
   * @param index [optional] The index at which to add the child.  If
   *   less than 0 or greater than the number of child properties, the
   *   child will be added at the end. */
  virtual void addChild( Property* child, int index = -1 );

  /** @brief Set the model managing this Property and all its child properties, recursively. */
  void setModel( PropertyTreeModel* model );

  /** @brief Return the model managing this Property and its childrent. */
  PropertyTreeModel* getModel() const { return model_; }

  /** @brief Return the row number of this property within its parent,
   * or -1 if it has no parent. */
  int rowNumberInParent() const;

  /** @brief Move the child at from_index to to_index. */
  virtual void moveChild( int from_index, int to_index );

  /** @brief Load the value of this property and/or its children from
   * the given YAML node. */
  virtual void load( const YAML::Node& yaml_node );

  /** @brief Write the value of this property and/or its children to
   * the given YAML emitter. */
  virtual void save( YAML::Emitter& emitter );

  /** @brief Load the children of this property from the given YAML
   * node, which should be a map node.
   *
   * This base version presumes the children to be loaded already
   * exist as sub-properties of this, and looks for keys in the YAML
   * map which match their names. */
  virtual void loadChildren( const YAML::Node& yaml_node );

  /** @brief Write the children of this property to the given YAML
   * emitter, which should be in a map context. */
  virtual void saveChildren( YAML::Emitter& emitter );

  /** @brief Override this function to return true if this property
   * should be saved to the config file, or false if it should not.
   * The default implementation returns true. */
  virtual bool shouldBeSaved() const { return true; }

Q_SIGNALS:
  /** @brief Emitted by setValue() just before the value has changed. */
  void aboutToChange();
  /** @brief Emitted by setValue() just after the value has changed. */
  void changed();

protected:
  /** @brief This is the central property value.  If you set it
   * directly in a subclass, do so with care because many things
   * depend on the aboutToChange() and changed() events emitted by
   * setValue(). */
  QVariant value_;

  PropertyTreeModel* model_;
  bool child_indexes_valid_;

private:
  void reindexChildren();

  Property* parent_;
  QList<Property*> children_;
  QString description_;

  /** @brief The property returned by subProp() when the requested
   * name is not found. */
  static Property* failprop_;

  int row_number_within_parent_;
};

} // end namespace rviz

#endif // PROPERTY_H
