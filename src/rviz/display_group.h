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
#ifndef DISPLAY_GROUP_H
#define DISPLAY_GROUP_H

#include "display.h"
#include <rviz/rviz_export.h>

namespace rviz
{
class DisplayFactory;

/** @brief A Display object which stores other Displays as children.
 *
 * A DisplayGroup can have non-Display child properties as well as
 * Display children, but they are kept separate.  Non-display
 * properties come first, and Display children come after.  The
 * Property superclass stores the non-Display properties and this
 * class stores the Display objects in a separate list.  The
 * separation is enforced in addChild(). */
class RVIZ_EXPORT DisplayGroup : public Display
{
  Q_OBJECT
public:
  DisplayGroup();
  ~DisplayGroup() override;

  Display* createDisplay(const QString& class_id);

  /** @brief Return the number of child objects (Property and Display).
   *
   * Overridden from Property to include the number of child Displays. */
  int numChildren() const override;

  /** @brief Return the child with the given index, without
   * checking whether the index is within bounds.
   *
   * Overridden from Property to include Display children. */
  Property* childAtUnchecked(int index) const override;

  /** @brief Take a child out of the child list, but don't destroy it.
   * @return Returns the child property at the given index, or NULL if the index is out of bounds.
   *
   * This notifies the model about the removal.
   *
   * This is overridden from Property to include Display children. */
  Property* takeChildAt(int index) override;

  /** @brief Add a child Property or Display.
   * @param child The child to add.
   * @param index [optional] The index at which to add the child.  If
   *   less than 0 or greater than the number of child properties, the
   *   child will be added at the end.
   *
   * This notifies the model about the addition.
   *
   * This is overridden from Property to keep non-Display child
   * Properties in Property's list of children and Display children in
   * DisplayGroup's list of child Displays. */
  void addChild(Property* child, int index = -1) override;

  /** @brief Return item flags appropriate for the given column (0 or
   * 1) for this DisplayGroup. */
  Qt::ItemFlags getViewFlags(int column) const override;

  /** @brief Load subproperties and the list of displays in this group
   * from the given Config node, which must be a map. */
  void load(const Config& config) override;

  /** @brief Save subproperties and the list of displays in this group
   * to the given Config node. */
  void save(Config config) const override;

  /** @brief Add a child Display to the end of the list of Displays.
   *
   * This also tells the model that we are adding a child, so it can
   * update widgets.
   *
   * @note This does @e not remove @a child from its parent.  That
   * must be done first to avoid problems. */
  virtual void addDisplay(Display* child);

  /** @brief Remove a child Display from the the list of Displays, but
   *         don't destroy it.
   * @return Returns child if it is found, or NULL if child is not found.
   *
   * This also tells the model that we are removing a child, so it can
   * update widgets. */
  virtual Display* takeDisplay(Display* child);

  /** @brief Remove and destroy all child Displays, but preserve any
   * non-Display children. */
  virtual void removeAllDisplays();

  /** @brief Return the number of child Displays. */
  virtual int numDisplays() const;

  /** @brief Return the index-th Display in this group, or NULL if the
   * index is invalid. */
  virtual Display* getDisplayAt(int index) const;

  /** @brief Find the index-th child Display in this group.  If the
   * child is itself a DisplayGroup, return the pointer to it.  If it
   * is not, return NULL. */
  virtual DisplayGroup* getGroupAt(int index) const;

  /** @brief Call update() on all child Displays. */
  void update(float wall_dt, float ros_dt) override;

  /** @brief Reset this and all child Displays. */
  void reset() override;

public Q_SLOTS:
  void onEnableChanged() override;

protected:
  /** @brief Update the fixed frame in all contained displays. */
  void fixedFrameChanged() override;

  /** @brief Add a child Display to the end of the list of Displays,
   * but without telling the model. */
  virtual void addDisplayWithoutSignallingModel(Display* child);

Q_SIGNALS:

  void displayAdded(rviz::Display* display);
  void displayRemoved(rviz::Display* display);

private:
  QList<Display*> displays_;
};

} // end namespace rviz

#endif // DISPLAY_GROUP_H
