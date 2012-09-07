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
#ifndef VIEW_MANAGER_H
#define VIEW_MANAGER_H

#include <QList>
#include <QObject>
#include <QStringList>

#include "rviz/pluginlib_factory.h"
#include "rviz/view_controller.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class DisplayContext;
class Property;
class PropertyTreeModel;
class ViewController;
class ViewControllerContainer;

class ViewManager: public QObject
{
Q_OBJECT
public:
  ViewManager( DisplayContext* context );
  ~ViewManager();

  void initialize();

  void update( float wall_dt, float ros_dt );

  /** @brief Return the current ViewController in use for the main
   * RenderWindow. */
  ViewController* getCurrent() const;

  ViewController* create( const QString& type );

  int getNumViews() const;

  ViewController* getViewAt( int index ) const;

  void add( ViewController* view, int index = -1 );

  /** @brief Remove the given ViewController from the list and return
   * it.  If it is not in the list, NULL is returned and nothing
   * changes. */
  ViewController* take( ViewController* view );

  /** @brief Remove the ViewController at the given index from the
   * list and return it.  If the index is not valid, NULL is returned
   * and nothing changes. */
  ViewController* takeAt( int index );

  PropertyTreeModel* getPropertyModel() { return property_model_; }

  void load( const Config& config );
  void save( Config config ) const;

  /** @brief Make a copy of @a view_to_copy and install that as the new current ViewController. */
  void setCurrentFrom( ViewController* view_to_copy );

  /** @brief Return a copy of source, made by saving source to
   * a Config and instantiating and loading a new one from that. */
  ViewController* copy( ViewController* source );

  PluginlibFactory<ViewController>* getFactory() const { return factory_; }

  /** @brief Set the 3D view widget whose view will be controlled by
   * ViewController instances from by this ViewManager. */
  void setRenderPanel( RenderPanel* render_panel );

  /** @brief Return the 3D view widget managed by this ViewManager. */
  RenderPanel* getRenderPanel() const { return render_panel_; }

public Q_SLOTS:

  /** @brief Make a copy of the current ViewController and add it to the end of the list of saved views. */
  void copyCurrentToList();

  /** @brief Create a new view controller of the given type and set it
   * up to mimic and replace the previous current view. */
  void setCurrentViewControllerType( const QString& new_class_id );

Q_SIGNALS:
  void configChanged();

  /** @brief Emitted just after the current view controller changes. */
  void currentChanged();

private Q_SLOTS:
  void onCurrentDestroyed( QObject* obj );

private:
  /** @brief Set @a new_current as current.
   * @param mimic_view If true, call new_current->mimic( previous ), if false call new_current->transitionFrom( previous ).
   *
   * This calls mimic() or transitionFrom() on the new controller,
   * deletes the previous controller (if one existed), and tells the
   * RenderPanel about the new controller. */
  void setCurrent( ViewController* new_current, bool mimic_view );

  DisplayContext* context_;
  ViewControllerContainer* root_property_;
  PropertyTreeModel* property_model_;
  PluginlibFactory<ViewController>* factory_;
  ViewController* current_;
  RenderPanel* render_panel_;
};

/** @brief Container property for ViewControllers which gets the
 * drag/drop right for the funky way Current-View is always the first
 * entry. */
class ViewControllerContainer: public Property
{
Q_OBJECT
public:
  Qt::ItemFlags getViewFlags( int column ) const;

  /** @brief Add a child ViewController.
   * @param child The child to add.
   * @param index [optional] The index at which to add the child.  If
   *   less than 0 or greater than the number of child properties, the
   *   child will be added at the end.
   *
   * This notifies the model about the addition.
   *
   * This is overridden from Property to keep saved ViewControllers from being added 
   * at index 0, where the Current view belongs. */
  virtual void addChild( Property* child, int index = -1 );

  void addChildToFront( Property* child );
};

} // end namespace rviz

#endif // VIEW_MANAGER_H
