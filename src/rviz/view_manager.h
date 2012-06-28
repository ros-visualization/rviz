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

class ViewManager: public QObject
{
Q_OBJECT
public:
  ViewManager( DisplayContext* context );
  ~ViewManager();

  void initialize( Ogre::SceneNode* target_scene_node );

  void update( float wall_dt, float ros_dt );

  /** @brief Return the current ViewController in use for the main
   * RenderWindow. */
  ViewController* getCurrent() { return current_view_; }

  ViewController* create( const QString& type );

  int getNumViews() const;

  ViewController* getViewAt( int index ) const;

  /** @brief Set the current view controller.
   * @param view The new view controller to use.
   * @param deactivate_previous If true, deactive the previous current
   *        ViewController before activating the new one.  If false, ignore
   *        the previous one.
   * @return Returns true if the current view controller changes, false if it does not. */
  bool setCurrent( ViewController* view, bool deactivate_previous = true );

  void add( ViewController* view, int index = -1 );

//////////////////
// API I am moving towards:
//
// // current view
//  instance getCurrent();
//  bool setCurrent( instance );
//
// // view creation
//  instance create( type );
//
// // changing list of views
//  void add( instance, int index = -1 );
//  instance take( instance );
//  instance takeAt( int index );
//
// // iterating over list of views
//  instance getViewAt( int index );
//  int getNumViews();
//
//Q_SIGNALS:
//  void currentChanged( instance );
//////////////////

  PropertyTreeModel* getPropertyModel() { return property_model_; }

public Q_SLOTS:
  /** @brief Make a copy of the current view controller, add it to the
   * top of the list, and make it current. */
  void copyCurrent();

Q_SIGNALS:
  /** @brief Emitted after the current ViewController has changed. */
  void currentChanged( ViewController* new_current );

  void configChanged();

private Q_SLOTS:
  /** @brief If the object being deleted is the current view, this
   * sets the current view to be a different one, or if there are none
   * left, creates a new one to make current. */
  void onViewDeleted( QObject* deleted_object );

private:
  /** @brief Create, configure, and add a default ViewController, and return it.
   *
   * This does not set it to be current. */
  ViewController* makeDefaultView();

  DisplayContext* context_;
  ViewController* current_view_;
  Ogre::SceneNode* target_scene_node_;
  Property* root_property_;
  PropertyTreeModel* property_model_;
  QStringList class_ids_;
  PluginlibFactory<ViewController>* factory_;
};

} // end namespace rviz

#endif // VIEW_MANAGER_H
