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
#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H

#include <QList>
#include <QObject>
#include <QStringList>

#include "rviz/pluginlib_factory.h"
#include "rviz/tool.h"

class QKeyEvent;

namespace rviz
{
class DisplayContext;
class PropertyTreeModel;
class RenderPanel;

class ToolManager: public QObject
{
Q_OBJECT
public:
  ToolManager( DisplayContext* context );
  virtual ~ToolManager();

  /** @brief Initialization for after the DisplayContext is created.
   * Loads standard RViz tools. */
  void initialize();

  void load( const Config& config );
  void save( Config config ) const;
  PropertyTreeModel* getPropertyModel() const { return property_tree_model_; }

  /** @brief Create a tool by class lookup name, add it to the list, and return it. */
  Tool* addTool( const QString& tool_class_lookup_name );

  /**
   * \brief Return the tool currently in use.
   * \sa setCurrentTool()
   */
  Tool* getCurrentTool() { return current_tool_; }

  /**
   * \brief Return the tool at a given index in the Tool list.
   * If index is less than 0 or greater than the number of tools, this
   * will fail an assertion.
   */
  Tool* getTool( int index );

  int numTools() { return tools_.size(); }
  void removeTool( int index );

  void removeAll();

  /** @brief Triggers redrawing the tool's icon/text in the toolbar. */
  void refreshTool( Tool* tool );

  /**
   * \brief Set the current tool.
   * The current tool is given all mouse and keyboard events which
   * VisualizationManager receives via handleMouseEvent() and
   * handleChar().
   * \sa getCurrentTool()
   */
  void setCurrentTool( Tool* tool );

  /**
   * \brief Set the default tool.
   *
   * The default tool is selected directly by pressing the Escape key.
   * The default tool is indirectly selected when a Tool returns
   * Finished in the bit field result of Tool::processMouseEvent().
   * This is how control moves from the InitialPoseTool back to
   * MoveCamera when InitialPoseTool receives a left mouse button
   * release event.
   * \sa getDefaultTool()
   */
  void setDefaultTool( Tool* tool );

  /**
   * \brief Get the default tool.
   * \sa setDefaultTool()
   */
  Tool* getDefaultTool() { return default_tool_; }

  QStringList getToolClasses();

  void handleChar( QKeyEvent* event, RenderPanel* panel );

  PluginlibFactory<Tool>* getFactory() { return factory_; }

Q_SIGNALS:
  /** @brief Emitted when anything changes which will change the saved config file contents. */
  void configChanged();

  /** @brief Emitted by addTool() after the tool is added to the list of tools. */
  void toolAdded( Tool* );

  /** @brief Emitted by setCurrentTool() after the newly chosen tool
   * is activated. */
  void toolChanged( Tool* );

  void toolRemoved( Tool* );

  /** @brief Emitted by refreshTool() to gedraw the tool's icon in the toolbar'. */
  void toolRefreshed( Tool* );

private Q_SLOTS:
  /** @brief If @a property has children, it is added to the tool
   * property tree, and if it does not, it is removed. */
  void updatePropertyVisibility( Property* property );

private:

  bool toKey( QString const& str, uint& key_out );
  PluginlibFactory<Tool>* factory_;
  PropertyTreeModel* property_tree_model_;
  QList<Tool*> tools_;
  DisplayContext* context_;
  Tool* current_tool_;
  Tool* default_tool_;
  std::map<int,Tool*> shortkey_to_tool_map_;

};

} // end namespace rviz

#endif // TOOL_MANAGER_H
