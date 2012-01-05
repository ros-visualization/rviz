/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_DISPLAYS_PANEL_H
#define RVIZ_DISPLAYS_PANEL_H

#include <QWidget>

#include <boost/thread/mutex.hpp>

#include <vector>
#include <map>
#include <set>

class QTreeWidgetItem;
class QPushButton;

namespace rviz
{

class PropertyTreeWidget;
class PropertyTreeWithHelp;
class Config;
class VisualizationManager;
class Display;
class DisplayWrapper;
typedef std::vector<DisplayWrapper*> V_DisplayWrapper;

/**
 * \class DisplaysPanel
 *
 */
class DisplaysPanel: public QWidget
{
Q_OBJECT
public:
  DisplaysPanel( QWidget* parent = 0);
  virtual ~DisplaysPanel();

  void initialize( VisualizationManager* manager );

  PropertyTreeWidget* getPropertyTreeWidget() { return property_grid_; }
  VisualizationManager* getManager() { return manager_; }

protected Q_SLOTS:
  /// Call 5 times per second.
  void onStateChangedTimer();

  /// Called when the "Add" button is pressed
  void onNewDisplay();
  /// Called when the "Remove" button is pressed
  void onDeleteDisplay();
  /// Called when the "Rename" button is pressed
  void onRenameDisplay();

  void onSelectionChanged();

  /** Renumber displays based on order in tree widget. */
  void renumberDisplays();

  // Other callbacks
  /// Called when a display is enabled or disabled
  void onDisplayStateChanged( Display* display );
  void onDisplayCreated( DisplayWrapper* display );
  void onDisplayDestroyed( DisplayWrapper* display );
  void onDisplayAdding( DisplayWrapper* display );
  void onDisplayAdded( DisplayWrapper* display );
  void onDisplayRemoved( DisplayWrapper* display );

  /** Read saved state from the given config object. */
  void readFromConfig( const boost::shared_ptr<Config>& config );

  /** Write state to the given config object. */
  void writeToConfig( const boost::shared_ptr<Config>& config );

protected:
  void setDisplayCategoryLabel( const DisplayWrapper* display, int index );
  void setDisplayCategoryColor( const DisplayWrapper* display );

  void sortDisplays();

  /** Given a QTreeWidgetItem, return the corresponding
   * DisplayWrapper, or NULL if the item does not correspond to a
   * DisplayWrapper. */
  DisplayWrapper* displayWrapperFromItem( QTreeWidgetItem* selected_item );

  /** Return the set of DisplayWrappers which are currently selected. */
  std::set<DisplayWrapper*> getSelectedDisplays();

  PropertyTreeWidget* property_grid_;
  VisualizationManager* manager_;

  typedef std::map<DisplayWrapper*, uint32_t> M_DisplayToIndex;
  M_DisplayToIndex display_map_;

  typedef std::set<Display*> S_Display;
  boost::mutex state_changed_displays_mutex_;
  S_Display state_changed_displays_;

  QPushButton* remove_button_;
  QPushButton* rename_button_;
  PropertyTreeWithHelp* tree_with_help_;
};

} // namespace rviz

#endif

