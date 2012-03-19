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

#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QInputDialog>

#include <set>

#include <boost/bind.hpp>

#include "displays_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "display_wrapper.h"
#include "new_object_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "properties/property_tree_with_help.h"
#include "properties/property_tree_widget.h"
#include "properties/property_widget_item.h"
#include "config.h"
#include "rviz/uniform_string_stream.h"

static const std::string PROPERTY_GRID_CONFIG("Property Grid State");
static const std::string PROPERTY_GRID_SPLITTER("Property Grid Splitter");

namespace rviz
{

DisplaysPanel::DisplaysPanel( QWidget* parent )
  : QWidget( parent )
  , manager_( NULL )
{
  tree_with_help_ = new PropertyTreeWithHelp;
  property_grid_ = tree_with_help_->getTree();
  property_grid_->setDragEnabled( true );
  property_grid_->setAcceptDrops( true );
  property_grid_->setAnimated( true );

  QPushButton* add_button = new QPushButton( "Add" );
  add_button->setShortcut( QKeySequence( QString( "Ctrl+N" )));
  add_button->setToolTip( "Add a new display, Ctrl+N" );
  remove_button_ = new QPushButton( "Remove" );
  remove_button_->setShortcut( QKeySequence( QString( "Ctrl+X" )));
  remove_button_->setToolTip( "Remove displays, Ctrl+X" );
  remove_button_->setEnabled( false );
  rename_button_ = new QPushButton( "Rename" );
  rename_button_->setShortcut( QKeySequence( QString( "Ctrl+R" )));
  rename_button_->setToolTip( "Rename a display, Ctrl+R" );
  rename_button_->setEnabled( false );

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( add_button );
  button_layout->addWidget( remove_button_ );
  button_layout->addWidget( rename_button_ );

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( tree_with_help_ );
  layout->addLayout( button_layout );

  setLayout( layout );

  connect( add_button, SIGNAL( clicked( bool )), this, SLOT( onNewDisplay() ));
  connect( remove_button_, SIGNAL( clicked( bool )), this, SLOT( onDeleteDisplay() ));
  connect( rename_button_, SIGNAL( clicked( bool )), this, SLOT( onRenameDisplay() ));
  connect( property_grid_, SIGNAL( itemSelectionChanged() ), this, SLOT( onSelectionChanged() ));
  connect( property_grid_, SIGNAL( orderChanged() ), this, SLOT( renumberDisplays() ));

  QTimer* timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( onStateChangedTimer() ));
  timer->start( 200 );
}

DisplaysPanel::~DisplaysPanel()
{
}

void DisplaysPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;
  connect( manager_, SIGNAL( displayAdding( DisplayWrapper* )), this, SLOT( onDisplayAdding( DisplayWrapper* )));
  connect( manager_, SIGNAL( displayAdded( DisplayWrapper* )), this, SLOT( onDisplayAdded( DisplayWrapper* )));
  connect( manager_, SIGNAL( displayRemoved( DisplayWrapper* )), this, SLOT( onDisplayRemoved( DisplayWrapper* )));
  connect( manager_, SIGNAL( displaysConfigLoaded( const boost::shared_ptr<Config>& )),
           this, SLOT( readFromConfig( const boost::shared_ptr<Config>& )));
  connect( manager_, SIGNAL( displaysConfigSaved( const boost::shared_ptr<Config>& )),
           this, SLOT( writeToConfig( const boost::shared_ptr<Config>& )));

  manager_->getPropertyManager()->setPropertyTreeWidget( property_grid_ );

  sortDisplays();
}

void DisplaysPanel::sortDisplays()
{
  property_grid_->sortItems( 0, Qt::AscendingOrder );
}

DisplayWrapper* DisplaysPanel::displayWrapperFromItem( QTreeWidgetItem* selected_item )
{
  DisplayWrapper* selected_display = 0;

  PropertyWidgetItem* pwi = dynamic_cast<PropertyWidgetItem*>( selected_item );
  if( pwi )
  {
    PropertyBase* property = pwi->getProperty();

    void* user_data = property->getUserData();
    if( user_data )
    {
      DisplayWrapper* wrapper = reinterpret_cast<DisplayWrapper*>( user_data );

      if( manager_->isValidDisplay( wrapper ))
      {
        selected_display = manager_->getDisplayWrapper( wrapper->getName() );
      }
      else
      {
        DisplayWrapper* wrapper = manager_->getDisplayWrapper( reinterpret_cast<Display*>( user_data ));

        if( wrapper )
        {
          selected_display = wrapper;
        }
      }
    }
  }
  return selected_display;
}

void DisplaysPanel::onNewDisplay()
{
  // Get the list of current display names, so we can enforce that the
  // new display has a unique name.
  S_string current_display_names;
  manager_->getDisplayNames(current_display_names);

  std::string lookup_name;
  std::string display_name;

  NewObjectDialog* dialog = new NewObjectDialog( manager_->getDisplayClassLoader(),
                                                 "Display",
                                                 current_display_names,
                                                 &lookup_name,
                                                 &display_name );
  if( dialog->exec() == QDialog::Accepted )
  {
    manager_->createDisplay( lookup_name, display_name, true );
  }
  activateWindow(); // Force keyboard focus back on main window.
}

void DisplaysPanel::onDeleteDisplay()
{
  std::set<DisplayWrapper*> displays_to_delete = getSelectedDisplays();

  std::set<DisplayWrapper*>::iterator di;
  for( di = displays_to_delete.begin(); di != displays_to_delete.end(); di++ )
  {
    manager_->removeDisplay( *di );
  }
}

std::set<DisplayWrapper*> DisplaysPanel::getSelectedDisplays()
{
  std::set<DisplayWrapper*> displays;

  QList<QTreeWidgetItem*> selection = property_grid_->selectedItems();
  QList<QTreeWidgetItem*>::iterator si;
  for( si = selection.begin(); si != selection.end(); si++ )
  {
    DisplayWrapper* selected = displayWrapperFromItem( *si );
    if( selected )
    {
      displays.insert( selected );
    }
  }
  return displays;
}

void DisplaysPanel::onSelectionChanged()
{
  std::set<DisplayWrapper*> displays = getSelectedDisplays();

  int num_displays_selected = displays.size();

  remove_button_->setEnabled( num_displays_selected > 0 );
  rename_button_->setEnabled( num_displays_selected == 1 );
}

void DisplaysPanel::onRenameDisplay()
{
  std::set<DisplayWrapper*> displays = getSelectedDisplays();
  if( displays.size() == 0 )
  {
    return;
  }
  DisplayWrapper* display_to_rename = *(displays.begin());

  if( !display_to_rename )
  {
    return;
  }

  bool ok = true;
  QString new_name;
  std::string new_name_std;
  QString old_name = QString::fromStdString( display_to_rename->getName() );
  do
  {
    QString prompt;

    if (!ok)
    {
      prompt = "That name is already taken.  Please try another.";
    }
    else
    {
      prompt = "New Name?";
    }
    new_name = QInputDialog::getText( this, "Rename Display", prompt, QLineEdit::Normal, old_name );

    ok = true;
    if( new_name.isEmpty() || new_name == old_name )
    {
      return;
    }

    new_name_std = new_name.toStdString();

    // Make sure the new name is not already taken
    M_DisplayToIndex::iterator it = display_map_.begin();
    M_DisplayToIndex::iterator end = display_map_.end();
    for (; it != end; ++it)
    {
      DisplayWrapper* wrapper = (*it).first;
      if( wrapper->getName() == new_name_std )
      {
        ok = false;
        break;
      }
    }
  } while (!ok);

  display_to_rename->setName( new_name_std );
  renumberDisplays();
}

void DisplaysPanel::setDisplayCategoryLabel(const DisplayWrapper* wrapper, int index)
{
  char buf[1024];
  snprintf( buf, 1024, "%02d. %s (%s)", index + 1, wrapper->getName().c_str(), wrapper->getClassDisplayName().c_str());
  wrapper->getCategory().lock()->setLabel(buf);
}

void DisplaysPanel::setDisplayCategoryColor(const DisplayWrapper* wrapper)
{
  CategoryPropertyPtr cat = wrapper->getCategory().lock();

  if (!wrapper->isLoaded())
  {
    cat->setToError();
  }
  else if ( wrapper->getDisplay()->isEnabled() )
  {
    switch (wrapper->getDisplay()->getStatus())
    {
    case status_levels::Ok:
      cat->setToOK();
      break;
    case status_levels::Warn:
      cat->setToWarn();
      break;
    case status_levels::Error:
      cat->setToError();
      break;
    }
  }
  else
  {
    cat->setToDisabled();
  }

  PropertyWidgetItem* item = cat->getWidgetItem();
  if( item )
  {
    bool ign = property_grid_->setIgnoreChanges( true );
    item->setFlags( item->flags() | Qt::ItemIsDragEnabled );
    property_grid_->setIgnoreChanges( ign );
  }
}

void DisplaysPanel::onStateChangedTimer()
{
  S_Display local_displays;
  {
    boost::mutex::scoped_lock lock(state_changed_displays_mutex_);
    local_displays.swap(state_changed_displays_);
  }

  S_Display::iterator it = local_displays.begin();
  S_Display::iterator end = local_displays.end();
  for (; it != end; ++it)
  {
    Display* display = *it;
    DisplayWrapper* wrapper = manager_->getDisplayWrapper(display);
    if (!wrapper)
    {
      continue;
    }

    M_DisplayToIndex::iterator it = display_map_.find(wrapper);
    if (it == display_map_.end())
    {
      continue;
    }

    int index = it->second;
    setDisplayCategoryColor(wrapper);
    setDisplayCategoryLabel(wrapper, index);
  }
}

void DisplaysPanel::onDisplayStateChanged( Display* display )
{
  // This can be called from different threads, so we have to push this to the GUI update thread
  boost::mutex::scoped_lock lock(state_changed_displays_mutex_);
  state_changed_displays_.insert(display);
}

void DisplaysPanel::onDisplayCreated( DisplayWrapper* wrapper )
{
  connect( wrapper->getDisplay(), SIGNAL( stateChanged( Display* )), this, SLOT( onDisplayStateChanged( Display* )));

  setDisplayCategoryColor(wrapper);

  update();
}

void DisplaysPanel::onDisplayDestroyed( DisplayWrapper* wrapper )
{
  M_DisplayToIndex::iterator it = display_map_.find(wrapper);
  if (it == display_map_.end())
  {
    return;
  }

  setDisplayCategoryColor(wrapper);


  int index = it->second;
  setDisplayCategoryLabel(wrapper, index);

  update();
}

void DisplaysPanel::onDisplayAdding( DisplayWrapper* wrapper )
{
  connect( wrapper, SIGNAL( displayCreated( DisplayWrapper* )), this, SLOT( onDisplayCreated( DisplayWrapper* )));
  connect( wrapper, SIGNAL( displayDestroyed( DisplayWrapper* )), this, SLOT( onDisplayDestroyed( DisplayWrapper* )));
}

void DisplaysPanel::onDisplayAdded( DisplayWrapper* wrapper )
{
  int index = display_map_.size();
  bool inserted = display_map_.insert(std::make_pair(wrapper, index)).second;
  ROS_ASSERT(inserted);
  setDisplayCategoryLabel(wrapper, index);
  setDisplayCategoryColor(wrapper);
}

void DisplaysPanel::onDisplayRemoved( DisplayWrapper* wrapper )
{
  M_DisplayToIndex::iterator it = display_map_.find(wrapper);
  ROS_ASSERT(it != display_map_.end());

  uint32_t index = it->second;

  display_map_.erase(it);

  it = display_map_.begin();
  M_DisplayToIndex::iterator end = display_map_.end();
  for (;it != end; ++it)
  {
    if (it->second > index)
    {
      --it->second;
      setDisplayCategoryLabel(it->first, it->second);
    }
  }

  sortDisplays();
}

void DisplaysPanel::readFromConfig(const boost::shared_ptr<Config>& config)
{
  std::string grid_state;
  if ( config->get( PROPERTY_GRID_CONFIG, &grid_state ) )
  {
    property_grid_->restoreEditableState( grid_state );
  }

  std::string sizes_string;
  if ( config->get( PROPERTY_GRID_SPLITTER, &sizes_string ) )
  {
    QList<int> sizes;

    UniformStringStream iss( sizes_string );
    int size;
    iss >> size;
    sizes.push_back( size );
    char c;
    iss >> c; // skip the ','
    iss >> size;
    sizes.push_back( size );
    tree_with_help_->setSizes( sizes );
  }
}

void DisplaysPanel::writeToConfig(const boost::shared_ptr<Config>& config)
{
  config->set( PROPERTY_GRID_CONFIG, property_grid_->saveEditableState() );
  QList<int> sizes = tree_with_help_->sizes();
  UniformStringStream sizes_stream;
  sizes_stream << sizes.at( 0 ) << ',' << sizes.at( 1 );
  config->set( PROPERTY_GRID_SPLITTER, sizes_stream.str() );
}

void DisplaysPanel::renumberDisplays()
{
  V_DisplayWrapper new_wrapper_list;

  int display_number = 0;
  display_map_.clear();
  for( int i = 0; i < property_grid_->topLevelItemCount(); i++ )
  {
    DisplayWrapper* wrapper = displayWrapperFromItem( property_grid_->topLevelItem( i ));
    if( wrapper )
    {
      setDisplayCategoryLabel( wrapper, display_number );
      display_map_[ wrapper ] = display_number;
      display_number++;

      new_wrapper_list.push_back( wrapper );
    }
  }

  // Swap our new vector of DisplayWrappers in for the original, so the order of Displays gets saved.
  V_DisplayWrapper& wrapper_list = manager_->getDisplays();
  wrapper_list.swap( new_wrapper_list );
  manager_->notifyConfigChanged();
}

} // namespace rviz
