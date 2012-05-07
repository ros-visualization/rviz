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
#include "new_object_dialog.h"
#include "properties/property.h"
#include "properties/property_tree_with_help.h"
#include "properties/property_tree_widget.h"

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
  connect( property_grid_, SIGNAL( selectionChanged() ), this, SLOT( onSelectionChanged() ));
}

DisplaysPanel::~DisplaysPanel()
{
}

void DisplaysPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;
  property_grid_->setModel( manager_->getDisplayTreeModel() );
}

void DisplaysPanel::onNewDisplay()
{
  // Get the list of current display names, so we can enforce that the
  // new display has a unique name.
  S_string current_display_names;
  ///// manager_->getDisplayNames(current_display_names); // don't know if i want this back or not.

  std::string lookup_name;
  std::string display_name;

  NewObjectDialog* dialog = new NewObjectDialog( manager_->getDisplayClassLoader(),
                                                 "Display",
                                                 current_display_names,
                                                 std::set<std::string>(),
                                                 &lookup_name,
                                                 &display_name );
  if( dialog->exec() == QDialog::Accepted )
  {
    manager_->createDisplay( QString::fromStdString( lookup_name ), QString::fromStdString( display_name ), true );
  }
  activateWindow(); // Force keyboard focus back on main window.
}

void DisplaysPanel::onDeleteDisplay()
{
  QList<Display*> displays_to_delete = property_grid_->getSelectedObjects<Display>();

  for( int i = 0; i < displays_to_delete.size(); i++ )
  {
    delete displays_to_delete[ i ];
  }
}

void DisplaysPanel::onSelectionChanged()
{
  QList<Display*> displays = property_grid_->getSelectedObjects<Display>();

  int num_displays_selected = displays.size();

  remove_button_->setEnabled( num_displays_selected > 0 );
  rename_button_->setEnabled( num_displays_selected == 1 );
}

void DisplaysPanel::onRenameDisplay()
{
  QList<Display*> displays = property_grid_->getSelectedObjects<Display>();
  if( displays.size() == 0 )
  {
    return;
  }
  Display* display_to_rename = displays[ 0 ];

  if( !display_to_rename )
  {
    return;
  }

  QString old_name = display_to_rename->getName();
  QString new_name = QInputDialog::getText( this, "Rename Display", "New Name?", QLineEdit::Normal, old_name );

  if( new_name.isEmpty() || new_name == old_name )
  {
    return;
  }

  display_to_rename->setName( new_name );
}

///// void DisplaysPanel::readFromConfig(const boost::shared_ptr<Config>& config)
///// {
/////   std::string grid_state;
/////   if ( config->get( PROPERTY_GRID_CONFIG, &grid_state ) )
/////   {
/////     property_grid_->restoreEditableState( grid_state );
/////   }
///// 
/////   std::string sizes_string;
/////   if ( config->get( PROPERTY_GRID_SPLITTER, &sizes_string ) )
/////   {
/////     QList<int> sizes;
///// 
/////     UniformStringStream iss( sizes_string );
/////     int size;
/////     iss >> size;
/////     sizes.push_back( size );
/////     char c;
/////     iss >> c; // skip the ','
/////     iss >> size;
/////     sizes.push_back( size );
/////     tree_with_help_->setSizes( sizes );
/////   }
///// }
///// 
///// void DisplaysPanel::writeToConfig(const boost::shared_ptr<Config>& config)
///// {
/////   config->set( PROPERTY_GRID_CONFIG, property_grid_->saveEditableState() );
/////   QList<int> sizes = tree_with_help_->sizes();
/////   UniformStringStream sizes_stream;
/////   sizes_stream << sizes.at( 0 ) << ',' << sizes.at( 1 );
/////   config->set( PROPERTY_GRID_SPLITTER, sizes_stream.str() );
///// }

} // namespace rviz
