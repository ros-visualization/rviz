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

#include <QLabel>
#include <QListWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>

#include <boost/bind.hpp>

#include "visualization_manager.h"
#include "view_controller.h"
#include "config.h"
#include "rviz/view_manager.h"
#include "rviz/properties/property_tree_widget.h"

#include "views_panel.h"

namespace rviz
{

ViewsPanel::ViewsPanel( QWidget* parent )
  : QWidget( parent )
  , manager_( NULL )
{
  camera_type_selector_ = new QComboBox;
  properties_view_ = new PropertyTreeWidget();

  copy_button_ = new QPushButton( "Copy" );
  QPushButton* load_button = new QPushButton( "Load" );
  QPushButton* delete_button = new QPushButton( "Delete" );
  QPushButton* zero_button = new QPushButton( "Zero" );
  zero_button->setToolTip( "Jump to 0,0,0 with the current view controller. Shortcut: Z" );

  QHBoxLayout* top_layout = new QHBoxLayout;
  top_layout->addWidget( new QLabel( "Type:" ));
  top_layout->addWidget( camera_type_selector_ );
  top_layout->addStretch();
  top_layout->addWidget( zero_button );

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( copy_button_ );
  button_layout->addWidget( load_button );
  button_layout->addWidget( delete_button );

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( top_layout );
  main_layout->addWidget( properties_view_ );
  main_layout->addLayout( button_layout );
  setLayout( main_layout );

//  connect( load_button, SIGNAL( clicked() ), this, SLOT( loadSelected() ));
  connect( delete_button, SIGNAL( clicked() ), this, SLOT( onDeleteClicked() ));
  connect( zero_button, SIGNAL( clicked() ), this, SLOT( onZeroClicked() ));

  connect( camera_type_selector_, SIGNAL( activated( int )), this, SLOT( onCameraTypeSelected( int )));
//  connect( views_list_, SIGNAL( itemActivated( QListWidgetItem* )), this, SLOT( loadSelected() ));
  connect( properties_view_, SIGNAL( clicked( const QModelIndex& )), this, SLOT( onItemClicked( const QModelIndex& )));
}

ViewsPanel::~ViewsPanel()
{
}

void ViewsPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;

  properties_view_->setModel( manager_->getViewManager()->getPropertyModel() );

/////  connect( manager_, SIGNAL( displaysConfigLoaded( const boost::shared_ptr<Config>& )),
/////           this, SLOT( readFromConfig( const boost::shared_ptr<Config>& )));
/////  connect( manager_, SIGNAL( displaysConfigSaved( const boost::shared_ptr<Config>& )),
/////           this, SLOT( writeToConfig( const boost::shared_ptr<Config>& )));
  connect( manager_->getViewManager(), SIGNAL( viewControllerTypeAdded( const std::string&, const std::string& )),
           this, SLOT( onViewControllerTypeAdded( const std::string&, const std::string& )));
  connect( manager_->getViewManager(), SIGNAL( currentChanged( ViewController* )),
           this, SLOT( onViewControllerChanged( ViewController* )));

  connect( copy_button_, SIGNAL( clicked() ), manager_->getViewManager(), SLOT( copyCurrent() ));
}

void ViewsPanel::onViewControllerTypeAdded( const std::string& class_name, const std::string& name )
{
  camera_type_selector_->addItem( QString::fromStdString( name ), QString::fromStdString( class_name ));

  if( camera_type_selector_->count() == 1 )
  {
    camera_type_selector_->setCurrentIndex( 0 );
  }
}

void ViewsPanel::onViewControllerChanged( ViewController* new_current )
{
  // Update the item showing in the type selector combo-box.
  int count = camera_type_selector_->count();
  for( int i = 0; i < count; ++i )
  {
    QVariant type_var = camera_type_selector_->itemData( i );
    if( type_var.isValid() && new_current->getClassName() == type_var.toString().toStdString() )
    {
      camera_type_selector_->setCurrentIndex( i );
      break;
    }
  }

  // Expand the new current view controller and collapse all others.
  ViewManager* vman = manager_->getViewManager();
  for( int i = 0; i < vman->getNumViews(); i++ )
  {
    ViewController* view = vman->getViewAt( i );
    if( view == new_current )
    {
      view->expand();
    }
    else
    {
      view->collapse();
    }
  }
}

void ViewsPanel::onCameraTypeSelected( int index )
{
  QVariant type_var = camera_type_selector_->itemData( index );
  if( type_var.isValid() )
  {
    manager_->getViewManager()->setCurrentViewControllerType( type_var.toString().toStdString() );
  }
}

void ViewsPanel::onZeroClicked()
{
  if( manager_->getViewManager()->getCurrent() )
  {
    manager_->getViewManager()->getCurrent()->reset();
  }
}

void ViewsPanel::onItemClicked( const QModelIndex& index )
{
  Property* prop = manager_->getViewManager()->getPropertyModel()->getProp( index );
  if( ViewController* view = qobject_cast<ViewController*>( prop ))
  {
    manager_->getViewManager()->setCurrent( view );
  }
}

void ViewsPanel::onDeleteClicked()
{
/////  int index = views_list_->currentRow();
/////  if( index >= 0 && index < views_list_->count() )
/////  {
/////    views_.erase( views_.begin() + index );
/////    delete views_list_->item( index );
/////    Q_EMIT configChanged();
/////  }
}

void ViewsPanel::clear()
{
//  views_.clear();
/////  views_list_->clear();
}

void ViewsPanel::readFromConfig( const boost::shared_ptr<Config>& config )
{
}

void ViewsPanel::writeToConfig( const boost::shared_ptr<Config>& config )
{
}

} // namespace rviz

