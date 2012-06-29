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

  save_button_ = new QPushButton( "Save" );
  QPushButton* remove_button = new QPushButton( "Remove" );
  QPushButton* rename_button = new QPushButton( "Rename" );
  QPushButton* zero_button = new QPushButton( "Zero" );
  zero_button->setToolTip( "Jump to 0,0,0 with the current view controller. Shortcut: Z" );

  QHBoxLayout* top_layout = new QHBoxLayout;
  top_layout->addWidget( new QLabel( "Type:" ));
  top_layout->addWidget( camera_type_selector_ );
  top_layout->addStretch();
  top_layout->addWidget( zero_button );

  QHBoxLayout* button_layout = new QHBoxLayout;
  button_layout->addWidget( save_button_ );
  button_layout->addWidget( remove_button );
  button_layout->addWidget( rename_button );

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( top_layout );
  main_layout->addWidget( properties_view_ );
  main_layout->addLayout( button_layout );
  setLayout( main_layout );

  connect( remove_button, SIGNAL( clicked() ), this, SLOT( onDeleteClicked() ));
  connect( rename_button, SIGNAL( clicked() ), this, SLOT( renameSelected() ));
  connect( zero_button, SIGNAL( clicked() ), this, SLOT( onZeroClicked() ));
  connect( properties_view_, SIGNAL( clicked( const QModelIndex& )), this, SLOT( setCurrentViewFromIndex( const QModelIndex& )));
  connect( properties_view_, SIGNAL( activated( const QModelIndex& )), this, SLOT( setCurrentViewFromIndex( const QModelIndex& )));
}

ViewsPanel::~ViewsPanel()
{
}

void ViewsPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;

  properties_view_->setModel( manager_->getViewManager()->getPropertyModel() );

  connect( save_button_, SIGNAL( clicked() ), manager_->getViewManager(), SLOT( copyCurrentToList() ));

  camera_type_selector_->addItems( manager_->getViewManager()->getFactory()->getDeclaredClassIds() );

  connect( camera_type_selector_, SIGNAL( activated( const QString& )),
           manager_->getViewManager(), SLOT( setCurrentViewControllerType( const QString& )));
  connect( manager_->getViewManager(), SIGNAL( currentChanged() ),
           this, SLOT( onCurrentChanged() ));
}

void ViewsPanel::onZeroClicked()
{
  if( manager_->getViewManager()->getCurrent() )
  {
    manager_->getViewManager()->getCurrent()->reset();
  }
}

void ViewsPanel::setCurrentViewFromIndex( const QModelIndex& index )
{
  Property* prop = manager_->getViewManager()->getPropertyModel()->getProp( index );
  if( ViewController* view = qobject_cast<ViewController*>( prop ))
  {
    manager_->getViewManager()->setCurrentFrom( view );
  }
}

void ViewsPanel::onDeleteClicked()
{
  QList<ViewController*> views_to_delete = properties_view_->getSelectedObjects<ViewController>();

  for( int i = 0; i < views_to_delete.size(); i++ )
  {
    // TODO: should eventually move to a scheme where the CURRENT view
    // is not in the same list as the saved views, at which point this
    // check can go away.
    if( views_to_delete[ i ] != manager_->getViewManager()->getCurrent() )
    {
      delete views_to_delete[ i ];
    }
  }
}

void ViewsPanel::renameSelected()
{
  QList<ViewController*> views_to_rename = properties_view_->getSelectedObjects<ViewController>();
  if( views_to_rename.size() == 1 )
  {
    ViewController* view = views_to_rename[ 0 ];

    // TODO: should eventually move to a scheme where the CURRENT view
    // is not in the same list as the saved views, at which point this
    // check can go away.
    if( view == manager_->getViewManager()->getCurrent() )
    {
      return;
    }

    QString old_name = view->getName();
    QString new_name = QInputDialog::getText( this, "Rename View", "New Name?", QLineEdit::Normal, old_name );

    if( new_name.isEmpty() || new_name == old_name )
    {
      return;
    }

    view->setName( new_name );
  }
}

void ViewsPanel::onCurrentChanged()
{
  // Make sure the type selector shows the type of the current view.
  // This is only here in case the type is changed programmatically,
  // instead of via the camera_type_selector_ being used.
  camera_type_selector_->setCurrentIndex( camera_type_selector_->findText( manager_->getViewManager()->getCurrent()->getClassId() ));

  manager_->getViewManager()->getCurrent()->expand();
}

} // namespace rviz

