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

#include "views_panel.h"

namespace rviz
{

ViewsPanel::ViewsPanel( QWidget* parent )
  : QWidget( parent )
  , manager_( NULL )
{
  camera_type_selector_ = new QComboBox;
  views_list_ = new QListWidget;

  QPushButton* save_button = new QPushButton( "Save Current" );
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
  button_layout->addWidget( save_button );
  button_layout->addWidget( load_button );
  button_layout->addWidget( delete_button );

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( top_layout );
  main_layout->addWidget( views_list_ );
  main_layout->addLayout( button_layout );
  setLayout( main_layout );

  connect( save_button, SIGNAL( clicked() ), this, SLOT( onSaveClicked() ));
  connect( load_button, SIGNAL( clicked() ), this, SLOT( loadSelected() ));
  connect( delete_button, SIGNAL( clicked() ), this, SLOT( onDeleteClicked() ));
  connect( zero_button, SIGNAL( clicked() ), this, SLOT( onZeroClicked() ));

  connect( camera_type_selector_, SIGNAL( activated( int )), this, SLOT( onCameraTypeSelected( int )));
  connect( views_list_, SIGNAL( itemActivated( QListWidgetItem* )), this, SLOT( loadSelected() ));
}

ViewsPanel::~ViewsPanel()
{
}

void ViewsPanel::initialize( VisualizationManager* manager )
{
  manager_ = manager;

  connect( manager_, SIGNAL( displaysConfigLoaded( const boost::shared_ptr<Config>& )),
           this, SLOT( readFromConfig( const boost::shared_ptr<Config>& )));
  connect( manager_, SIGNAL( displaysConfigSaved( const boost::shared_ptr<Config>& )),
           this, SLOT( writeToConfig( const boost::shared_ptr<Config>& )));
  connect( manager_, SIGNAL( viewControllerTypeAdded( const std::string&, const std::string& )),
           this, SLOT( onViewControllerTypeAdded( const std::string&, const std::string& )));
  connect( manager_, SIGNAL( viewControllerChanged( ViewController* )),
           this, SLOT( onViewControllerChanged( ViewController* )));
}

void ViewsPanel::loadSelected()
{
  int index = views_list_->currentRow();
  if( index >= 0 && index < (int) views_.size() )
  {
    const View& view = views_[ index ];
    manager_->setTargetFrame( view.target_frame_ );
    manager_->setCurrentViewControllerType( view.controller_class_ );
    manager_->getCurrentViewController()->fromString( view.controller_config_ );
    manager_->queueRender();
  }
}

void ViewsPanel::addView( const View& view )
{
  views_.push_back( view );

  std::stringstream ss;
  ss << view.name_
     << "; Target=[" << view.target_frame_
     << "] Type=[" << view.controller_class_
     << "] Config=[" << view.controller_config_ << "]";

  views_list_->addItem( QString::fromStdString( ss.str() ));
}

void ViewsPanel::save( const std::string& name )
{
  View view;
  view.target_frame_ = manager_->getTargetFrame();
  view.controller_class_ = manager_->getCurrentViewControllerType();
  view.name_ = name;
  view.controller_config_ = manager_->getCurrentViewController()->toString();

  addView( view );
  Q_EMIT configChanged();
}

void ViewsPanel::onViewControllerTypeAdded( const std::string& class_name, const std::string& name )
{
  camera_type_selector_->addItem( QString::fromStdString( name ), QString::fromStdString( class_name ));

  if( camera_type_selector_->count() == 1 )
  {
    camera_type_selector_->setCurrentIndex( 0 );
  }
}

void ViewsPanel::onViewControllerChanged( ViewController* controller )
{
  int count = camera_type_selector_->count();
  for( int i = 0; i < count; ++i )
  {
    QVariant type_var = camera_type_selector_->itemData( i );
    if( type_var.isValid() && controller->getClassName() == type_var.toString().toStdString() )
    {
      camera_type_selector_->setCurrentIndex( i );
      break;
    }
  }
}

void ViewsPanel::onCameraTypeSelected( int index )
{
  QVariant type_var = camera_type_selector_->itemData( index );
  if( type_var.isValid() )
  {
    manager_->setCurrentViewControllerType( type_var.toString().toStdString() );
  }
}

void ViewsPanel::onSaveClicked()
{
  bool ok;
  QString q_name = QInputDialog::getText( this, "Name the View", "Name",
                                          QLineEdit::Normal,
                                          "My View", &ok );
  if( ok && !q_name.isEmpty() )
  {
    save( q_name.toStdString() );
  }
}

void ViewsPanel::onZeroClicked()
{
  if( manager_->getCurrentViewController() )
  {
    manager_->getCurrentViewController()->reset();
  }
}

void ViewsPanel::onDeleteClicked()
{
  int index = views_list_->currentRow();
  if( index >= 0 && index < views_list_->count() )
  {
    views_.erase( views_.begin() + index );
    delete views_list_->item( index );
    Q_EMIT configChanged();
  }
}

void ViewsPanel::clear()
{
  views_.clear();
  views_list_->clear();
}

void ViewsPanel::readFromConfig( const boost::shared_ptr<Config>& config )
{
  clear();

  int i = 0;
  while( 1 )
  {
    std::stringstream type, target, cam_config, name;
    type << "Views/" << i << "/Type";
    target << "Views/" << i << "/Target";
    cam_config << "Views/" << i << "/Config";
    name << "Views/" << i << "Name";

    std::string view_type, view_name, view_target, view_config;
    if( !config->get( type.str(), &view_type ))
    {
      break;
    }

    if( !config->get( name.str(), &view_name ))
    {
      break;
    }

    if( !config->get( target.str(), &view_target ))
    {
      break;
    }

    if( !config->get( cam_config.str(), &view_config ))
    {
      break;
    }

    View view;
    view.name_ = view_name;
    view.controller_class_ = view_type;
    view.target_frame_ = view_target;
    view.controller_config_ = view_config;

    addView( view );

    ++i;
  }
}

void ViewsPanel::writeToConfig( const boost::shared_ptr<Config>& config )
{
  V_View::const_iterator it = views_.begin();
  V_View::const_iterator end = views_.end();
  uint32_t i = 0;
  for (; it != end; ++it, ++i)
  {
    const View& view = *it;

    std::stringstream type, target, cam_config, name;
    type << "Views/" << i << "/Type";
    target << "Views/" << i << "/Target";
    cam_config << "Views/" << i << "/Config";
    name << "Views/" << i << "Name";

    config->set( name.str(), view.name_ );
    config->set( type.str(), view.controller_class_ );
    config->set( target.str(), view.target_frame_ );
    config->set( cam_config.str(), view.controller_config_ );
  }
}

} // namespace rviz

