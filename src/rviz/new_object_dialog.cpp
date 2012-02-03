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

#include <QGroupBox>
#include <QTreeWidget>
#include <QLabel>
#include <QLineEdit>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>

#include "new_object_dialog.h"

namespace rviz
{

NewObjectDialog::NewObjectDialog( pluginlib::ClassLoaderBase* class_loader,
                                  const std::string& object_type,
                                  const S_string& current_display_names,
                                  std::string* lookup_name_output,
                                  std::string* display_name_output,
                                  QWidget* parent )
: QDialog( parent )
, class_loader_( class_loader )
, current_display_names_( current_display_names )
, lookup_name_output_( lookup_name_output )
, display_name_output_( display_name_output )
{
  //***** Layout

  // Display Type group
  QGroupBox* type_box = new QGroupBox( QString::fromStdString( object_type + " Type" ));
  
  QTreeWidget* tree = new QTreeWidget;
  tree->setHeaderHidden( true );
  fillTree( tree );

  QLabel* description_label = new QLabel( "Description:" );
  description_ = new QTextBrowser;
  description_->setMaximumHeight( 100 );
  description_->setOpenExternalLinks( true );

  QVBoxLayout* type_layout = new QVBoxLayout;
  type_layout->addWidget( tree );
  type_layout->addWidget( description_label );
  type_layout->addWidget( description_ );

  type_box->setLayout( type_layout );

  // Display Name group
  QGroupBox* name_box = new QGroupBox( QString::fromStdString( object_type + " Name" ));
  
  name_editor_ = new QLineEdit;
  QVBoxLayout* name_layout = new QVBoxLayout;
  name_layout->addWidget( name_editor_ );
  name_box->setLayout( name_layout );

  // Buttons
  button_box_ = new QDialogButtonBox( QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                      Qt::Horizontal );

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( type_box );
  main_layout->addWidget( name_box );
  main_layout->addWidget( button_box_ );
  setLayout( main_layout );

  //***** Connections
  connect( tree, SIGNAL( currentItemChanged( QTreeWidgetItem*, QTreeWidgetItem* )),
           this, SLOT( onDisplaySelected( QTreeWidgetItem* )));
  connect( tree, SIGNAL( itemActivated( QTreeWidgetItem*, int )),
           this, SLOT( accept() ));
  connect( button_box_, SIGNAL( accepted() ), this, SLOT( accept() ));
  connect( button_box_, SIGNAL( rejected() ), this, SLOT( reject() ));
  connect( name_editor_, SIGNAL( textEdited( const QString& )),
           this, SLOT( onNameChanged() ));
}

void NewObjectDialog::fillTree( QTreeWidget* tree )
{
  std::vector<std::string> classes = class_loader_->getDeclaredClasses();

  // Map from package names to the corresponding top-level tree widget items.
  std::map<std::string, QTreeWidgetItem*> package_items;

  std::vector<std::string>::const_iterator ci;
  for( ci = classes.begin(); ci != classes.end(); ci++ )
  {
    std::string lookup_name = (*ci);
    std::string package = class_loader_->getClassPackage( lookup_name );
    std::string description = class_loader_->getClassDescription( lookup_name );
    std::string name = class_loader_->getName( lookup_name );

    QTreeWidgetItem* package_item;

    std::map<std::string, QTreeWidgetItem*>::iterator mi;
    mi = package_items.find( package );
    if( mi == package_items.end() )
    {
      package_item = new QTreeWidgetItem( tree );
      package_item->setText( 0, QString::fromStdString( package ));
      package_item->setExpanded( true );
      package_items[ package ] = package_item;
    }
    else
    {
      package_item = (*mi).second;
    }
    QTreeWidgetItem* class_item = new QTreeWidgetItem( package_item );
    class_item->setText( 0, QString::fromStdString( name ));
    class_item->setWhatsThis( 0, QString::fromStdString( description ));
    // Store the lookup name for each class in the UserRole of the item.
    class_item->setData( 0, Qt::UserRole, QString::fromStdString( lookup_name ));
  }
}

void NewObjectDialog::onDisplaySelected( QTreeWidgetItem* selected_item )
{
  QString html = "<html><body>" + selected_item->whatsThis( 0 ) + "</body></html>";
  description_->setHtml( html );

  // We stored the lookup name for the class in the UserRole of the items.
  QVariant user_data = selected_item->data( 0, Qt::UserRole );
  bool selection_is_valid = user_data.isValid();
  if( selection_is_valid )
  {
    lookup_name_ = user_data.toString().toStdString();
    std::string display_name = selected_item->text( 0 ).toStdString();

    int counter = 1;
    std::string name;
    do
    {
      std::stringstream ss;
      ss << display_name;
 
      if( counter > 1 )
      {
        ss << counter;
      }
 
      ++counter;
 
      name = ss.str();
    } while( current_display_names_.find( name ) != current_display_names_.end() );
 
    name_editor_->setText( QString::fromStdString( name ));
  }
  else
  {
    lookup_name_ = "";
    name_editor_->setText( "" );
  }
  button_box_->button( QDialogButtonBox::Ok )->setEnabled( isValid() );
}

bool NewObjectDialog::isValid()
{
  std::string display_name = name_editor_->text().toStdString();
  if( lookup_name_.size() == 0 )
  {
    setError( "Select a Display type." );
    return false;
  }
  if( display_name.size() == 0 )
  {
    setError( "Enter a name for the display." );
    return false;
  }
  if( current_display_names_.find( display_name ) != current_display_names_.end() )
  {
    setError( "Name in use.  Display names must be unique." );
    return false;
  }
  setError( "" );
  return true;
}

void NewObjectDialog::setError( const QString& error_text )
{
  button_box_->button( QDialogButtonBox::Ok )->setToolTip( error_text );
}

void NewObjectDialog::onNameChanged()
{
  button_box_->button( QDialogButtonBox::Ok )->setEnabled( isValid() );
}

void NewObjectDialog::accept()
{
  if( isValid() )
  {
    *lookup_name_output_ = lookup_name_;
    *display_name_output_ = name_editor_->text().toStdString();
    QDialog::accept();
  }
}

} // rviz

