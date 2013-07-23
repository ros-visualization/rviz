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

#include <map>

#include <boost/filesystem.hpp>

#include <ros/package.h>
#include <ros/ros.h>

#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QTabWidget>
#include <QCheckBox>

#include "add_display_dialog.h"
#include "rviz/load_resource.h"

#include "display_factory.h"

namespace rviz
{

AddDisplayDialog::AddDisplayDialog( Factory* factory,
                                    const QString& object_type,
                                    const QStringList& disallowed_display_names,
                                    const QStringList& disallowed_class_lookup_names,
                                    QString* lookup_name_output,
                                    QString* display_name_output,
                                    QString* topic_output,
                                    QString* datatype_output,
                                    QWidget* parent )
: QDialog( parent )
, factory_( factory )
, disallowed_display_names_( disallowed_display_names )
, disallowed_class_lookup_names_( disallowed_class_lookup_names )
, lookup_name_output_( lookup_name_output )
, display_name_output_( display_name_output )
, topic_output_( topic_output )
, datatype_output_( datatype_output )
{
  //***** Layout

  // Display Type group
  QGroupBox* type_box = new QGroupBox( "Create visualization" );

  QLabel* description_label = new QLabel( "Description:" );
  description_ = new QTextBrowser;
  description_->setMaximumHeight( 100 );
  description_->setOpenExternalLinks( true );

  DisplayTypeTree *display_tree = new DisplayTypeTree;
  display_tree->fillTree(factory);

  TopicDisplayWidget *topic_widget = new TopicDisplayWidget;
  topic_widget->fill(factory);

  tab_widget_ = new QTabWidget;
  topic_tab_ = tab_widget_->addTab( topic_widget, tr("By topic") );
  display_tab_ = tab_widget_->addTab( display_tree, tr("By display type") );

  QVBoxLayout *type_layout = new QVBoxLayout;
  type_layout->addWidget( tab_widget_ );
  type_layout->addWidget( description_label );
  type_layout->addWidget( description_ );

  type_box->setLayout( type_layout );

  // Display Name group
  QGroupBox* name_box;
  if( display_name_output_ )
  {
    name_box = new QGroupBox( "Display Name" );
    name_editor_ = new QLineEdit;
    QVBoxLayout* name_layout = new QVBoxLayout;
    name_layout->addWidget( name_editor_ );
    name_box->setLayout( name_layout );
  }

  // Buttons
  button_box_ = new QDialogButtonBox( QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                      Qt::Horizontal );

  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( type_box );
  if( display_name_output_ )
  {
    main_layout->addWidget( name_box );
  }
  main_layout->addWidget( button_box_ );
  setLayout( main_layout );

  //***** Connections
  connect( display_tree, SIGNAL( itemChanged( SelectionData* )),
           this, SLOT( onDisplaySelected( SelectionData* )));
  connect( display_tree, SIGNAL( itemActivated( QTreeWidgetItem*, int )),
           this, SLOT( accept() ));

  connect( topic_widget, SIGNAL( itemChanged( SelectionData* )),
           this, SLOT( onTopicSelected( SelectionData* )));
  connect( topic_widget, SIGNAL( itemActivated( QTreeWidgetItem*, int )),
           this, SLOT( accept() ));

  connect( button_box_, SIGNAL( accepted() ), this, SLOT( accept() ));
  connect( button_box_, SIGNAL( rejected() ), this, SLOT( reject() ));

  connect( tab_widget_, SIGNAL( currentChanged( int ) ),
           this, SLOT( onTabChanged( int ) ));
  if( display_name_output_ )
  {
    connect( name_editor_, SIGNAL( textEdited( const QString& )),
             this, SLOT( onNameChanged() ));
  }

  button_box_->button( QDialogButtonBox::Ok )->setEnabled( isValid() );
}

QSize AddDisplayDialog::sizeHint () const
{
  return( QSize(500,660) );
}

void AddDisplayDialog::onTabChanged( int index )
{
  updateDisplay();
}

void AddDisplayDialog::onDisplaySelected( SelectionData *data)
{
  display_data_ = *data;
  updateDisplay();
}

void AddDisplayDialog::onTopicSelected( SelectionData *data )
{
  topic_data_ = *data;
  updateDisplay();
}

void AddDisplayDialog::updateDisplay()
{
  SelectionData *data = NULL;
  if ( tab_widget_->currentIndex() == topic_tab_ )
  {
    data = &topic_data_;
  }
  else if ( tab_widget_->currentIndex() == display_tab_ )
  {
    data = &display_data_;
  }
  else
  {
    ROS_WARN("Unknown tab index: %i", tab_widget_->currentIndex());
    return;
  }

  QString html = "<html><body>" + data->whats_this + "</body></html>";
  description_->setHtml( html );

  lookup_name_ = data->lookup_name;
  if( display_name_output_ )
  {
    name_editor_->setText( data->display_name );
  }

  *topic_output_ = data->topic;
  *datatype_output_ = data->datatype;

  button_box_->button( QDialogButtonBox::Ok )->setEnabled( isValid() );
}

bool AddDisplayDialog::isValid()
{
  if( lookup_name_.size() == 0 )
  {
    setError( "Select a Display type." );
    return false;
  }
  if( display_name_output_ )
  {
    QString display_name = name_editor_->text();
    if( display_name.size() == 0 )
    {
      setError( "Enter a name for the display." );
      return false;
    }
    if( disallowed_display_names_.contains( display_name ))
    {
      setError( "Name in use.  Display names must be unique." );
      return false;
    }
  }
  setError( "" );
  return true;
}

void AddDisplayDialog::setError( const QString& error_text )
{
  button_box_->button( QDialogButtonBox::Ok )->setToolTip( error_text );
}

void AddDisplayDialog::onNameChanged()
{
  button_box_->button( QDialogButtonBox::Ok )->setEnabled( isValid() );
}

void AddDisplayDialog::accept()
{
  if( isValid() )
  {
    *lookup_name_output_ = lookup_name_;
    if( display_name_output_ )
    {
      *display_name_output_ = name_editor_->text();
    }
    QDialog::accept();
  }
}

DisplayTypeTree::DisplayTypeTree()
{
  setHeaderHidden( true );

  connect(this, SIGNAL( currentItemChanged( QTreeWidgetItem*, QTreeWidgetItem* )),
          this, SLOT( onCurrentItemChanged( QTreeWidgetItem*, QTreeWidgetItem* )));
}

void DisplayTypeTree::onCurrentItemChanged(QTreeWidgetItem *curr,
                                           QTreeWidgetItem *prev)
{
  // If display is selected, populate selection data.  Otherwise, clear data.
  SelectionData sd;
  if ( curr->parent() != NULL )
  {
    // Leave topic and datatype blank
    sd.whats_this = curr->whatsThis( 0 );
    sd.lookup_name = curr->data( 0, Qt::UserRole).toString();
    sd.display_name = curr->text( 0 );
  }
  Q_EMIT itemChanged( &sd );
}

void DisplayTypeTree::fillTree( Factory *factory )
{
    QIcon default_package_icon = loadPixmap( "package://rviz/icons/default_package_icon.png" );

    QStringList classes = factory->getDeclaredClassIds();
    classes.sort();

    // Map from package names to the corresponding top-level tree widget items.
    std::map<QString, QTreeWidgetItem*> package_items;

    for( int i = 0; i < classes.size(); i++ )
    {
      QString lookup_name = classes[ i ];
      QString package = factory->getClassPackage( lookup_name );
      QString description = factory->getClassDescription( lookup_name );
      QString name = factory->getClassName( lookup_name );

      QTreeWidgetItem* package_item;

      std::map<QString, QTreeWidgetItem*>::iterator mi;
      mi = package_items.find( package );
      if( mi == package_items.end() )
      {
        package_item = new QTreeWidgetItem( this );
        package_item->setText( 0, package );
        package_item->setIcon( 0, default_package_icon );

        package_item->setExpanded( true );
        package_items[ package ] = package_item;
      }
      else
      {
        package_item = (*mi).second;
      }
      QTreeWidgetItem* class_item = new QTreeWidgetItem( package_item );

      class_item->setIcon( 0, factory->getIcon( lookup_name ) );

      class_item->setText( 0, name );
      class_item->setWhatsThis( 0, description );
      // Store the lookup name for each class in the UserRole of the item.
      class_item->setData( 0, Qt::UserRole, lookup_name );
    }
}

TopicDisplayWidget::TopicDisplayWidget()
{
  tree_ = new QTreeWidget;
  tree_->setHeaderHidden( true );
  // Hide check boxes for topics
  tree_->setRootIsDecorated( false );
  // Don't let users collapse topics
  tree_->setItemsExpandable( false );

  enable_hidden_box_ = new QCheckBox( "Show unvisualizable topics" );
  enable_hidden_box_->setCheckState( Qt::Unchecked );

  QVBoxLayout *layout = new QVBoxLayout;
  layout->setContentsMargins( QMargins( 0, 0, 0, 0 ) );

  layout->addWidget( tree_ );
  layout->addWidget( enable_hidden_box_ );

  // Forward signals from tree_
  connect( tree_, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
           this, SLOT(onCurrentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)) );
  connect( tree_, SIGNAL(itemActivated(QTreeWidgetItem*, int)),
           this, SIGNAL(itemActivated(QTreeWidgetItem*, int)) );

  // Connect signal from checkbox
  connect( enable_hidden_box_, SIGNAL(stateChanged(int)),
           this, SLOT(stateChanged(int)) );

  setLayout( layout );
}

void TopicDisplayWidget::onCurrentItemChanged(QTreeWidgetItem* curr,
                                              QTreeWidgetItem* prev)
{
  // If topic is selected, populate selection data.  Otherwise, clear data.
  SelectionData sd;
  if (curr->parent() != NULL)
  {
    sd.whats_this = curr->whatsThis( 0 );
    sd.lookup_name = curr->data( 0, Qt::UserRole ).toString();
    sd.display_name = curr->text( 0 );

    QStringList topic_datatype = curr->data( 1, Qt::UserRole ).toStringList();
    sd.topic = topic_datatype[0];
    sd.datatype = topic_datatype[1];
  }
  Q_EMIT itemChanged( &sd );
}

void TopicDisplayWidget::stateChanged( int state )
{
  bool hide_nomarker = state == Qt::Unchecked;
  for (int i = 0; i < tree_->topLevelItemCount(); ++i)
  {
    QTreeWidgetItem *topic = tree_->topLevelItem( i );
    if (topic->childCount() == 0)
    {
      topic->setHidden( hide_nomarker );
    }
  }
}

void TopicDisplayWidget::fill( Factory *factory )
{
  QMap<QString, QString> datatype_plugins;
  findPlugins( &datatype_plugins );

  // Loop through all published topics
  ros::master::V_TopicInfo topics;
  ros::master::getTopics( topics );
  ros::master::V_TopicInfo::iterator it;
  for( it = topics.begin(); it != topics.end(); ++it )
  {
    const ros::master::TopicInfo& topic_info = *it;
    QString datatype = QString::fromStdString(topic_info.datatype);
    QString topic = QString::fromStdString(topic_info.name);

    QTreeWidgetItem *topic_item = new QTreeWidgetItem(tree_);
    topic_item->setText( 0, topic + QString(" (") + datatype + QString(")") );
    topic_item->setWhatsThis( 0, datatype );

    Qt::ItemFlags flags = topic_item->flags();
    flags &= ~(Qt::ItemIsSelectable | Qt::ItemIsEditable);
    topic_item->setFlags( flags );
    topic_item->setExpanded( true );

    if (datatype_plugins.contains(datatype))
    {
      // Add all compatible plugins to output along with topic type
      const QList<QString> &plugins = datatype_plugins.values(datatype);
      for (int i = 0; i < plugins.size(); ++i)
      {
        const QString &plugin_name = plugins[i];

        QTreeWidgetItem *plugin = new QTreeWidgetItem( topic_item );
        plugin->setText( 0, factory->getClassName(plugin_name) );
        plugin->setIcon( 0, factory->getIcon(plugin_name) );
        plugin->setWhatsThis( 0, factory->getClassDescription(plugin_name) );
        plugin->setData( 0, Qt::UserRole, plugin_name );
        QStringList topic_datatype;
        topic_datatype.append(topic);
        topic_datatype.append(datatype);
        plugin->setData( 1, Qt::UserRole, topic_datatype );
      }
    }
    else
    {
      topic_item->setHidden( !enable_hidden_box_->isChecked() );
      topic_item->setDisabled( true );
    }
  }

  // Display items alphabetically
  tree_->sortItems( 0, Qt::AscendingOrder );
}

void TopicDisplayWidget::findPlugins( QMap<QString, QString> *datatype_plugins )
{
  // Build map from topic type to plugin by instantiating every plugin we have.
  pluginlib::ClassLoader<Display> loader( "rviz", "rviz::Display" );
  std::vector<std::string> lookup_names = loader.getDeclaredClasses();

  // Explicitly ignore plugins that take forever to instantiate.  This is OK,
  // because right now, none of these work with selection by topic.
  QSet<QString> blacklist;
  blacklist.insert("rviz/DepthCloud");
  for (int i = 0; i < lookup_names.size(); ++i)
  {
    QString lookup_name = QString::fromStdString( lookup_names[i] );
    // ROS_INFO("Class: %s", lookup_name.toStdString().c_str());
    if (blacklist.contains(lookup_name))
    {
      continue;
    }

    // This is a memory leak, but many plugins cannot be deleted without being
    // initialized and the data to properly initialize each plugin isn't here.
    Display* disp = loader.createUnmanagedInstance( lookup_name.toStdString() );

    QSet<QString> topic_types = disp->getROSTopicTypes();
    Q_FOREACH( QString topic_type, topic_types )
    {
      // ROS_INFO("Type: %s", topic_type.toStdString().c_str());
      datatype_plugins->insertMulti( topic_type, lookup_name );
    }
  }
}

} // rviz
