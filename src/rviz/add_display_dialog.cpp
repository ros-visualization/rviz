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
#include <QComboBox>

#include "add_display_dialog.h"
#include "rviz/load_resource.h"

#include "display_factory.h"

namespace rviz
{

// Utilities for grouping topics together

struct LexicalTopicInfo {
  bool operator()(const ros::master::TopicInfo &a, const ros::master::TopicInfo &b) {
    return a.name < b.name;
  }
};

/**
 * Return true if one topic is a subtopic of the other.
 *
 * A topic is a subtopic of another if a subset of its path exactly matches the
 * other.  For example, /camera/image_raw/compressed is a subtopic of
 * /camera/image_raw but not /camera/image.
 *
 * @param base A valid ROS topic
 *
 * @param topic A valid ROS topic
 *
 * @return True if topic is a subtopic of base.  False otherwise or if either
 *         argument is an invalid ROS topic.
 */
bool isSubtopic( const std::string &base, const std::string &topic )
{
  std::string error;
  if ( !ros::names::validate(base, error) )
  {
    ROS_ERROR_STREAM("isSubtopic() Invalid basename: " << error);
    return false;
  }
  if ( !ros::names::validate(topic, error) )
  {
    ROS_ERROR_STREAM("isSubtopic() Invalid topic: " << error);
    return false;
  }

  std::string query = topic;
  while ( query != "/" )
  {
    if ( query == base )
    {
      return true;
    }
    query = ros::names::parentNamespace( query );
  }
  return false;
}

/**
 * Combine topics into groups of subtopics.
 *
 * Each entry of subtopic_groups will contain a list of topics where the first
 * element is a topic and all subsequent items are subtopics of it.
 *
 * @param List of ROS topics.  The list is sorted lexicographically.
 *
 * @param subtopic_groups The resulting groups..
 */
void groupSubtopics( ros::master::V_TopicInfo *all_topics,
                     QList<QList<ros::master::TopicInfo> > *subtopic_groups )
{
  ros::master::V_TopicInfo::const_iterator it;
  std::sort( all_topics->begin(), all_topics->end(), LexicalTopicInfo() );

  QList<ros::master::TopicInfo> group;
  std::string base_topic;
  for ( it = all_topics->begin(); it != all_topics->end(); ++it )
  {
    std::string topic = it->name;

    if ( group.isEmpty() )
    {
      base_topic = topic;
    }
    else if ( !isSubtopic( base_topic, topic ) )
    {
      base_topic = topic;
      subtopic_groups->append( group );
      group.clear();
    }
    group.append( *it );
  }
  if ( !group.isEmpty() )
  {
    subtopic_groups->append( group );
  }
}

/**
 * Value class for groups of topics that can be viewed by the same plugins.
 *
 * Each instance has at least one topic and 0 or more plugins.  The first topic
 * of is a base topic; all other topics are subtopics of it.  All plugins can
 * visualize all of the topics.
 */
struct TopicGroup {
  QList<ros::master::TopicInfo> topics;
  QStringList plugins;
};

/** Helper function for combining plugin information with grouped subtopics to
 * produce list of topic_groups.
 */
void groupVisualizableTopics( const QMap<QString, QString> &datatype_plugins,
                              const QList<QList<ros::master::TopicInfo> > &subtopic_groups,
                              QList<TopicGroup> *topic_groups)
{
  for ( int i = 0; i < subtopic_groups.size(); ++i )
  {
    const QList<ros::master::TopicInfo> &subtopics = subtopic_groups.at( i );

    // Map from set of plugins to list of subtopics they can all visualize
    std::map<std::set<QString>, QList<ros::master::TopicInfo> > plugins_subtopics;
    // List of unvisualizable topics
    QList<ros::master::TopicInfo> unvisualizable;

    // Group topics that can all be visualized by same set of plugins
    for ( QList<ros::master::TopicInfo>::const_iterator it = subtopics.begin();
          it != subtopics.end(); ++it )
    {
      QString datatype = QString::fromStdString( it->datatype );
      QStringList plugins = datatype_plugins.values( datatype );
      if ( plugins.size() == 0)
      {
        unvisualizable.append( *it );
      }
      else
      {
        std::set<QString> plugin_set( plugins.begin(), plugins.end() );
        plugins_subtopics[plugin_set].append( *it );
      }
    }

    // Make unvisualizable separate topic groups
    for ( QList<ros::master::TopicInfo>::iterator it = unvisualizable.begin();
          it != unvisualizable.end(); ++it )
    {
      TopicGroup tg;
      tg.topics.append( *it );
      topic_groups->append( tg );
    }

    // Add groups
    std::map<std::set<QString>, QList<ros::master::TopicInfo> >::iterator it;
    for ( it = plugins_subtopics.begin(); it != plugins_subtopics.end(); ++it )
    {
      TopicGroup tg;
      const std::set<QString> &plugins = it->first;
      for ( std::set<QString>::iterator plugin_it = plugins.begin();
            plugin_it != plugins.end(); ++plugin_it )
      {
        tg.plugins.append( *plugin_it );
      }
      tg.topics = it->second;
      topic_groups->append( tg );
    }
  }
}

/**
 * Get groups of topics that are compatible with the same plugins.
 *
 * @param datatype_plugins Map from ROS topic type to plugin name.
 *
 * @param topic_groups The output variable.
 */
void getGroupedTopics( const QMap<QString, QString> &datatype_plugins,
                       QList<TopicGroup> *topic_groups )
{
  ros::master::V_TopicInfo all_topics;
  QList<QList<ros::master::TopicInfo> > subtopic_groups;

  ros::master::getTopics( all_topics );

  groupSubtopics( &all_topics, &subtopic_groups );

  groupVisualizableTopics( datatype_plugins, subtopic_groups, topic_groups );
}

// Dialog implementation
AddDisplayDialog::AddDisplayDialog( DisplayFactory* factory,
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

  tree_->setColumnCount( 2 );

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

    QTreeWidgetItem *parent = curr->parent();
    sd.topic = parent->text( 0 );

    QComboBox *combo = qobject_cast<QComboBox*>( tree_->itemWidget( parent, 1 ) );
    if ( combo != NULL )
    {
      QString combo_text = combo->currentText();
      if ( combo_text != "raw" )
      {
        sd.topic += "/" + combo_text;
      }
      sd.datatype = combo->itemData( combo->currentIndex() ).toString();
    }
    else
    {
      sd.datatype = curr->data( 0, Qt::UserRole ).toString();
    }
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

void TopicDisplayWidget::fill( DisplayFactory *factory )
{
  QMap<QString, QString> datatype_plugins;
  findPlugins( factory, &datatype_plugins );

  QList<TopicGroup> group;
  rviz::getGroupedTopics( datatype_plugins, &group );

  // Visualize all groups
  for( QList<TopicGroup>::iterator it = group.begin(); it != group.end(); ++it )
  {
    const TopicGroup &tg = *it;
    QString base_topic = QString::fromStdString( tg.topics.at( 0 ).name );

    QTreeWidgetItem *row = new QTreeWidgetItem( tree_ );
    row->setExpanded( true );
    row->setText( 0, base_topic );
    Qt::ItemFlags flags = row->flags();
    flags &= ~(Qt::ItemIsSelectable | Qt::ItemIsEditable);
    row->setFlags( flags );
    row->setExpanded( true );

    // If no plugins can visualize this group, disable it and move on.
    if ( tg.plugins.size() == 0 )
    {
      row->setHidden( !enable_hidden_box_->isChecked() );
      row->setDisabled( true );
      continue;
    }

    // If there's only one topic, store datatype in row.  Otherwise, let user
    // pick topic in a combo box and store datatype in each entry.
    if ( tg.topics.size() == 1 )
    {
      QString datatype = QString::fromStdString( tg.topics.at(0).datatype );
      row->setData( 0, Qt::UserRole, datatype );
    }
    else
    {
      QComboBox *box = new QComboBox;
      for ( int i = 0; i < tg.topics.size(); ++i )
      {
        QString datatype = QString::fromStdString( tg.topics.at( i ).datatype );
        QString label;

        if ( i == 0 )
        {
          label = "raw";
        }
        else
        {
          label = QString::fromStdString( tg.topics.at( i ).name );
          label.remove( 0, base_topic.size() + 1 );
        }
        box->addItem( label, datatype  );
      }
      tree_->setItemWidget( row, 1, box );
    }

    // Add all compatible plugins as child of previous of row.
    QStringList::const_iterator plugin_it;
    for ( plugin_it = tg.plugins.begin(); plugin_it != tg.plugins.end(); ++plugin_it )
    {
      const QString &plugin_name = *plugin_it;

      QTreeWidgetItem *plugin = new QTreeWidgetItem( row );
      plugin->setText( 0, factory->getClassName( plugin_name ) );
      plugin->setIcon( 0, factory->getIcon( plugin_name ) );
      plugin->setWhatsThis( 0, factory->getClassDescription( plugin_name ) );
      plugin->setData( 0, Qt::UserRole, plugin_name );
    }
  }

  // Display items alphabetically
  tree_->sortItems( 0, Qt::AscendingOrder );
  // Formatting for long names
  tree_->resizeColumnToContents( 0 );
}

void TopicDisplayWidget::findPlugins( DisplayFactory *factory,
                                      QMap<QString, QString> *datatype_plugins )
{
  // Build map from topic type to plugin by instantiating every plugin we have.
  QStringList lookup_names = factory->getDeclaredClassIds();

  // Explicitly ignore plugins that take forever to instantiate.  This is OK,
  // because right now, none of these work with selection by topic.
  QSet<QString> blacklist;
  blacklist.insert("rviz/DepthCloud");
  QStringList::iterator it;
  for (it = lookup_names.begin(); it != lookup_names.end(); ++it)
  {
    const QString &lookup_name = *it;
    // ROS_INFO("Class: %s", lookup_name.toStdString().c_str());
    if (blacklist.contains(lookup_name))
    {
      continue;
    }

    boost::scoped_ptr<Display> disp( factory->make( lookup_name ));

    QSet<QString> topic_types = disp->getROSTopicTypes();
    Q_FOREACH( QString topic_type, topic_types )
    {
      // ROS_INFO("Type: %s", topic_type.toStdString().c_str());
      datatype_plugins->insertMulti( topic_type, lookup_name );
    }
  }
}

} // rviz
