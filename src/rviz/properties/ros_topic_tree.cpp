/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <sstream>

#include "rviz/properties/ros_topic_tree.h"
#include "ros/master.h"

#include <QTimer>

namespace rviz
{

class TopicItem: public QTreeWidgetItem
{
public:
  TopicItem( QTreeWidgetItem* parent = 0 )
    : QTreeWidgetItem( parent )
    {}

  void setSelectable( bool selectable )
  {
    setFlags( selectable ? (flags() | Qt::ItemIsSelectable) : (flags() & ~Qt::ItemIsSelectable) );
  }

  std::string full_name;
};

RosTopicTree::RosTopicTree( const std::string& message_type, QWidget* parent )
  : QTreeWidget( parent )
  , message_type_( message_type )
{
  setHeaderHidden( true );
  connect( this, SIGNAL( itemSelectionChanged() ), this, SLOT( onSelectionChanged() ));
  connect( this, SIGNAL( itemActivated( QTreeWidgetItem*, int )),
           this, SLOT( onItemActivated( QTreeWidgetItem*, int )));
  refreshTopics();
}

void RosTopicTree::refreshTopics()
{
  QFont bold_font = font();
  bold_font.setBold( true );

  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);

  // Loop through all published topics
  ros::master::V_TopicInfo::iterator it = topics.begin();
  ros::master::V_TopicInfo::iterator end = topics.end();
  for (; it != end; ++it)
  {
    const ros::master::TopicInfo& topic = *it;

    // If we are filtering by type, skip topics whose type does not match.
    if (!message_type_.empty() && topic.datatype != message_type_)
    {
      continue;
    }

    // Topic not in cache yet.  Find right place to put it in the tree
    std::istringstream iss( topic.name );
    std::string token;

    QTreeWidgetItem* item = invisibleRootItem();

    while( std::getline( iss, token, '/' ))
    {
      if (!token.empty())
      {
        QTreeWidgetItem* child = 0;
        bool exists = false;

        for( int child_index = 0; child_index < item->childCount(); child_index++ )
        {
          child = item->child( child_index );

          if( child->text( 0 ) == QString::fromStdString( token ))
          {
            exists = true;
            break;
          }
        }

        if( exists )
        {
          item = child;
        }
        else
        {
          TopicItem* topic_item = new TopicItem( item );
          topic_item->setSelectable( false );
          item = topic_item;
          item->setText( 0, QString::fromStdString( token ));
        }
      }
    }

    TopicItem* topic_item = dynamic_cast<TopicItem*>( item );
    if( topic_item )
    {
      topic_item->full_name = topic.name;
      topic_item->setSelectable( true );
    }
    item->setText( 0, QString::fromStdString( token ) + " (" + QString::fromStdString( topic.datatype ) + ")" );
    item->setFont( 0, bold_font );
  }
}

void RosTopicTree::onSelectionChanged()
{
  QList<QTreeWidgetItem *> selection = selectedItems();
  if( selection.empty() )
  {
    selected_topic_ = "";
  }
  else
  {
    TopicItem* topic_item = dynamic_cast<TopicItem*>( selection.first() );
    if( topic_item )
    {
      selected_topic_ = topic_item->full_name;
    }
    else
    {
      selected_topic_ = "";
    }
  }
  Q_EMIT selectedTopicChanged( selected_topic_ );
}

void RosTopicTree::onItemActivated( QTreeWidgetItem* item, int column )
{
  TopicItem* topic_item = dynamic_cast<TopicItem*>( item );
  if( topic_item )
  {
    selected_topic_ = topic_item->full_name;
    if( selected_topic_ != "" )
    {
      Q_EMIT topicActivated( selected_topic_ );
    }
  }
}

} // end namespace rviz
