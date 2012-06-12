/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#include <QHash>
#include <QSet>

#include <yaml-cpp/emitter.h>
#include <yaml-cpp/node.h>

#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_delegate.h"
#include "rviz/properties/splitter_handle.h"
#include "rviz/properties/status_list.h"
#include "rviz/properties/yaml_helpers.h"

#include "rviz/properties/property_tree_widget.h"

namespace rviz
{

PropertyTreeWidget::PropertyTreeWidget( QWidget* parent )
  : QTreeView( parent )
  , model_( NULL )
  , splitter_handle_( new SplitterHandle( this ))
{
  setItemDelegateForColumn( 1, new PropertyTreeDelegate( this ));
  setDropIndicatorShown( true );
  setUniformRowHeights( true );
  setHeaderHidden( true );
  setDragEnabled( true );
  setAcceptDrops( true );
  setAnimated( true );
  setSelectionMode( QAbstractItemView::ExtendedSelection );
  setEditTriggers( QAbstractItemView::AllEditTriggers );

  QTimer* timer = new QTimer( this );
  connect( timer, SIGNAL( timeout() ), this, SLOT( update() ));
  timer->start( 100 );
}

void PropertyTreeWidget::currentChanged( const QModelIndex& new_current_index, const QModelIndex& previous_current_index )
{
  QTreeView::currentChanged( new_current_index, previous_current_index );
  Q_EMIT currentPropertyChanged( static_cast<const Property*>( new_current_index.internalPointer() ));
}

void PropertyTreeWidget::selectionChanged( const QItemSelection& selected, const QItemSelection& deselected )
{
  QTreeView::selectionChanged( selected, deselected );
  Q_EMIT selectionHasChanged();
}

void PropertyTreeWidget::setModel( PropertyTreeModel* model )
{
  if( model_ )
  {
    disconnect( model_, SIGNAL( propertyHiddenChanged( const Property* )),
                this, SLOT( propertyHiddenChanged( const Property* )));
    disconnect( model_, SIGNAL( expand( const QModelIndex& )),
                this, SLOT( expand( const QModelIndex& )));
  }
  model_ = model;
  QTreeView::setModel( model_ );
  connect( model_, SIGNAL( propertyHiddenChanged( const Property* )),
           this, SLOT( propertyHiddenChanged( const Property* )), Qt::QueuedConnection );
  connect( model_, SIGNAL( expand( const QModelIndex& )),
           this, SLOT( expand( const QModelIndex& )));
}

void PropertyTreeWidget::propertyHiddenChanged( const Property* property )
{
  setRowHidden( property->rowNumberInParent(), model_->parentIndex( property ), property->getHidden() );
}

void PropertyTreeWidget::save( YAML::Emitter& emitter )
{
  emitter << YAML::BeginMap;

  emitter << YAML::Key << "Expanded";
  emitter << YAML::Value;
  {
    emitter << YAML::BeginSeq;
    saveExpandedEntries( emitter, QModelIndex(), "" );
    emitter << YAML::EndSeq;
  }

  emitter << YAML::Key << "Splitter Ratio";
  emitter << YAML::Value << splitter_handle_->getRatio();

  emitter << YAML::EndMap;
}

void PropertyTreeWidget::saveExpandedEntries( YAML::Emitter& emitter, const QModelIndex& parent_index, const QString& prefix )
{
  int num_children = model_->rowCount( parent_index );
  if( num_children > 0 )
  {
    QHash<QString, int> name_counts;
    for( int i = 0; i < num_children; i++ )
    {
      QModelIndex child_index = model_->index( i, 0, parent_index );
      Property* child = model_->getProp( child_index );
      QString child_name = child->getName();
      if( qobject_cast<StatusList*>( child ))
      {
        // StatusList objects change their name dynamically, so
        // normalize to a standard string.
        child_name = "Status";
      }
      int name_occurrence = ++( name_counts[ child_name ]);
      QString full_name = prefix + "/" + child_name + QString::number( name_occurrence );
      if( isExpanded( child_index ))
      {
        emitter << full_name;
      }
      saveExpandedEntries( emitter, child_index, full_name );
    }
  }
}

void PropertyTreeWidget::load( const YAML::Node& yaml_node )
{
  if( const YAML::Node *expanded_node = yaml_node.FindValue( "Expanded" ))
  {
    QSet<QString> expanded_full_names;
    for( unsigned i = 0; i < expanded_node->size(); i++ )
    {
      QString full_name;
      (*expanded_node)[ i ] >> full_name;
      expanded_full_names.insert( full_name );
    }

    expandEntries( expanded_full_names, QModelIndex(), "" );
  }

  if( const YAML::Node *ratio_node = yaml_node.FindValue( "Splitter Ratio" ))
  {
    float ratio;
    (*ratio_node) >> ratio;
    splitter_handle_->setRatio( ratio );
  }
}

void PropertyTreeWidget::expandEntries( const QSet<QString>& expanded_full_names,
                                        const QModelIndex& parent_index,
                                        const QString& prefix )
{
  int num_children = model_->rowCount( parent_index );
  if( num_children > 0 )
  {
    QHash<QString, int> name_counts;
    for( int i = 0; i < num_children; i++ )
    {
      QModelIndex child_index = model_->index( i, 0, parent_index );
      Property* child = model_->getProp( child_index );
      QString child_name = child->getName();
      if( qobject_cast<StatusList*>( child ))
      {
        child_name = "Status";
      }
      int name_occurrence = ++( name_counts[ child_name ]);
      QString full_name = prefix + "/" + child_name + QString::number( name_occurrence );
      if( expanded_full_names.contains( full_name ))
      {
        setExpanded( child_index, true );
      }
      expandEntries( expanded_full_names, child_index, full_name );
    }
  }
}

} // end namespace rviz
