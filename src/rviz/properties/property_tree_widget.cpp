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

#include "rviz/properties/property.h"
#include "rviz/properties/property_tree_delegate.h"
#include "rviz/properties/splitter_handle.h"

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
  }
  model_ = model;
  QTreeView::setModel( model_ );
  connect( model_, SIGNAL( propertyHiddenChanged( const Property* )),
           this, SLOT( propertyHiddenChanged( const Property* )));
}

void PropertyTreeWidget::propertyHiddenChanged( const Property* property )
{
  setRowHidden( property->rowNumberInParent(), model_->parentIndex( property ), property->getHidden() );
}

} // end namespace rviz
