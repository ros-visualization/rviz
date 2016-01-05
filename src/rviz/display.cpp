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

#include <stdio.h>

#include <QApplication>
#include <QColor>
#include <QDockWidget>
#include <QFont>
#include <QMetaObject>
#include <QWidget>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/ogre_helpers/apply_visibility_bits.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/status_list.h"
#include "rviz/window_manager_interface.h"
#include "rviz/panel_dock_widget.h"

#include "display.h"

#include <boost/filesystem.hpp>

namespace rviz
{

Display::Display()
  : context_( 0 )
  , scene_node_( NULL )
  , status_( 0 )
  , initialized_( false )
  , visibility_bits_( 0xFFFFFFFF )
  , associated_widget_( NULL )
  , associated_widget_panel_( NULL )
{
  // Needed for timeSignal (see header) to work across threads
  qRegisterMetaType<ros::Time>();

  // Make the display-enable checkbox show up, and make it unchecked by default.
  setValue( false );

  connect( this, SIGNAL( changed() ), this, SLOT( onEnableChanged() ));

  setDisableChildrenIfFalse(true);
}

Display::~Display()
{
  if( scene_node_ )
  {
    scene_manager_->destroySceneNode( scene_node_ );
  }
}

void Display::initialize( DisplayContext* context )
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  update_nh_.setCallbackQueue( context_->getUpdateQueue() );
  threaded_nh_.setCallbackQueue( context_->getThreadedQueue() );
  fixed_frame_ = context_->getFixedFrame();

  onInitialize();

  initialized_ = true;
}

void Display::queueRender()
{
  if( context_ )
  {
    context_->queueRender();
  }
}

QVariant Display::getViewData( int column, int role ) const
{
  switch( role )
  {
  case Qt::BackgroundRole:
  {
    /*
    QLinearGradient q( 0,0, 0,5 );
    q.setColorAt( 0.0, QColor(230,230,230) );
    q.setColorAt( 1.0, Qt::white );
    return QBrush( q );
    */
    return QColor( Qt::white );
  }
  case Qt::ForegroundRole:
  {
    // if we're item-enabled (not greyed out) and in warn/error state, set appropriate color
    if ( getViewFlags( column ) & Qt::ItemIsEnabled )
    {
      if ( isEnabled() )
      {
        if ( status_ && status_->getLevel() != StatusProperty::Ok )
        {
          return StatusProperty::statusColor( status_->getLevel() );
        }
        else
        {
          // blue means that the enabled checkmark is set
          return QColor( 40, 120, 197 );
        }
      }
      else
      {
        return QColor( Qt::black );
      }
    }
    break;
  }
  case Qt::FontRole:
  {
    QFont font;
    if ( isEnabled() )
    {
      font.setBold( true );
    }
    return font;
  }
  case Qt::DecorationRole:
  {
    if( column == 0 )
    {
      if ( isEnabled() )
      {
        StatusProperty::Level level = status_ ? status_->getLevel() : StatusProperty::Ok;
        switch( level )
        {
        case StatusProperty::Ok:
          return getIcon();
        case StatusProperty::Warn:
        case StatusProperty::Error:
          return status_->statusIcon( status_->getLevel() );
        }
      }
      else
      {
        return getIcon();
      }
    }
    break;
  }
  }
  return BoolProperty::getViewData( column, role );
}

Qt::ItemFlags Display::getViewFlags( int column ) const
{
  return BoolProperty::getViewFlags( column ) | Qt::ItemIsDragEnabled;
}

void Display::setStatus( StatusProperty::Level level, const QString& name, const QString& text )
{
  QMetaObject::invokeMethod( this, "setStatusInternal", Qt::QueuedConnection,
                             Q_ARG( int, level ),
                             Q_ARG( QString, name ),
                             Q_ARG( QString, text ));
}

void Display::setStatusInternal( int level, const QString& name, const QString& text )
{
  if( !status_ )
  {
    status_ = new StatusList( "Status" );
    addChild( status_, 0 );
  }
  StatusProperty::Level old_level = status_->getLevel();
  
  status_->setStatus( (StatusProperty::Level) level, name, text );
  if( model_ && old_level != status_->getLevel() )
  {
    model_->emitDataChanged( this );
  }
}

void Display::deleteStatus( const QString& name )
{
  QMetaObject::invokeMethod( this, "deleteStatusInternal", Qt::QueuedConnection,
                             Q_ARG( QString, name ));
}

void Display::deleteStatusInternal( const QString& name )
{
  if( status_ )
  {
    status_->deleteStatus( name );
  }
}

void Display::clearStatuses()
{
  QMetaObject::invokeMethod( this, "clearStatusesInternal", Qt::QueuedConnection );
}

void Display::clearStatusesInternal()
{
  if( status_ )
  {
    StatusProperty::Level old_level = status_->getLevel();
    status_->clear();
    if( model_ && old_level != StatusProperty::Ok )
    {
      model_->emitDataChanged( this );
    }
  }
}

void Display::load( const Config& config )
{
  // Base class loads sub-properties.
  BoolProperty::load( config );

  QString name;
  if( config.mapGetString( "Name", &name ))
  {
    setObjectName( name );
  }

  bool enabled;
  if( config.mapGetBool( "Enabled", &enabled ))
  {
    setEnabled( enabled );
  }
}

void Display::save( Config config ) const
{
  // Base class saves sub-properties.
  BoolProperty::save( config );

  config.mapSetValue( "Class", getClassId() );
  config.mapSetValue( "Name", getName() );
  config.mapSetValue( "Enabled", getBool() );
}

void Display::setEnabled( bool enabled )
{
  if ( enabled == isEnabled() ) return;
  setValue( enabled );
}

void Display::disable()
{
  setEnabled( false );
}

bool Display::isEnabled() const
{
  return getBool() && (getViewFlags( 0 ) & Qt::ItemIsEnabled);
}

void Display::setFixedFrame( const QString& fixed_frame )
{
  fixed_frame_ = fixed_frame;
  if( initialized_ )
  {
    fixedFrameChanged();
  }
}

void Display::emitTimeSignal( ros::Time time )
{
  Q_EMIT( timeSignal( this, time ) );
}

void Display::reset()
{
  clearStatuses();
}

void Display::onEnableChanged()
{
  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  queueRender();
  if( isEnabled() )
  {
    scene_node_->setVisible( true );

    if( associated_widget_panel_ )
    {
      associated_widget_panel_->show();
    }
    else if( associated_widget_ )
    {
      associated_widget_->show();
    }

    onEnable();
  }
  else
  {
    onDisable();

    if( associated_widget_panel_ )
    {
      if( associated_widget_panel_->isVisible() )
      {
        associated_widget_panel_->hide();
      }
    }
    else if( associated_widget_ && associated_widget_->isVisible() )
    {
      associated_widget_->hide();
    }

    scene_node_->setVisible( false );
  }
  QApplication::restoreOverrideCursor();
}

void Display::setVisibilityBits( uint32_t bits )
{
  visibility_bits_ |= bits;
  applyVisibilityBits( visibility_bits_, scene_node_ );
}

void Display::unsetVisibilityBits( uint32_t bits )
{
  visibility_bits_ &= ~bits;
  applyVisibilityBits( visibility_bits_, scene_node_ );
}

void Display::setAssociatedWidget( QWidget* widget )
{
  if( associated_widget_panel_ )
  {
    disconnect( associated_widget_panel_, SIGNAL( visibilityChanged( bool ) ), this, SLOT( associatedPanelVisibilityChange( bool ) ));
    disconnect( associated_widget_panel_, SIGNAL( closed( ) ), this, SLOT( disable( )));
  }

  associated_widget_ = widget;
  if( associated_widget_ )
  {
    WindowManagerInterface* wm = context_->getWindowManager();
    if( wm )
    {
      associated_widget_panel_ = wm->addPane( getName(), associated_widget_ );
      connect( associated_widget_panel_, SIGNAL( visibilityChanged( bool ) ), this, SLOT( associatedPanelVisibilityChange( bool ) ));
      connect( associated_widget_panel_, SIGNAL( closed( ) ), this, SLOT( disable( )));
      associated_widget_panel_->setIcon( getIcon() );
    }
    else
    {
      associated_widget_panel_ = NULL;
      associated_widget_->setWindowTitle( getName() );
    }
  }
  else
  {
    associated_widget_panel_ = NULL;
  }
}

void Display::associatedPanelVisibilityChange( bool visible )
{
  // if something external makes the panel visible, make sure we're enabled
  if ( visible )
  {
    setEnabled( true );
  }
  else
  {
    setEnabled( false );
  }
}

void Display::setIcon( const QIcon& icon )
{
  icon_=icon;
  if ( associated_widget_panel_ )
  {
    associated_widget_panel_->setIcon( getIcon() );
  }
}

void Display::setName( const QString& name )
{
  BoolProperty::setName( name );

  if( associated_widget_panel_ )
  {
    associated_widget_panel_->setWindowTitle( name );
    associated_widget_panel_->setObjectName( name ); // QMainWindow::saveState() needs objectName to be set.
  }
  else if( associated_widget_ )
  {
    associated_widget_->setWindowTitle( name );
  }
}

} // end namespace rviz
