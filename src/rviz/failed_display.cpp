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

#include <QColor>

#include "rviz/properties/status_property.h"
#include "rviz/display_context.h"
#include "rviz/load_resource.h"

#include "failed_display.h"

namespace rviz
{

FailedDisplay::FailedDisplay( const QString& desired_class_id, const QString& error_message )
  : error_message_( error_message )
{
  setClassId( desired_class_id );
  setIcon( loadPixmap( "package://rviz/icons/failed_display.png" ) );
}

QVariant FailedDisplay::getViewData( int column, int role ) const
{
  if( column == 0 )
  {
    switch( role )
    {
    case Qt::BackgroundRole: return QColor( Qt::white );
    case Qt::ForegroundRole: return StatusProperty::statusColor( StatusProperty::Error );
    default: break;
    }
  }
  return Display::getViewData( column, role );
}

QString FailedDisplay::getDescription() const
{
  return "The class required for this display, '" + getClassId() + "', could not be loaded.<br><b>Error:</b><br>" + error_message_;
}

void FailedDisplay::load( const Config& config )
{
  saved_config_ = config;
  Display::load( config );
}

void FailedDisplay::save( Config config )
{
  if( saved_config_.isValid() )
  {
    config.copy( saved_config_ );
  }
}

} // end namespace rviz
