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

#include "rviz/properties/property_tree_model.h"

#include "rviz/properties/status_property.h"

namespace rviz
{

QColor StatusProperty::status_colors_[3] = { QColor(), QColor( 195, 180, 0 ), QColor( 178, 23, 46 ) };
QString StatusProperty::status_words_[3] = { "Ok", "Warn", "Error" };

StatusProperty::StatusProperty( const QString& name, const QString& text, Level level, Property* parent )
  : Property( name, text, text, parent )
  , level_( level )
{
}

QVariant StatusProperty::getViewData( int column, int role ) const
{
  if( column == 0 && role == Qt::ForegroundRole )
  {
    return statusColor( level_ );
  }
  return Property::getViewData( column, role );
}

Qt::ItemFlags StatusProperty::getViewFlags( int column ) const
{
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

// static function
QColor StatusProperty::statusColor( Level level )
{
  return status_colors_[ (int) level ];
}

/** @brief Return the word appropriate for the given status level:
 * "Ok", "Warn", or "Error". */
// static function
QString StatusProperty::statusWord( Level level )
{
  return status_words_[ (int) level ];
}


void StatusProperty::setLevel( Level level )
{
  if( level_ != level )
  {
    level_ = level;
    model_->emitDataChanged( this );
  }
}

} // end namespace rviz
