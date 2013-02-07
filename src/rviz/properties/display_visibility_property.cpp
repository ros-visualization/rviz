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

#include "display_visibility_property.h"

#include "rviz/properties/bool_property.h"
#include "rviz/display_context.h"
#include "rviz/bit_allocator.h"
#include "rviz/display.h"
#include "rviz/display_group.h"

namespace rviz
{

DisplayVisibilityProperty::DisplayVisibilityProperty( uint32_t vis_bit,
    Display* display,
    const QString& name,
    bool default_value,
    const QString& description,
    Property* parent,
    const char *changed_slot,
    QObject* receiver )
: BoolProperty( name, default_value, description, parent, changed_slot, receiver )
, vis_bit_(vis_bit)
, display_(display)
{
  custom_name_ = (name.size() != 0);
  update();
}

DisplayVisibilityProperty::~DisplayVisibilityProperty()
{
}

void DisplayVisibilityProperty::update()
{
  // update name, unless we had a custom name given in the constructor
  if ( !custom_name_ && getName() != display_->getName() )
  {
    setName( display_->getName() );
  }
  if ( getBool() &&
       (getViewFlags( 0 ) & Qt::ItemIsEnabled ) )
  {
    display_->setVisibilityBits( vis_bit_ );
  }
  else
  {
    display_->unsetVisibilityBits( vis_bit_ );
  }
}

bool DisplayVisibilityProperty::setValue( const QVariant& new_value )
{
  if ( Property::setValue( new_value ) )
  {
    update();
    return true;
  }
  return false;
}

bool DisplayVisibilityProperty::getBool() const
{
  if ( !display_->isEnabled() )
  {
    return false;
  }
  return BoolProperty::getBool();
}

Qt::ItemFlags DisplayVisibilityProperty::getViewFlags( int column ) const
{
  if ( !display_->isEnabled() )
  {
    return Qt::ItemIsSelectable;
  }
  return BoolProperty::getViewFlags( column );
}


}
