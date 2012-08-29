/*
 * display_visibility_manager.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: gossow
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
  if ( getBool() )
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
