/*
 * display_visibility_manager.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: gossow
 */

#include "display_group_visibility_property.h"

#include "rviz/properties/bool_property.h"
#include "rviz/display_context.h"
#include "rviz/bit_allocator.h"
#include "rviz/display.h"
#include "rviz/display_group.h"

namespace rviz
{


DisplayGroupVisibilityProperty::DisplayGroupVisibilityProperty(
    uint32_t vis_bit,
    DisplayGroup* display_group,
    Display* parent_display,
    const QString& name,
    bool default_value,
    const QString& description,
    Property* parent,
    const char *changed_slot,
    QObject* receiver )
: DisplayVisibilityProperty( vis_bit, display_group, name, default_value, description, parent, changed_slot, receiver )
, display_group_(display_group)
, parent_display_(parent_display)
{
  connect( display_group, SIGNAL( displayAdded( rviz::Display* ) ), this, SLOT( onDisplayAdded( rviz::Display* ) ));
  connect( display_group, SIGNAL( displayRemoved( rviz::Display* ) ), this, SLOT( onDisplayRemoved( rviz::Display* ) ));

  for( int i = 0; i < display_group->numDisplays(); i++ )
  {
    rviz::Display* display = display_group->getDisplayAt( i );
    if ( display != parent_display )
    {
      onDisplayAdded( display );
    }
  }
}

void DisplayGroupVisibilityProperty::update()
{
  DisplayVisibilityProperty::update();
  std::map<rviz::Display*, DisplayVisibilityProperty*>::iterator it = disp_vis_props_.begin();
  for( ; it != disp_vis_props_.end(); it++ )
  {
    it->second->update();
  }
}

void DisplayGroupVisibilityProperty::sortDisplayList()
{
  // remove and re-add everything in our property list
  // in the same order as it appears in the display group
  for( int i = 0; i < display_group_->numDisplays(); i++ )
  {
    rviz::Display* display = display_group_->getDisplayAt( i );
    std::map<rviz::Display*, DisplayVisibilityProperty*>::iterator it = disp_vis_props_.find( display );
    if ( it != disp_vis_props_.end() )
    {
      takeChild( it->second );
      addChild( it->second );
    }
  }
}

void DisplayGroupVisibilityProperty::onDisplayAdded( Display* display )
{
  DisplayGroup* display_group = qobject_cast<DisplayGroup*>( display );
  DisplayVisibilityProperty* vis_prop;
  if( display_group )
  {
    vis_prop = new DisplayGroupVisibilityProperty( vis_bit_, display_group, parent_display_, "", true, "Uncheck to hide everything in this Display Group", this );
  }
  else
  {
    vis_prop = new DisplayVisibilityProperty( vis_bit_, display, "", true, "Show or hide this Display", this );
  }
  disp_vis_props_[ display ] = vis_prop;
  sortDisplayList();
}

void DisplayGroupVisibilityProperty::onDisplayRemoved( Display* display )
{
  std::map<rviz::Display*, DisplayVisibilityProperty*>::iterator it = disp_vis_props_.find( display );
  if ( it != disp_vis_props_.end() )
  {
    Property* child = takeChild( it->second );
    child->setParent( NULL );
    delete child;
    disp_vis_props_.erase( display );
  }
}


DisplayGroupVisibilityProperty::~DisplayGroupVisibilityProperty()
{
}

}
