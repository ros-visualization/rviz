/*
 * display_visibility_manager.cpp
 *
 *  Created on: Aug 27, 2012
 *      Author: gossow
 */

#include "display_visibility_manager.h"

#include "rviz/properties/bool_property.h"
#include "rviz/display_context.h"
#include "rviz/bit_allocator.h"
#include "rviz/display.h"
#include "rviz/display_group.h"

namespace rviz
{

DisplayVisibilityManager::DisplayVisibilityManager( Display* parent, DisplayContext* context, Property* root_property )
: context_(context)
, root_property_(root_property)
{
  vis_bit_ = context_->visibilityBits()->allocBit();

  rviz::DisplayGroup* displays = context_->getRootDisplayGroup();
  for( int i = 0; i < displays->numDisplays(); i++ )
  {
    rviz::Display* display = displays->getDisplayAt( i );
    if ( display != parent )
    {
      onDisplayAdded( display );
    }
  }

  connect( context_->getRootDisplayGroup(), SIGNAL( displayAdded( rviz::Display* ) ), this, SLOT( onDisplayAdded( rviz::Display* ) ));
  connect( context_->getRootDisplayGroup(), SIGNAL( displayRemoved( rviz::Display* ) ), this, SLOT( onDisplayRemoved( rviz::Display* ) ));
}

void DisplayVisibilityManager::update()
{
  std::map<rviz::Display*, DisplayVisibilityContext*>::iterator it = vis_contexts_.begin();
  for( ; it != vis_contexts_.end(); it++ )
  {
    it->second->update();
  }
}

void DisplayVisibilityManager::sortDisplayList()
{
  rviz::DisplayGroup* displays = context_->getRootDisplayGroup();
  for( int i = 0; i < displays->numDisplays(); i++ )
  {
    rviz::Display* display = displays->getDisplayAt( i );
    std::map<rviz::Display*, DisplayVisibilityContext*>::iterator it = vis_contexts_.find( display );
    if ( it != vis_contexts_.end() )
    {
      BoolProperty* prop = it->second->getEnabledProp();
      root_property_->takeChild( prop );
      root_property_->addChild( prop );
    }
  }
}

void DisplayVisibilityManager::onDisplayAdded( Display* display )
{
  DisplayVisibilityContext* ctx = new DisplayVisibilityContext( vis_bit_, display, root_property_ );
  vis_contexts_[ display ] = ctx;
  sortDisplayList();
}

void DisplayVisibilityManager::onDisplayRemoved( Display* display )
{
  std::map<rviz::Display*, DisplayVisibilityContext*>::iterator it = vis_contexts_.find( display );
  if ( it != vis_contexts_.end() )
  {
    delete vis_contexts_[ display ];
    vis_contexts_.erase( display );
  }
}


DisplayVisibilityManager::~DisplayVisibilityManager()
{
  context_->visibilityBits()->freeBits(vis_bit_);

  std::map<rviz::Display*, DisplayVisibilityContext*>::iterator it = vis_contexts_.begin();
  for( ; it != vis_contexts_.end(); it++ )
  {
    delete it->second;
  }
}

DisplayVisibilityContext::DisplayVisibilityContext( uint32_t vis_bit, Display* display, Property* root_property )
: vis_bit_(vis_bit)
, display_(display)
{
  enabled_prop_ = new BoolProperty( display->getName(), true, "Show or hide this Display.",
      root_property, SLOT( visibilityChanged() ), this );
}

DisplayVisibilityContext::~DisplayVisibilityContext()
{
  delete enabled_prop_;
}

void DisplayVisibilityContext::update()
{
  if ( display_name_ != display_->getName() )
  {
    display_name_ = display_->getName();
    enabled_prop_->setName( display_name_ );
  }
  bool enabled = enabled_prop_->getBool();
  if ( enabled )
  {
    display_->setVisibilityBits( vis_bit_ );
  }
  else
  {
    display_->unsetVisibilityBits( vis_bit_ );
  }
}

void DisplayVisibilityContext::visibilityChanged()
{
  update();
}

}
