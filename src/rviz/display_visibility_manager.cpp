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
      DisplayVisibilityContext* ctx = new DisplayVisibilityContext( vis_bit_, display, root_property_ );
      contexts_[ display ] = ctx;
    }
  }
}

DisplayVisibilityManager::~DisplayVisibilityManager()
{
  context_->visibilityBits()->freeBits(vis_bit_);
}

DisplayVisibilityContext::DisplayVisibilityContext( uint32_t vis_bit, Display* display, Property* root_property )
: vis_bit_(vis_bit)
, display_(display)
{
  enabled_prop_ = new BoolProperty( display->getName(), true, "Show or hide this Display.",
      root_property, SLOT( visibilityChanged() ), this );
  display_->setVisibilityBits( vis_bit_ );
}

DisplayVisibilityContext::~DisplayVisibilityContext()
{
  display_->setVisibilityBits( vis_bit_ );
  delete enabled_prop_;
}

void DisplayVisibilityContext::visibilityChanged()
{
  bool enabled = enabled_prop_->getBool();
  if ( enabled )
  {
    display_->setVisibilityBits( vis_bit_ );
    std::cout << display_->getNameStd() << " on." << std::endl;
  }
  else
  {
    display_->unsetVisibilityBits( vis_bit_ );
    std::cout << display_->getNameStd() << " off." << std::endl;
  }
}

}
