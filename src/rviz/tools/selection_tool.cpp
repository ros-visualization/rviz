/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "selection_tool.h"
#include "visualization_manager.h"
#include "render_panel.h"
#include "display.h"

#include "ogre_tools/camera_base.h"
#include "ogre_tools/wx_ogre_render_window.h"

#include <wx/wx.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneQuery.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMovableObject.h>

namespace rviz
{

SelectionTool::SelectionTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: Tool( name, shortcut_key, manager )
, selection_( NULL )
{
  ray_scene_query_ = scene_manager_->createRayQuery( Ogre::Ray() );
}

SelectionTool::~SelectionTool()
{
  scene_manager_->destroyQuery( ray_scene_query_ );
}

void SelectionTool::deactivate()
{
  if ( selection_ )
  {
    selection_->getParentSceneNode()->showBoundingBox( false );
    selection_ = NULL;
  }
}

Ogre::MovableObject* SelectionTool::pick( int mouse_x, int mouse_y )
{
  RenderPanel* render_panel = manager_->getRenderPanel();

  int width, height;
  render_panel->getRenderPanel()->GetSize( &width, &height );

  Ogre::Ray mouse_ray = render_panel->getCurrentCamera()->getOgreCamera()->getCameraToViewportRay( (float)mouse_x / (float)width, (float)mouse_y / (float)height );
  ray_scene_query_->setRay( mouse_ray );

  Ogre::RaySceneQueryResult& result = ray_scene_query_->execute();

  Ogre::MovableObject* picked = NULL;
  float closest_distance = 9999999.0f;

  // find the closest object that is also selectable
  Ogre::RaySceneQueryResult::iterator it = result.begin();
  Ogre::RaySceneQueryResult::iterator end = result.end();
  for ( ; it != end; ++it )
  {
    Ogre::RaySceneQueryResultEntry& entry = *it;

    const Ogre::Any& user_any = entry.movable->getUserAny();
    if ( user_any.isEmpty() )
    {
      continue;
    }

    // ugh -- can't just any_cast to Display because it's abstract
    /// @todo This is dangerous, should find a better way
    const Display* display = reinterpret_cast<const Display*>( Ogre::any_cast<void*>( user_any ) );

    if ( display && display->isObjectPickable( entry.movable ) )
    {
      if ( entry.distance < closest_distance )
      {
        closest_distance = entry.distance;
        picked = entry.movable;
      }
    }
  }

  return picked;
}

int SelectionTool::processMouseEvent( wxMouseEvent& event, int last_x, int last_y )
{
  int flags = 0;

  if ( selection_ )
  {
    selection_->getParentSceneNode()->showBoundingBox( false );
  }

  selection_ = pick( event.GetX(), event.GetY() );

  if ( selection_ )
  {
    selection_->getParentSceneNode()->showBoundingBox( true );
  }

  flags |= Render;

  if ( event.LeftDown() )
  {
    if ( selection_ )
    {
      const Ogre::AxisAlignedBox& aabb = selection_->getParentSceneNode()->_getWorldAABB();
      manager_->getRenderPanel()->getCurrentCamera()->lookAt( aabb.getCenter() );

      selection_->getParentSceneNode()->showBoundingBox( false );
    }

    flags |= Finished;
  }

  return flags;
}

}

