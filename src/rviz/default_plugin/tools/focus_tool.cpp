/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include <OgreRay.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <OgreCamera.h>

#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"
#include "rviz/display_context.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/view_controller.h"

#include "rviz/default_plugin/tools/focus_tool.h"

#include <sstream>

namespace rviz
{

FocusTool::FocusTool()
  : Tool()
{
}

FocusTool::~FocusTool()
{
}

void FocusTool::onInitialize()
{
  std_cursor_ = getDefaultCursor();
  hit_cursor_ = makeIconCursor( "package://rviz/icons/crosshair.svg" );
}

void FocusTool::activate()
{
}

void FocusTool::deactivate()
{
}

int FocusTool::processMouseEvent( ViewportMouseEvent& event )
{
  int flags = 0;

  Ogre::Vector3 pos;
  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  setCursor( success ? hit_cursor_ : std_cursor_ );

  if ( !success )
  {
    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
        (float)event.x / (float)event.viewport->getActualWidth(),
        (float)event.y / (float)event.viewport->getActualHeight() );

    pos = mouse_ray.getPoint(1.0);
    setStatus( "<b>Left-Click:</b> Look in this direction." );
  }
  else
  {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Focus on this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus( s.str().c_str() );
  }

  if( event.leftUp() )
  {
    if ( event.panel->getViewController() )
    {
      event.panel->getViewController()->lookAt( pos );
    }
    flags |= Finished;
  }

  return flags;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::FocusTool, rviz::Tool )
