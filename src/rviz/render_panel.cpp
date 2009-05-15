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

#include "render_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "tools/tool.h"
#include "viewport_mouse_event.h"

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>

namespace rviz
{

RenderPanel::RenderPanel( wxWindow* parent, bool create_render_window )
: wxOgreRenderWindow( Ogre::Root::getSingletonPtr(), parent, wxID_ANY, wxDefaultPosition, wxSize(800, 600), wxSUNKEN_BORDER, wxDefaultValidator, create_render_window )
, mouse_x_( 0 )
, mouse_y_( 0 )
, manager_(NULL)
{
  SetFocus();
  Connect( wxEVT_CHAR, wxKeyEventHandler( RenderPanel::onChar ), NULL, this );

  Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_MOTION, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_LEFT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
}

RenderPanel::~RenderPanel()
{
  Disconnect( wxEVT_CHAR, wxKeyEventHandler( RenderPanel::onChar ), NULL, this );
  Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_MOTION, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
}

void RenderPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;
}

void RenderPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event.GetX();
  mouse_y_ = event.GetY();

  if (manager_)
  {
    SetFocus();

    ViewportMouseEvent vme( getViewport(), event, last_x, last_y );
    manager_->handleMouseEvent(vme);
  }
}

void RenderPanel::onChar( wxKeyEvent& event )
{
  if (manager_)
  {
    manager_->handleChar( event );
  }
}

} // namespace rviz
