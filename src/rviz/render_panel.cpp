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

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/fps_camera.h"
#include "ogre_tools/orbit_camera.h"
#include "ogre_tools/ortho_camera.h"

#include <boost/bind.hpp>

#include <OgreRoot.h>
#include <OgreViewport.h>

namespace rviz
{

namespace Views
{
enum View
{
  Orbit,
  FPS,
  TopDownOrtho,

  Count
};
}
typedef Views::View View;

const char* g_view_names[Views::Count] =
{
  "Orbit",
  "FPS",
  "TopDownOrtho"
};

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_RENDER, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_RENDER);

RenderPanel::RenderPanel( wxWindow* parent )
: RenderPanelGenerated( parent )
, current_camera_( NULL )
, mouse_x_( 0 )
, mouse_y_( 0 )
, manager_(NULL)
{
  render_panel_ = new ogre_tools::wxOgreRenderWindow( Ogre::Root::getSingletonPtr(), this );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  views_->Append( wxT( "Orbit" ) );
  views_->Append( wxT( "FPS" ) );
  views_->Append( wxT( "Top-down Orthographic" ) );
  views_->SetSelection( 0 );

  render_panel_->SetFocus();
  render_panel_->Connect( wxEVT_CHAR, wxKeyEventHandler( RenderPanel::onChar ), NULL, this );

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->setPreRenderCallback( boost::bind( &RenderPanel::lockRender, this ) );
  render_panel_->setPostRenderCallback( boost::bind( &RenderPanel::unlockRender, this ) );

  Connect( EVT_RENDER, wxCommandEventHandler( RenderPanel::onRender ), NULL, this );

  tools_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( RenderPanel::onToolClicked ), NULL, this );
}

RenderPanel::~RenderPanel()
{
  tools_->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( RenderPanel::onToolClicked ), NULL, this );

  delete fps_camera_;
  delete orbit_camera_;
  delete top_down_ortho_;

  render_panel_->Disconnect( wxEVT_CHAR, wxKeyEventHandler( RenderPanel::onChar ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( RenderPanel::onRenderWindowMouseEvents ), NULL, this );

  Disconnect( EVT_RENDER, wxCommandEventHandler( RenderPanel::onRender ), NULL, this );

  render_panel_->Destroy();
}

void RenderPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;

  fps_camera_ = new ogre_tools::FPSCamera( manager_->getSceneManager() );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  fps_camera_->setPosition( 0, 0, 15 );
  fps_camera_->setRelativeNode( manager_->getTargetRelativeNode() );

  orbit_camera_ = new ogre_tools::OrbitCamera( manager_->getSceneManager() );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  orbit_camera_->setPosition( 0, 0, 15 );
  orbit_camera_->setRelativeNode( manager_->getTargetRelativeNode() );

  top_down_ortho_ = new ogre_tools::OrthoCamera( render_panel_, manager_->getSceneManager() );
  top_down_ortho_->setPosition( 0, 30, 0 );
  top_down_ortho_->pitch( -Ogre::Math::HALF_PI );
  top_down_ortho_->setRelativeNode( manager_->getTargetRelativeNode() );

  current_camera_ = orbit_camera_;
  current_camera_type_ = Views::Orbit;

  render_panel_->getViewport()->setCamera( current_camera_->getOgreCamera() );
}

void RenderPanel::addTool( Tool* tool )
{
  char ascii_str[2] = { tool->getShortcutKey(), 0 };
  wxString tooltip = wxString( wxT("Shortcut Key: ")) + wxString::FromAscii( ascii_str );
  tools_->AddRadioTool( tools_->GetToolsCount(), wxString::FromAscii( tool->getName().c_str() ), wxNullBitmap, wxNullBitmap, tooltip );
}

void RenderPanel::setTool( Tool* tool )
{
  int count = tools_->GetToolsCount();
  for ( int i = 0; i < count; ++i )
  {
    if ( manager_->getTool( i ) == tool )
    {
      tools_->ToggleTool( i, true );
      break;
    }
  }
}

void RenderPanel::queueRender()
{
  wxCommandEvent event( EVT_RENDER, GetId() );
  wxPostEvent( this, event );
}

void RenderPanel::setCurrentCamera(int camera_type)
{
  if (camera_type == current_camera_type_)
  {
    return;
  }

  ogre_tools::CameraBase* prev_camera = current_camera_;

  bool set_from_old = false;

  switch ( camera_type )
  {
  case Views::FPS:
    {
      if ( current_camera_ == orbit_camera_ )
      {
        set_from_old = true;
      }

      current_camera_ = fps_camera_;
    }
    break;

  case Views::Orbit:
    {
      if ( current_camera_ == fps_camera_ )
      {
        set_from_old = true;
      }

      current_camera_ = orbit_camera_;
    }
    break;

  case Views::TopDownOrtho:
    {
      current_camera_ = top_down_ortho_;
    }
    break;
  }

  if ( set_from_old )
  {
    current_camera_->setFrom( prev_camera );
  }

  current_camera_type_ = camera_type;
  views_->SetSelection(camera_type);
  render_panel_->setCamera( current_camera_->getOgreCamera() );
}

bool RenderPanel::setCurrentCamera(const std::string& camera_type)
{
  for (int i = 0; i < Views::Count; ++i)
  {
    if (g_view_names[i] == camera_type)
    {
      setCurrentCamera(i);
      return true;
    }
  }

  return false;
}

const char* RenderPanel::getCurrentCameraType()
{
  return g_view_names[current_camera_type_];
}

void RenderPanel::onRender( wxCommandEvent& event )
{
  render_panel_->Refresh();
}

void RenderPanel::onToolClicked( wxCommandEvent& event )
{
  Tool* tool = manager_->getTool( event.GetId() );

  manager_->setCurrentTool( tool );
}

void RenderPanel::onResetTime( wxCommandEvent& event )
{
  manager_->resetTime();
}

void RenderPanel::onViewSelected( wxCommandEvent& event )
{
  setCurrentCamera(views_->GetSelection());
}

void RenderPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event.GetX();
  mouse_y_ = event.GetY();

  render_panel_->SetFocus();

  int flags = manager_->getCurrentTool()->processMouseEvent( event, last_x, last_y );

  if ( flags & Tool::Render )
  {
    queueRender();
  }

  if ( flags & Tool::Finished )
  {
    manager_->setCurrentTool( manager_->getDefaultTool() );
  }
}

void RenderPanel::onChar( wxKeyEvent& event )
{
  manager_->handleChar( event );
}

} // namespace rviz
