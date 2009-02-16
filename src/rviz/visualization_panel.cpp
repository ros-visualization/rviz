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

#include "visualization_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "new_display_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "tools/tool.h"

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/fps_camera.h"
#include "ogre_tools/orbit_camera.h"
#include "ogre_tools/ortho_camera.h"

#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>
#include <wx/artprov.h>

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

DEFINE_EVENT_TYPE(EVT_RENDER)

VisualizationPanel::VisualizationPanel( wxWindow* parent )
: VisualizationPanelGenerated( parent )
, current_camera_( NULL )
, mouse_x_( 0 )
, mouse_y_( 0 )
, selected_display_( NULL )
{
  render_panel_ = new ogre_tools::wxOgreRenderWindow( Ogre::Root::getSingletonPtr(), VisualizationPanelGenerated::render_panel_ );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  views_->Append( wxT( "Orbit" ) );
  views_->Append( wxT( "FPS" ) );
  views_->Append( wxT( "Top-down Orthographic" ) );
  views_->SetSelection( 0 );

  render_panel_->SetFocus();
  render_panel_->Connect( wxEVT_CHAR, wxKeyEventHandler( VisualizationPanel::onChar ), NULL, this );

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->setPreRenderCallback( boost::bind( &VisualizationPanel::lockRender, this ) );
  render_panel_->setPostRenderCallback( boost::bind( &VisualizationPanel::unlockRender, this ) );

  Connect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_ = new wxPropertyGrid( properties_panel_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  properties_panel_sizer_->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( VisualizationPanel::onPropertySelected ), NULL, this );

  property_grid_->SetCaptionBackgroundColour( wxColour( 2, 0, 174 ) );
  property_grid_->SetCaptionForegroundColour( *wxLIGHT_GREY );

  manager_ = new VisualizationManager( this );
  manager_->initialize();
  manager_->getDisplayStateSignal().connect( boost::bind( &VisualizationPanel::onDisplayStateChanged, this, _1 ) );

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

  tools_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onToolClicked ), NULL, this );

  up_button_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_UP, wxART_OTHER, wxSize(16,16) ) );
  down_button_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_DOWN, wxART_OTHER, wxSize(16,16) ) );
}

VisualizationPanel::~VisualizationPanel()
{
  tools_->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onToolClicked ), NULL, this );

  delete fps_camera_;
  delete orbit_camera_;
  delete top_down_ortho_;

  delete manager_;

  render_panel_->Disconnect( wxEVT_CHAR, wxKeyEventHandler( VisualizationPanel::onChar ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  Disconnect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanged ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( VisualizationPanel::onPropertySelected ), NULL, this );
  property_grid_->Destroy();

  render_panel_->Destroy();
}

void VisualizationPanel::addTool( Tool* tool )
{
  char ascii_str[2] = { tool->getShortcutKey(), 0 };
  wxString tooltip = wxString( wxT("Shortcut Key: ")) + wxString::FromAscii( ascii_str );
  tools_->AddRadioTool( tools_->GetToolsCount(), wxString::FromAscii( tool->getName().c_str() ), wxNullBitmap, wxNullBitmap, tooltip );
}

void VisualizationPanel::setTool( Tool* tool )
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

void VisualizationPanel::queueRender()
{
  wxCommandEvent event( EVT_RENDER, GetId() );
  wxPostEvent( this, event );
}

void VisualizationPanel::setCurrentCamera(int camera_type)
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

bool VisualizationPanel::setCurrentCamera(const std::string& camera_type)
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

const char* VisualizationPanel::getCurrentCameraType()
{
  return g_view_names[current_camera_type_];
}

void VisualizationPanel::onRender( wxCommandEvent& event )
{
  render_panel_->Refresh();
}

void VisualizationPanel::onToolClicked( wxCommandEvent& event )
{
  Tool* tool = manager_->getTool( event.GetId() );

  manager_->setCurrentTool( tool );
}

void VisualizationPanel::onResetTime( wxCommandEvent& event )
{
  manager_->resetTime();
}

void VisualizationPanel::onViewSelected( wxCommandEvent& event )
{
  setCurrentCamera(views_->GetSelection());
}

void VisualizationPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanging( event );
}

void VisualizationPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanged( event );

  queueRender();
}

void VisualizationPanel::onPropertySelected( wxPropertyGridEvent& event )
{
  wxPGProperty* pg_property = event.GetProperty();

  selected_display_ = NULL;

  if ( !pg_property )
  {
    return;
  }

  void* client_data = pg_property->GetClientData();
  if ( client_data )
  {
    PropertyBase* property = reinterpret_cast<PropertyBase*>(client_data);

    void* user_data = property->getUserData();
    if ( user_data )
    {
      Display* display = reinterpret_cast<Display*>(user_data);

      if ( manager_->isValidDisplay( display ) )
      {
        selected_display_ = display;
      }
    }
  }
}

void VisualizationPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
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

void VisualizationPanel::onNewDisplay( wxCommandEvent& event )
{
  V_string types;
  V_string descriptions;
  manager_->getRegisteredTypes( types, descriptions );

  S_string current_display_names;
  manager_->getDisplayNames(current_display_names);

  NewDisplayDialog dialog( this, types, descriptions, current_display_names );
  while (1)
  {
    if ( dialog.ShowModal() == wxOK )
    {
      std::string type = dialog.getTypeName();
      std::string name = dialog.getDisplayName();

      if ( manager_->getDisplay( name ) != NULL )
      {
        wxMessageBox( wxT("A display with that name already exists!"), wxT("Invalid name"), wxICON_ERROR | wxOK, this );
        continue;
      }

      Display* display = manager_->createDisplay( type, name, true );
      ROS_ASSERT(display);
      (void)display;

      break;
    }
    else
    {
      break;
    }
  }
}

void VisualizationPanel::onDeleteDisplay( wxCommandEvent& event )
{
  if ( !selected_display_ )
  {
    return;
  }

  manager_->removeDisplay( selected_display_ );
  selected_display_ = NULL;
}

void VisualizationPanel::onMoveUp( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    manager_->moveDisplayUp( selected_display_ );
  }
}

void VisualizationPanel::onMoveDown( wxCommandEvent& event )
{
  if ( selected_display_ )
  {
    manager_->moveDisplayDown( selected_display_ );
  }
}

void VisualizationPanel::onDisplayStateChanged( Display* display )
{
  DisplayInfo* info = manager_->getDisplayInfo( display );
  ROS_ASSERT( info );
  wxPGProperty* property = info->category_->getPGProperty();
  ROS_ASSERT( property );

  wxPGCell* cell = property->GetCell( 0 );
  if ( !cell )
  {
    cell = new wxPGCell( property->GetLabel(), wxNullBitmap, *wxLIGHT_GREY, *wxGREEN );
    property->SetCell( 0, cell );
  }

  if ( display->isEnabled() )
  {
    cell->SetBgCol( wxColour( 32, 116, 38 ) );
  }
  else
  {
    cell->SetBgCol( wxColour( 151, 24, 41 ) );
  }
}

void VisualizationPanel::onChar( wxKeyEvent& event )
{
  manager_->handleChar( event );
}

} // namespace rviz
