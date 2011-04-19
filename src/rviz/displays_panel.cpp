
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

#include "displays_panel.h"
#include "visualization_manager.h"
#include "display.h"
#include "display_wrapper.h"
#include "new_display_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "plugin/type_registry.h"
#include "plugin/plugin_manager.h"
#include "plugin/plugin.h"

#include <wx/propgrid/propgrid.h>
#include <wx/msgdlg.h>
#include <wx/confbase.h>
#include <wx/artprov.h>
#include <wx/timer.h>
#include <wx/textdlg.h>

#include <boost/bind.hpp>

static const wxString PROPERTY_GRID_CONFIG(wxT("Property Grid State"));

namespace rviz
{

class ManageDisplaysDialog : public ManageDisplaysDialogGenerated
{
public:
  ManageDisplaysDialog(V_DisplayWrapper& displays, VisualizationManager* manager, wxWindow* parent);

  virtual void onRename( wxCommandEvent& event );
  virtual void onRemove( wxCommandEvent& event );
  virtual void onRemoveAll( wxCommandEvent& event );
  virtual void onMoveUp( wxCommandEvent& event );
  virtual void onMoveDown( wxCommandEvent& event );
  virtual void onOK( wxCommandEvent& event );

  V_DisplayWrapper& displays_;
  VisualizationManager* manager_;
};

ManageDisplaysDialog::ManageDisplaysDialog(V_DisplayWrapper& displays, VisualizationManager* manager, wxWindow* parent)
: ManageDisplaysDialogGenerated(parent)
, displays_(displays)
, manager_(manager)
{
  move_up_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_UP, wxART_OTHER, wxSize(16,16) ) );
  move_down_->SetBitmapLabel( wxArtProvider::GetIcon( wxART_GO_DOWN, wxART_OTHER, wxSize(16,16) ) );

  V_DisplayWrapper::iterator it = displays_.begin();
  V_DisplayWrapper::iterator end = displays_.end();
  for (; it != end; ++it)
  {
    DisplayWrapper* wrapper = *it;
    const std::string& name = wrapper->getName();
    listbox_->Append(wxString::FromAscii(name.c_str()));
  }
}

void ManageDisplaysDialog::onRename( wxCommandEvent& event )
{
  int sel = listbox_->GetSelection();
  if (sel < 0)
  {
    return;
  }

  bool ok = true;
  wxString new_name;
  do
  {
    if (!ok)
    {
      new_name = wxGetTextFromUser(wxT("That name is already taken.  Please try another."), wxT("Rename Display"), listbox_->GetString(sel), this);
    }
    else
    {
      new_name = wxGetTextFromUser(wxT("New Name?"), wxT("Rename Display"), listbox_->GetString(sel), this);
    }

    ok = true;
    if (new_name.IsEmpty() || new_name == listbox_->GetString(sel))
    {
      return;
    }

    // Make sure the new name is not already taken
    V_DisplayWrapper::iterator it = displays_.begin();
    V_DisplayWrapper::iterator end = displays_.end();
    for (; it != end; ++it)
    {
      DisplayWrapper* wrapper = *it;
      if (wrapper->getName() == (const char*)new_name.mb_str())
      {
        ok = false;
        break;
      }
    }
  } while (!ok);

  displays_[sel]->setName((const char*)new_name.mb_str());
  listbox_->SetString(sel, new_name);
}

void ManageDisplaysDialog::onRemove(wxCommandEvent& event)
{
  int sel = listbox_->GetSelection();
  if (sel < 0)
  {
    return;
  }

  manager_->removeDisplay(displays_[sel]);
  listbox_->Delete(sel);
  if (sel < listbox_->GetCount())
  {
    listbox_->SetSelection(sel);
  }
  else if (sel > 0)
  {
    listbox_->SetSelection(sel - 1);
  }
}

void ManageDisplaysDialog::onRemoveAll(wxCommandEvent& event)
{
  manager_->removeAllDisplays();
  listbox_->Clear();
}

void ManageDisplaysDialog::onMoveUp( wxCommandEvent& event )
{
  int sel = listbox_->GetSelection();
  if (sel > 0)
  {
    std::swap(displays_[sel], displays_[sel - 1]);
    listbox_->Insert(listbox_->GetString(sel - 1), sel + 1);
    listbox_->Delete(sel - 1);
    listbox_->SetSelection(sel - 1);
  }
}

void ManageDisplaysDialog::onMoveDown( wxCommandEvent& event )
{
  int sel = listbox_->GetSelection();
  if (sel >= 0 && sel < listbox_->GetCount() - 1)
  {
    std::swap(displays_[sel], displays_[sel + 1]);
    listbox_->Insert(listbox_->GetString(sel + 1), sel);
    listbox_->Delete(sel + 2);
    listbox_->SetSelection(sel + 1);
  }
}

void ManageDisplaysDialog::onOK( wxCommandEvent& event )
{
  EndModal(wxOK);
}

DisplaysPanel::DisplaysPanel( wxWindow* parent )
: DisplaysPanelGenerated( parent )
, manager_(NULL)
{
  property_grid_ = new wxPropertyGrid( properties_panel_, wxID_ANY, wxDefaultPosition, wxSize(500, 500), wxPG_SPLITTER_AUTO_CENTER | wxPG_DEFAULT_STYLE );
  properties_panel_sizer_->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle(wxPG_EX_DISABLE_TLP_TRACKING);

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( DisplaysPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( DisplaysPanel::onPropertySelected ), NULL, this );
  property_grid_->Connect( wxEVT_PG_HIGHLIGHTED, wxPropertyGridEventHandler( DisplaysPanel::onPropertyHighlighted ), NULL, this );

  property_grid_->SetCaptionBackgroundColour( wxColour( 4, 89, 127 ) );
  property_grid_->SetCaptionForegroundColour( *wxWHITE );

  help_html_->Connect(wxEVT_COMMAND_HTML_LINK_CLICKED, wxHtmlLinkEventHandler(DisplaysPanel::onLinkClicked), NULL, this);

  state_changed_timer_ = new wxTimer(this);
  state_changed_timer_->Start(200);
  Connect(state_changed_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(DisplaysPanel::onStateChangedTimer), NULL, this);
}

DisplaysPanel::~DisplaysPanel()
{
  delete state_changed_timer_;

  property_grid_->Destroy();
}

void DisplaysPanel::initialize(VisualizationManager* manager)
{
  manager_ = manager;
  manager_->getDisplayAddingSignal().connect( boost::bind( &DisplaysPanel::onDisplayAdding, this, _1 ) );
  manager_->getDisplayAddedSignal().connect( boost::bind( &DisplaysPanel::onDisplayAdded, this, _1 ) );
  manager_->getDisplayRemovingSignal().connect( boost::bind( &DisplaysPanel::onDisplayRemoving, this, _1 ) );
  manager_->getDisplayRemovedSignal().connect( boost::bind( &DisplaysPanel::onDisplayRemoved, this, _1 ) );
  manager_->getDisplaysRemovingSignal().connect( boost::bind( &DisplaysPanel::onDisplaysRemoving, this, _1 ) );
  manager_->getDisplaysRemovedSignal().connect( boost::bind( &DisplaysPanel::onDisplaysRemoved, this, _1 ) );
  manager_->getDisplaysConfigLoadedSignal().connect( boost::bind( &DisplaysPanel::onDisplaysConfigLoaded, this, _1 ) );
  manager_->getDisplaysConfigSavingSignal().connect( boost::bind( &DisplaysPanel::onDisplaysConfigSaving, this, _1 ) );

  manager_->getPropertyManager()->setPropertyGrid(property_grid_);

  sortDisplays();
}

void DisplaysPanel::sortDisplays()
{
  property_grid_->Freeze();
  property_grid_->Sort(property_grid_->GetRoot());
  property_grid_->Refresh();
  property_grid_->Thaw();
}

void DisplaysPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanging( event );
}

void DisplaysPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanged( event );
}

void DisplaysPanel::onPropertySelected( wxPropertyGridEvent& event )
{
  // Hack to fix bug #4885.  If I put this in the constructor or in
  // initialize() this has no effect.  Similarly,
  // GetSizer()->SetMinSize() has no effect.  Only directly calling
  // this->SetMinSize() actually changes anything.  If the layout
  // structure is changed, these calls to GetItem(1) and (2) may need
  // to change. -hersh
  wxSize size = GetMinSize();
  int height = 0;
  height += GetSizer()->GetItem(1)->GetMinSize().GetHeight();
  height += GetSizer()->GetItem(2)->GetMinSize().GetHeight();
  size.SetHeight( height );
  SetMinSize( size );

  wxPGProperty* pg_property = event.GetProperty();

  selected_display_ = 0;

  if ( !pg_property )
  {
    return;
  }

  wxString text = pg_property->GetHelpString();
  wxString html = wxT("<html><body bgcolor=\"#EFEBE7\"><strong>") + pg_property->GetLabel() + wxT("</strong><br>") + text + wxT("</body></html>");

  help_html_->SetPage(html);

  void* client_data = pg_property->GetClientData();
  if ( client_data )
  {
    PropertyBase* property = reinterpret_cast<PropertyBase*>(client_data);

    void* user_data = property->getUserData();
    if ( user_data )
    {
      DisplayWrapper* wrapper = reinterpret_cast<DisplayWrapper*>(user_data);

      if ( manager_->isValidDisplay( wrapper ) )
      {
        selected_display_ = manager_->getDisplayWrapper(wrapper->getName());
      }
      else
      {
        DisplayWrapper* wrapper = manager_->getDisplayWrapper(reinterpret_cast<Display*>(user_data));

        if (wrapper)
        {
          selected_display_ = wrapper;
        }
      }
    }
  }
}

void DisplaysPanel::onPropertyHighlighted( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  /*
  wxString text = property->GetHelpString();
  wxString html = wxT("<html><body bgcolor=\"#EFEBE7\"><strong>") + property->GetLabel() + wxT("</strong><br>") + text + wxT("</body></html>");

  help_html_->SetPage(html);
  */
}

void DisplaysPanel::onLinkClicked(wxHtmlLinkEvent& event)
{
  wxLaunchDefaultBrowser(event.GetLinkInfo().GetHref());
}

void DisplaysPanel::onNewDisplay( wxCommandEvent& event )
{
  L_DisplayTypeInfo display_types;
  PluginManager* pm = manager_->getPluginManager();

  S_string current_display_names;
  manager_->getDisplayNames(current_display_names);

  NewDisplayDialog dialog( this, pm->getPlugins(), current_display_names );
  while (1)
  {
    if ( dialog.ShowModal() == wxOK )
    {
      std::string class_name = dialog.getClassName();
      std::string name = dialog.getDisplayName();
      std::string package = dialog.getPackageName();

      if (manager_->getDisplayWrapper(name))
      {
        wxMessageBox( wxT("A display with that name already exists!"), wxT("Invalid name"), wxICON_ERROR | wxOK, this );
        continue;
      }

      DisplayWrapper* wrapper = manager_->createDisplay( package, class_name, name, true );
      (void)wrapper;
      break;
    }
    else
    {
      break;
    }
  }
}

void DisplaysPanel::onDeleteDisplay( wxCommandEvent& event )
{
  DisplayWrapper* selected = selected_display_;
  if ( !selected )
  {
    return;
  }

  manager_->removeDisplay(selected);
  selected_display_ = 0;
}

void DisplaysPanel::onManage(wxCommandEvent& event)
{
  V_DisplayWrapper& displays = manager_->getDisplays();
  ManageDisplaysDialog d(displays, manager_, this);
  d.ShowModal();

  // Remap the displays based on the new indices
  {
    display_map_.clear();
    V_DisplayWrapper::iterator it = displays.begin();
    V_DisplayWrapper::iterator end = displays.end();
    for (; it != end; ++it)
    {
      DisplayWrapper* display = *it;
      uint32_t index = it - displays.begin();
      display_map_[display] = index;
      setDisplayCategoryLabel(display, index);
    }
  }

  sortDisplays();
}

void DisplaysPanel::setDisplayCategoryLabel(const DisplayWrapper* wrapper, int index)
{
  std::string display_name;
  if (wrapper->isLoaded())
  {
    display_name = wrapper->getTypeInfo()->display_name;
  }
  else
  {
    display_name = "Plugin from package [" + wrapper->getPackage() + "] not loaded for display class [" + wrapper->getClassName() + "]";
  }

  char buf[1024];
  snprintf( buf, 1024, "%02d. %s (%s)", index + 1, wrapper->getName().c_str(), display_name.c_str());
  wrapper->getCategory().lock()->setLabel(buf);
}

void DisplaysPanel::setDisplayCategoryColor(const DisplayWrapper* wrapper)
{
  CategoryPropertyPtr cat = wrapper->getCategory().lock();
  wxPGProperty* property = wrapper->getCategory().lock()->getPGProperty();

  wxPGCell* cell = property->GetCell( 0 );
  if ( !cell )
  {
    cell = new wxPGCell(*(wxString*)0, wxNullBitmap, wxNullColour, wxNullColour);
    property->SetCell( 0, cell );
  }

  if (!wrapper->isLoaded())
  {
    cat->setToError();
  }
  else if ( wrapper->getDisplay()->isEnabled() )
  {
    switch (wrapper->getDisplay()->getStatus())
    {
    case status_levels::Ok:
      cat->setToOK();
      break;
    case status_levels::Warn:
      cat->setToWarn();
      break;
    case status_levels::Error:
      cat->setToError();
      break;
    }
  }
  else
  {
    cat->setToDisabled();
  }
}

void DisplaysPanel::onStateChangedTimer(wxTimerEvent& event)
{
  S_Display local_displays;
  {
    boost::mutex::scoped_lock lock(state_changed_displays_mutex_);
    local_displays.swap(state_changed_displays_);
  }

  S_Display::iterator it = local_displays.begin();
  S_Display::iterator end = local_displays.end();
  for (; it != end; ++it)
  {
    Display* display = *it;
    DisplayWrapper* wrapper = manager_->getDisplayWrapper(display);
    if (!wrapper)
    {
      continue;
    }

    M_DisplayToIndex::iterator it = display_map_.find(wrapper);
    if (it == display_map_.end())
    {
      continue;
    }

    int index = it->second;
    setDisplayCategoryColor(wrapper);
    setDisplayCategoryLabel(wrapper, index);
  }
}

void DisplaysPanel::onDisplayStateChanged( Display* display )
{
  // This can be called from different threads, so we have to push this to the GUI update thread
  boost::mutex::scoped_lock lock(state_changed_displays_mutex_);
  state_changed_displays_.insert(display);

}

void DisplaysPanel::onDisplayCreated( DisplayWrapper* wrapper )
{
  wrapper->getDisplay()->getStateChangedSignal().connect( boost::bind( &DisplaysPanel::onDisplayStateChanged, this, _1 ) );

  setDisplayCategoryColor(wrapper);

  Refresh();
}

void DisplaysPanel::onDisplayDestroyed( DisplayWrapper* wrapper )
{
  M_DisplayToIndex::iterator it = display_map_.find(wrapper);
  if (it == display_map_.end())
  {
    return;
  }

  setDisplayCategoryColor(wrapper);


  int index = it->second;
  setDisplayCategoryLabel(wrapper, index);

  Refresh();
}

void DisplaysPanel::onDisplayAdding( DisplayWrapper* wrapper )
{
  property_grid_->Freeze();

  wrapper->getDisplayCreatedSignal().connect(boost::bind(&DisplaysPanel::onDisplayCreated, this, _1));
  wrapper->getDisplayDestroyedSignal().connect(boost::bind(&DisplaysPanel::onDisplayDestroyed, this, _1));
}

void DisplaysPanel::onDisplayAdded( DisplayWrapper* wrapper )
{
  int index = display_map_.size();
  bool inserted = display_map_.insert(std::make_pair(wrapper, index)).second;
  ROS_ASSERT(inserted);
  setDisplayCategoryLabel(wrapper, index);
  setDisplayCategoryColor(wrapper);

  property_grid_->Refresh();
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplayRemoving( DisplayWrapper* wrapper )
{
  property_grid_->Freeze();
}

void DisplaysPanel::onDisplayRemoved( DisplayWrapper* wrapper )
{
  M_DisplayToIndex::iterator it = display_map_.find(wrapper);
  ROS_ASSERT(it != display_map_.end());

  uint32_t index = it->second;

  display_map_.erase(it);

  it = display_map_.begin();
  M_DisplayToIndex::iterator end = display_map_.end();
  for (;it != end; ++it)
  {
    if (it->second > index)
    {
      --it->second;
      setDisplayCategoryLabel(it->first, it->second);
    }
  }

  sortDisplays();

  property_grid_->Refresh();
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplaysRemoving( const V_DisplayWrapper& displays )
{
  property_grid_->Freeze();
}

void DisplaysPanel::onDisplaysRemoved( const V_DisplayWrapper& displays )
{
  property_grid_->Thaw();
}

void DisplaysPanel::onDisplaysConfigLoaded(const boost::shared_ptr<wxConfigBase>& config)
{
  wxString grid_state;
  if ( config->Read( PROPERTY_GRID_CONFIG, &grid_state ) )
  {
    property_grid_->RestoreEditableState( grid_state );
  }
}

void DisplaysPanel::onDisplaysConfigSaving(const boost::shared_ptr<wxConfigBase>& config)
{
  config->Write( PROPERTY_GRID_CONFIG, property_grid_->SaveEditableState() );
}

} // namespace rviz
