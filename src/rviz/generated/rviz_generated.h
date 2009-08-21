///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __rviz_generated__
#define __rviz_generated__

#include <wx/sizer.h>
#include <wx/gdicmn.h>
#include <wx/panel.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/button.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/bmpbuttn.h>
#include <wx/treectrl.h>
#include <wx/stattext.h>
#include <wx/html/htmlwin.h>
#include <wx/statbox.h>
#include <wx/textctrl.h>
#include <wx/dialog.h>
#include <wx/choice.h>
#include <wx/listbox.h>
#include <wx/scrolwin.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class DisplaysPanelGenerated
///////////////////////////////////////////////////////////////////////////////
class DisplaysPanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxPanel* properties_panel_;
		wxBoxSizer* properties_panel_sizer_;
		wxButton* new_display_;
		wxButton* delete_display_;
		wxBitmapButton* down_button_;
		wxBitmapButton* up_button_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onNewDisplay( wxCommandEvent& event ){ event.Skip(); }
		virtual void onDeleteDisplay( wxCommandEvent& event ){ event.Skip(); }
		virtual void onMoveDown( wxCommandEvent& event ){ event.Skip(); }
		virtual void onMoveUp( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		DisplaysPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxTAB_TRAVERSAL );
		~DisplaysPanelGenerated();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class NewDisplayDialogGenerated
///////////////////////////////////////////////////////////////////////////////
class NewDisplayDialogGenerated : public wxDialog 
{
	private:
	
	protected:
		wxTreeCtrl* types_;
		wxStaticText* m_staticText2;
		wxHtmlWindow* type_description_;
		wxTextCtrl* name_;
		wxStdDialogButtonSizer* m_sdbSizer1;
		wxButton* m_sdbSizer1OK;
		wxButton* m_sdbSizer1Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onDisplayDClick( wxMouseEvent& event ){ event.Skip(); }
		virtual void onDisplaySelected( wxTreeEvent& event ){ event.Skip(); }
		virtual void onNameEnter( wxCommandEvent& event ){ event.Skip(); }
		virtual void onCancel( wxCommandEvent& event ){ event.Skip(); }
		virtual void onOK( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		NewDisplayDialogGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("New Display"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 485,497 ), long style = wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER );
		~NewDisplayDialogGenerated();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class ViewsPanelGenerated
///////////////////////////////////////////////////////////////////////////////
class ViewsPanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxChoice* camera_types_;
		wxListBox* views_list_;
		wxButton* save_button_;
		wxButton* load_button_;
		wxButton* delete_button_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onCameraTypeSelected( wxCommandEvent& event ){ event.Skip(); }
		virtual void onViewsClicked( wxCommandEvent& event ){ event.Skip(); }
		virtual void onViewsDClicked( wxCommandEvent& event ){ event.Skip(); }
		virtual void onSaveClicked( wxCommandEvent& event ){ event.Skip(); }
		virtual void onLoadClicked( wxCommandEvent& event ){ event.Skip(); }
		virtual void onDeleteClicked( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		ViewsPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 236,242 ), long style = wxTAB_TRAVERSAL );
		~ViewsPanelGenerated();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class TimePanelGenerated
///////////////////////////////////////////////////////////////////////////////
class TimePanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText2;
		wxTextCtrl* wall_time_;
		wxStaticText* m_staticText21;
		wxTextCtrl* wall_elapsed_;
		wxStaticText* m_staticText22;
		wxTextCtrl* ros_time_;
		wxStaticText* m_staticText23;
		wxTextCtrl* ros_elapsed_;
		wxButton* reset_button_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onReset( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		TimePanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 917,46 ), long style = wxTAB_TRAVERSAL );
		~TimePanelGenerated();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class PluginManagerDialogGenerated
///////////////////////////////////////////////////////////////////////////////
class PluginManagerDialogGenerated : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* m_staticText9;
		wxScrolledWindow* scrolled_window_;
		wxBoxSizer* plugins_sizer_;
		wxStdDialogButtonSizer* m_sdbSizer2;
		wxButton* m_sdbSizer2OK;
	
	public:
		PluginManagerDialogGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Manage Plugins"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 651,357 ), long style = wxDEFAULT_DIALOG_STYLE );
		~PluginManagerDialogGenerated();
	
};

#endif //__rviz_generated__
