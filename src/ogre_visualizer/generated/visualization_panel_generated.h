///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __visualization_panel_generated__
#define __visualization_panel_generated__

#include <wx/string.h>
#include <wx/choice.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/toolbar.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/bmpbuttn.h>
#include <wx/listbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statbox.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class RenderPanelGenerated
///////////////////////////////////////////////////////////////////////////////
class RenderPanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxBoxSizer* render_sizer_;
		wxChoice* views_;
		wxToolBar* tools_;
		wxButton* reset_time_button_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onViewSelected( wxCommandEvent& event ){ event.Skip(); }
		virtual void onResetTime( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		RenderPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxTAB_TRAVERSAL );
		~RenderPanelGenerated();
	
};

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
		wxListBox* types_;
		wxStaticText* m_staticText2;
		wxTextCtrl* type_description_;
		wxTextCtrl* name_;
		wxStdDialogButtonSizer* m_sdbSizer1;
		wxButton* m_sdbSizer1OK;
		wxButton* m_sdbSizer1Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onDisplaySelected( wxCommandEvent& event ){ event.Skip(); }
		virtual void onDisplayDClick( wxCommandEvent& event ){ event.Skip(); }
		virtual void onNameEnter( wxCommandEvent& event ){ event.Skip(); }
		virtual void onCancel( wxCommandEvent& event ){ event.Skip(); }
		virtual void onOK( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		NewDisplayDialogGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("New Display"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 485,497 ), long style = wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER );
		~NewDisplayDialogGenerated();
	
};

#endif //__visualization_panel_generated__
