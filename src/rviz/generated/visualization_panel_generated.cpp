///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "visualization_panel_generated.h"

///////////////////////////////////////////////////////////////////////////

RenderPanelGenerated::RenderPanelGenerated( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	render_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	wxArrayString views_Choices;
	views_ = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, views_Choices, 0 );
	views_->SetSelection( 0 );
	views_->SetMinSize( wxSize( 150,-1 ) );
	
	bSizer9->Add( views_, 0, wxALL, 5 );
	
	tools_ = new wxToolBar( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxTB_NOICONS|wxTB_TEXT ); 
	tools_->Realize();
	
	bSizer9->Add( tools_, 1, wxEXPAND, 0 );
	
	reset_time_button_ = new wxButton( this, wxID_ANY, wxT("Reset Time"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( reset_time_button_, 0, wxALL, 5 );
	
	render_sizer_->Add( bSizer9, 0, wxEXPAND, 5 );
	
	this->SetSizer( render_sizer_ );
	this->Layout();
	
	// Connect Events
	views_->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( RenderPanelGenerated::onViewSelected ), NULL, this );
	reset_time_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RenderPanelGenerated::onResetTime ), NULL, this );
}

RenderPanelGenerated::~RenderPanelGenerated()
{
	// Disconnect Events
	views_->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( RenderPanelGenerated::onViewSelected ), NULL, this );
	reset_time_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RenderPanelGenerated::onResetTime ), NULL, this );
}

DisplaysPanelGenerated::DisplaysPanelGenerated( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	properties_panel_ = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	properties_panel_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	properties_panel_->SetSizer( properties_panel_sizer_ );
	properties_panel_->Layout();
	properties_panel_sizer_->Fit( properties_panel_ );
	bSizer8->Add( properties_panel_, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxHORIZONTAL );
	
	new_display_ = new wxButton( this, wxID_ANY, wxT("Add"), wxDefaultPosition, wxDefaultSize, 0 );
	new_display_->SetToolTip( wxT("Add a new display") );
	
	bSizer7->Add( new_display_, 0, wxALL, 5 );
	
	delete_display_ = new wxButton( this, wxID_ANY, wxT("Remove"), wxDefaultPosition, wxDefaultSize, 0 );
	delete_display_->SetToolTip( wxT("Remove the selected display") );
	
	bSizer7->Add( delete_display_, 0, wxALL, 5 );
	
	down_button_ = new wxBitmapButton( this, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW );
	down_button_->SetToolTip( wxT("Move a display down in the list") );
	
	down_button_->SetToolTip( wxT("Move a display down in the list") );
	
	bSizer7->Add( down_button_, 0, wxALL, 5 );
	
	up_button_ = new wxBitmapButton( this, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW );
	up_button_->SetToolTip( wxT("Move a display up in the list") );
	
	up_button_->SetToolTip( wxT("Move a display up in the list") );
	
	bSizer7->Add( up_button_, 0, wxALL, 5 );
	
	bSizer8->Add( bSizer7, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer8 );
	this->Layout();
	
	// Connect Events
	new_display_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onNewDisplay ), NULL, this );
	delete_display_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onDeleteDisplay ), NULL, this );
	down_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onMoveDown ), NULL, this );
	up_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onMoveUp ), NULL, this );
}

DisplaysPanelGenerated::~DisplaysPanelGenerated()
{
	// Disconnect Events
	new_display_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onNewDisplay ), NULL, this );
	delete_display_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onDeleteDisplay ), NULL, this );
	down_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onMoveDown ), NULL, this );
	up_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( DisplaysPanelGenerated::onMoveUp ), NULL, this );
}

NewDisplayDialogGenerated::NewDisplayDialogGenerated( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer1;
	sbSizer1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Display Type") ), wxVERTICAL );
	
	types_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer1->Add( types_, 1, wxALL|wxEXPAND, 5 );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Description:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	sbSizer1->Add( m_staticText2, 0, wxALL, 5 );
	
	type_description_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP );
	type_description_->SetMinSize( wxSize( -1,100 ) );
	
	sbSizer1->Add( type_description_, 0, wxALL|wxEXPAND, 5 );
	
	bSizer8->Add( sbSizer1, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Display Name") ), wxVERTICAL );
	
	name_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	sbSizer2->Add( name_, 0, wxALL|wxEXPAND, 5 );
	
	bSizer8->Add( sbSizer2, 0, wxEXPAND, 5 );
	
	m_sdbSizer1 = new wxStdDialogButtonSizer();
	m_sdbSizer1OK = new wxButton( this, wxID_OK );
	m_sdbSizer1->AddButton( m_sdbSizer1OK );
	m_sdbSizer1Cancel = new wxButton( this, wxID_CANCEL );
	m_sdbSizer1->AddButton( m_sdbSizer1Cancel );
	m_sdbSizer1->Realize();
	bSizer8->Add( m_sdbSizer1, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer8 );
	this->Layout();
	
	// Connect Events
	types_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( NewDisplayDialogGenerated::onDisplaySelected ), NULL, this );
	types_->Connect( wxEVT_COMMAND_LISTBOX_DOUBLECLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onDisplayDClick ), NULL, this );
	name_->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( NewDisplayDialogGenerated::onNameEnter ), NULL, this );
	m_sdbSizer1Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onCancel ), NULL, this );
	m_sdbSizer1OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onOK ), NULL, this );
}

NewDisplayDialogGenerated::~NewDisplayDialogGenerated()
{
	// Disconnect Events
	types_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( NewDisplayDialogGenerated::onDisplaySelected ), NULL, this );
	types_->Disconnect( wxEVT_COMMAND_LISTBOX_DOUBLECLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onDisplayDClick ), NULL, this );
	name_->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( NewDisplayDialogGenerated::onNameEnter ), NULL, this );
	m_sdbSizer1Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onCancel ), NULL, this );
	m_sdbSizer1OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( NewDisplayDialogGenerated::onOK ), NULL, this );
}
