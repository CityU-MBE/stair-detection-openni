///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun  6 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "freeUI.h"

///////////////////////////////////////////////////////////////////////////

MyFrame1::MyFrame1( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	container = new wxStaticBoxSizer( new wxStaticBox( this, wxID_container, wxT("Manual Operations") ), wxVERTICAL );
	
	m_button_UGV = new wxButton( this, wxID_ANY, wxT("BUMPS"), wxDefaultPosition, wxDefaultSize, 0 );
	container->Add( m_button_UGV, 1, wxALL|wxEXPAND, 5 );
	
	m_button_walk = new wxButton( this, wxID_ANY, wxT("WALK"), wxDefaultPosition, wxDefaultSize, 0 );
	container->Add( m_button_walk, 1, wxALL|wxEXPAND, 5 );
	
	m_button_stair = new wxButton( this, wxID_ANY, wxT("STAIR"), wxDefaultPosition, wxDefaultSize, 0 );
	container->Add( m_button_stair, 1, wxALL|wxEXPAND, 5 );
	
	m_button_table = new wxButton( this, wxID_ANY, wxT("TABLE"), wxDefaultPosition, wxDefaultSize, 0 );
	container->Add( m_button_table, 1, wxALL|wxEXPAND, 5 );
	
	
	this->SetSizer( container );
	this->Layout();
	
	// Connect Events
	m_button_UGV->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::ugv_mode ), NULL, this );
	m_button_walk->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::walk_mode ), NULL, this );
	m_button_stair->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::stair_mode ), NULL, this );
	m_button_table->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::table_mode ), NULL, this );
}

MyFrame1::~MyFrame1()
{
	// Disconnect Events
	m_button_UGV->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::ugv_mode ), NULL, this );
	m_button_walk->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::walk_mode ), NULL, this );
	m_button_stair->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::stair_mode ), NULL, this );
	m_button_table->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MyFrame1::table_mode ), NULL, this );
	
}
