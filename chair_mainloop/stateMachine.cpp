///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Jun  6 2014)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "stateMachine.h"

///////////////////////////////////////////////////////////////////////////

MyFrame1::MyFrame1( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer_detect;
	bSizer_detect = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText1 = new wxStaticText( this, wxID_ANY, wxT("Detected Object:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	bSizer_detect->Add( m_staticText1, 0, wxALL, 5 );
	
	m_Text_obj = new wxStaticText( this, wxID_ANY, wxT("MyLabel"), wxDefaultPosition, wxDefaultSize, 0 );
	m_Text_obj->Wrap( -1 );
	m_Text_obj->SetFont( wxFont( 26, 74, 90, 90, false, wxT("Sans") ) );
	
	bSizer_detect->Add( m_Text_obj, 0, wxALL, 5 );
    
//    m_Text_obj->SetLabel(wxT("Fuck!"));
	
	
	bSizer1->Add( bSizer_detect, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Current State:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer3->Add( m_staticText2, 0, wxALL, 5 );
	
	m_Text_state = new wxStaticText( this, wxID_ANY, wxT("MyLabel"), wxDefaultPosition, wxDefaultSize, 0 );
	m_Text_state->Wrap( -1 );
	m_Text_state->SetFont( wxFont( 26, 74, 90, 90, false, wxT("Sans") ) );
	
	bSizer3->Add( m_Text_state, 0, wxALL, 5 );
	
	
	bSizer1->Add( bSizer3, 1, wxEXPAND, 5 );
	
	m_Text_note = new wxStaticText( this, wxID_ANY, wxT("Be Careful when driving."), wxDefaultPosition, wxDefaultSize, 0 );
	m_Text_note->Wrap( -1 );
	bSizer1->Add( m_Text_note, 0, wxALL, 5 );
	
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	this->Centre( wxBOTH );
}

MyFrame1::~MyFrame1()
{
}
