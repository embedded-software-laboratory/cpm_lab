///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version May 29 2018)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "noname.h"

///////////////////////////////////////////////////////////////////////////

MainFrame::MainFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	mainSizer = new wxBoxSizer( wxVERTICAL );
	
	mainPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	notebookSizer = new wxBoxSizer( wxVERTICAL );
	
	auinotebook = new wxAuiNotebook( mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_NB_TAB_MOVE|wxAUI_NB_TAB_SPLIT|wxAUI_NB_TOP );
	m_scrolledWindow_systemOverview = new wxScrolledWindow( auinotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL );
	m_scrolledWindow_systemOverview->SetScrollRate( 5, 5 );
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 0, 2, 10, 10 );
	fgSizer1->AddGrowableCol( 1 );
	fgSizer1->SetFlexibleDirection( wxVERTICAL );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_ALL );
	
	m_bitmap18 = new wxStaticBitmap( m_scrolledWindow_systemOverview, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize( 120,120 ), 0 );
	m_bitmap18->SetMinSize( wxSize( 120,120 ) );
	m_bitmap18->SetMaxSize( wxSize( 120,120 ) );
	
	fgSizer1->Add( m_bitmap18, 0, wxALL, 5 );
	
	m_richText2 = new wxRichTextCtrl( m_scrolledWindow_systemOverview, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0|wxVSCROLL|wxHSCROLL|wxNO_BORDER|wxWANTS_CHARS );
	m_richText2->SetMinSize( wxSize( -1,100 ) );
	
	fgSizer1->Add( m_richText2, 1, wxEXPAND | wxALL, 5 );
	
	
	m_scrolledWindow_systemOverview->SetSizer( fgSizer1 );
	m_scrolledWindow_systemOverview->Layout();
	fgSizer1->Fit( m_scrolledWindow_systemOverview );
	auinotebook->AddPage( m_scrolledWindow_systemOverview, wxT("System Overview"), true, wxNullBitmap );
	m_scrolledWindow_vehicles = new wxScrolledWindow( auinotebook, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL );
	m_scrolledWindow_vehicles->SetScrollRate( 5, 5 );
	wxBoxSizer* bSizer_vehicles;
	bSizer_vehicles = new wxBoxSizer( wxVERTICAL );
	
	
	m_scrolledWindow_vehicles->SetSizer( bSizer_vehicles );
	m_scrolledWindow_vehicles->Layout();
	bSizer_vehicles->Fit( m_scrolledWindow_vehicles );
	auinotebook->AddPage( m_scrolledWindow_vehicles, wxT("Vehicles"), false, wxNullBitmap );
	
	notebookSizer->Add( auinotebook, 1, wxEXPAND | wxALL, 5 );
	
	
	mainPanel->SetSizer( notebookSizer );
	mainPanel->Layout();
	notebookSizer->Fit( mainPanel );
	mainSizer->Add( mainPanel, 1, wxEXPAND | wxALL, 5 );
	
	
	this->SetSizer( mainSizer );
	this->Layout();
	
	this->Centre( wxBOTH );
}

MainFrame::~MainFrame()
{
}
