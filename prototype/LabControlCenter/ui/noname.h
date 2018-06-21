///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version May 29 2018)
// http://www.wxformbuilder.org/
//
// PLEASE DO *NOT* EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __NONAME_H__
#define __NONAME_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/statbmp.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/richtext/richtextctrl.h>
#include <wx/sizer.h>
#include <wx/scrolwin.h>
#include <wx/aui/auibook.h>
#include <wx/panel.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class MainFrame
///////////////////////////////////////////////////////////////////////////////
class MainFrame : public wxFrame 
{
	private:
	
	protected:
		wxBoxSizer* mainSizer;
		wxPanel* mainPanel;
		wxBoxSizer* notebookSizer;
		wxAuiNotebook* auinotebook;
		wxScrolledWindow* m_scrolledWindow_systemOverview;
		wxStaticBitmap* m_bitmap18;
		wxRichTextCtrl* m_richText2;
		wxScrolledWindow* m_scrolledWindow_vehicles;
	
	public:
		
		MainFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("CPM Lab Control Center"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1100,800 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~MainFrame();
	
};

#endif //__NONAME_H__
