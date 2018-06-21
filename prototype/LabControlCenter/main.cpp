#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include "wx/xrc/xmlres.h"
#include <wx/notebook.h>
#include "ui/noname.h"
#include "ui/green.h"
#include "ui/red.h"

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

wxIMPLEMENT_APP(MyApp);

class MainFrameImpl : public MainFrame 
{
public:
    MainFrameImpl() : MainFrame(NULL) {
        m_bitmap18->SetBitmap(wxBITMAP_PNG(red));
    }
};


bool MyApp::OnInit()
{
    wxInitAllImageHandlers();
    MainFrameImpl *frame = new MainFrameImpl();
    frame->Show(true);
    frame->Maximize(true);  
    return true;
}
