#include <wx/wxprec.h>
#ifndef WX_PRECOMP
    #include <wx/wx.h>
#endif

#include "wx/xrc/xmlres.h"
#include <wx/notebook.h>
#include "ui/noname.h"
#include "ui/green.h"
#include "ui/red.h"
#include <thread>
#include <iostream>
#include <cmath>
#include <string>
#include <unistd.h>

class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};

wxIMPLEMENT_APP(MyApp);

class MainFrameImpl : public MainFrame 
{
    int i = 0;
public:
    MainFrameImpl() : MainFrame(NULL) {
        m_bitmap18->SetBitmap(wxBITMAP_PNG(red));
    }

    bool ping() {
        i++;


        if((i/10)%2==0) {
            m_bitmap18->SetBitmap(wxBITMAP_PNG(red));
        }
        else {
            m_bitmap18->SetBitmap(wxBITMAP_PNG(green));
        }
        
        m_richText2->Clear();
        m_richText2->BeginBold();
        m_richText2->DoWriteText("Indoor Positioning System");
        m_richText2->EndBold();
        m_richText2->Newline();
        m_richText2->Newline();
        m_richText2->DoWriteText("FPS: "+std::to_string(int(sin(0.1*i)*5+20)));
        m_richText2->Newline();
        m_richText2->DoWriteText("Active vehicles: " + std::to_string(i/10+1));
        m_richText2->SetEditable(false);

    }
};

std::thread ping_thread;

bool MyApp::OnInit()
{
    wxInitAllImageHandlers();
    MainFrameImpl *frame = new MainFrameImpl();
    frame->Show(true);
    frame->Maximize(true);  

    ping_thread = std::thread([=](){
        auto handler = frame->GetEventHandler();

        while(true) {
            usleep(100000);
            handler->CallAfter([=](){frame->ping();});
        }
    });

    return true;
}
