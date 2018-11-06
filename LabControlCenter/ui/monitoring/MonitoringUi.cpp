#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi() 
{
    window = make_shared<Gtk::Window>();
    window->set_default_size( 800, 600 );

    this->webKitWebView = WEBKIT_WEB_VIEW( webkit_web_view_new() );
    this->gtkmm_webKitWebView = Glib::wrap( GTK_WIDGET( this->webKitWebView ) );

    window->add( *(this->gtkmm_webKitWebView) );
    webkit_web_view_load_html(this->webKitWebView, "<html><head><style>h1 {color: red;}</style><title>title</title></head><body><h1>header</h1><b>bold</b><i>italic</i></body></html>", NULL);

    window->set_title("Lab Monitoring");
    window->maximize();
    window->show_all();
}


Gtk::Window& MonitoringUi::get_window() 
{
    return *window;
}
