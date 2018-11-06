#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "defaults.hpp"
#include <webkit2/webkit2.h>

class MonitoringUi
{

    shared_ptr<Gtk::Window> window = nullptr;
    WebKitWebView* webKitWebView;
    Gtk::Widget* gtkmm_webKitWebView;
    
public:
    MonitoringUi();


    Gtk::Window& get_window();
    
};