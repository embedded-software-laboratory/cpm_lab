#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi() 
{


    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");
    builder->get_widget("window1", window);

    Gtk::Entry* entry2;
    builder->get_widget("entry2", entry2);
    assert(window);

    entry2->get_style_context()->add_class("asd");

    window->maximize();
    window->show_all();


    window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });
}


Gtk::Window& MonitoringUi::get_window() 
{
    return *window;
}
