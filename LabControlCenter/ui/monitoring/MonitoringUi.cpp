#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi() 
{


    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");
    builder->get_widget("window1", window);
    assert(window);

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
