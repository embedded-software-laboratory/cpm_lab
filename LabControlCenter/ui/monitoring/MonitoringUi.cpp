#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi() 
{
    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");

    builder->get_widget("window1", window);

    assert(window);

    window->show();
    window->set_size_request(350, 200);
}


Gtk::Window& MonitoringUi::get_window() 
{
    return *window;
}
