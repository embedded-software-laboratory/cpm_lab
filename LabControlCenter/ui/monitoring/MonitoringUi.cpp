#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi() 
{    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");

    builder->get_widget("window1", window);
    builder->get_widget("grid_vehicle_monitor", grid_vehicle_monitor);

    assert(window);
    assert(grid_vehicle_monitor);


    for (int i = 0; i < 15; ++i)
    {
        for (int j = 0; j < 14; ++j)
        {
            Gtk::Label* aWidget = Gtk::manage(new Gtk::Label()); 
            aWidget->set_text(to_string(i)+", "+to_string(j));
            aWidget->set_width_chars(10);
            aWidget->set_xalign(1);

            if(i == 0 && j == 0) aWidget->get_style_context()->add_class("ok");
            if(i == 1 && j == 1) aWidget->get_style_context()->add_class("warn");
            if(i == 2 && j == 2) aWidget->get_style_context()->add_class("alert");

            grid_vehicle_monitor->attach (*aWidget, i, j, 1, 1);
        }
    }

    window->maximize();
    window->show_all();

    function<int()> update_fn = [=](){

        for (int i = 0; i < 15; ++i)
        {
            for (int j = 0; j < 14; ++j)
            {
                Gtk::Label* aWidget = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(i,j));

                double p = frand();
                aWidget->set_text(to_string(p));

                aWidget->get_style_context()->remove_class("ok");
                aWidget->get_style_context()->remove_class("warn");
                aWidget->get_style_context()->remove_class("alert");

                if(p < 0.1)      aWidget->get_style_context()->add_class("ok");
                else if(p > 0.9) aWidget->get_style_context()->add_class("warn");
                else if(p > 0.8) aWidget->get_style_context()->add_class("alert");
            }
        }
        return 0;
    };

    for (int i = 0; i < 100; ++i)
    {
        Glib::signal_timeout().connect(update_fn, i*500);
    }


    window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });
}


Gtk::Window& MonitoringUi::get_window()
{
    return *window;
}
