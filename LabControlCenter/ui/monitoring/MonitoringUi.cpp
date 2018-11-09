#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi(const map<uint8_t, map<string, shared_ptr<TimeSeries> > >& _vehicle_data) 
:vehicle_data(_vehicle_data)
{    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");

    builder->get_widget("window1", window);
    builder->get_widget("grid_vehicle_monitor", grid_vehicle_monitor);

    assert(window);
    assert(grid_vehicle_monitor);


    window->maximize();
    window->show_all();


    update_loop = make_shared<AbsoluteTimer>(0, 100000000, 0, 0, [&](){ update_dispatcher.emit(); });

    update_dispatcher.connect([&](){

        // Row header
        for(const auto& entry : vehicle_data) {
            const auto vehicle_id = entry.first;

            Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(vehicle_id + 1, 0));

            if(!label)
            {
                label = Gtk::manage(new Gtk::Label()); 
                label->set_width_chars(10);
                label->set_xalign(1);
                label->set_text(string_format("Vehicle %02i", vehicle_id));
                label->show_all();
                grid_vehicle_monitor->attach(*label, vehicle_id + 1, 0, 1, 1);
            }
        }

        // Column header
        if(!vehicle_data.empty())
        {
            for (size_t i = 0; i < rows.size(); ++i)
            {
                Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(0, i + 1));

                if(!label)
                {
                    label = Gtk::manage(new Gtk::Label()); 
                    label->set_width_chars(25);
                    label->set_xalign(0);
                    label->set_text(
                        vehicle_data.begin()->second.at(rows[i])->get_name() + " [" + 
                        vehicle_data.begin()->second.at(rows[i])->get_unit() + "]"
                    );
                    label->show_all();
                    grid_vehicle_monitor->attach(*label, 0, i + 1, 1, 1);
                }
            }
        }

        for(const auto& entry: vehicle_data)
        {
            const auto vehicle_id = entry.first;

            for (size_t i = 0; i < rows.size(); ++i)
            {
                auto vehicle_sensor_timeseries = entry.second;
                if(vehicle_sensor_timeseries.count(rows[i]))
                {
                    Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(vehicle_id+1, i+1));

                    if(!label)
                    {
                        label = Gtk::manage(new Gtk::Label()); 
                        label->set_width_chars(10);
                        label->set_xalign(1);
                        label->show_all();
                        grid_vehicle_monitor->attach(*label, vehicle_id+1, i+1, 1, 1);
                    }

                    auto sensor_timeseries = vehicle_sensor_timeseries.at(rows[i]);

                    const uint64_t age = clock_gettime_nanoseconds() - sensor_timeseries->get_latest_time();

                    if(sensor_timeseries->has_new_data(0.5))
                    {
                        label->set_text(sensor_timeseries->format_value(sensor_timeseries->get_latest_value()));
                    }
                    else 
                    {
                        label->set_text("---");
                    }
                }
            }
        }
    });


    window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });
}


Gtk::Window& MonitoringUi::get_window()
{
    return *window;
}
