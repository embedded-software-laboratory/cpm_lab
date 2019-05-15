#include "MonitoringUi.hpp"
#include <cassert>

MonitoringUi::MonitoringUi(std::function<VehicleData()> get_vehicle_data_callback)
{
    this->get_vehicle_data = get_vehicle_data_callback;

    grid_vehicle_monitor = Gtk::manage(new Gtk::Grid()); 

    assert(grid_vehicle_monitor);

    grid_vehicle_monitor->set_name("grid_vehicle_monitor");
    grid_vehicle_monitor->show();




    update_loop = cpm::Timer::create("LabControlCenterMonitor",100000000ull, 0, false, false);
    update_loop->start_async([&](uint64_t t_now){ update_dispatcher.emit(); });

    update_dispatcher.connect([&](){

        auto vehicle_data = this->get_vehicle_data();

        // Top header
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

        // Left header
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

                    if(sensor_timeseries->has_new_data(0.5))
                    {
                        const auto value = sensor_timeseries->get_latest_value();
                        label->set_text(sensor_timeseries->format_value(value));

                        label->get_style_context()->remove_class("ok");
                        label->get_style_context()->remove_class("warn");
                        label->get_style_context()->remove_class("alert");


                        if(rows[i] == "battery_voltage")
                        {
                            if     (value > 6.6) label->get_style_context()->add_class("ok");
                            else if(value > 6.3) label->get_style_context()->add_class("warn");
                            else                 label->get_style_context()->add_class("alert");
                        }
                        else if(rows[i] == "clock_delta") 
                        {
                            if     (fabs(value) < 50)  label->get_style_context()->add_class("ok");
                            else if(fabs(value) < 500) label->get_style_context()->add_class("warn");
                            else                       label->get_style_context()->add_class("alert");
                        }

                    }
                    else 
                    {
                        label->set_text("---");

                        label->get_style_context()->remove_class("ok");
                        label->get_style_context()->remove_class("warn");
                        label->get_style_context()->add_class("alert");
                    }
                }
            }
        }
    });


}



Gtk::Grid* MonitoringUi::get_parent()
{
    return grid_vehicle_monitor;
}