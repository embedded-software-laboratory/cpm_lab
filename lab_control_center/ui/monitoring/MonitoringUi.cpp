// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "MonitoringUi.hpp"
#include <numeric>
#include <cassert>


MonitoringUi::MonitoringUi(
    std::shared_ptr<Deploy> deploy_functions_callback, 
    std::function<VehicleData()> get_vehicle_data_callback, 
    std::function<std::vector<uint8_t>()> get_hlc_data_callback, 
    std::function<VehicleTrajectories()> get_vehicle_trajectory_command_callback, 
    std::function<void()> reset_data_callback)
{
    this->deploy_functions = deploy_functions_callback;
    this->get_vehicle_data = get_vehicle_data_callback;
    this->get_hlc_data = get_hlc_data_callback;
    this->get_vehicle_trajectory = get_vehicle_trajectory_command_callback;
    this->reset_data = reset_data_callback;

    builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");
    builder->get_widget("parent", parent);
    builder->get_widget("viewport_monitoring", viewport_monitoring);
    builder->get_widget("grid_vehicle_monitor", grid_vehicle_monitor);
    builder->get_widget("button_reset_view", button_reset_view);
    builder->get_widget("box_buttons", box_buttons);
    builder->get_widget("label_hlc_description_short", label_hlc_description_short);
    builder->get_widget("label_hlc_description_long", label_hlc_description_long);

    assert(parent);
    assert(viewport_monitoring);
    assert(grid_vehicle_monitor);
    assert(button_reset_view);
    assert(box_buttons);
    assert(label_hlc_description_short);
    assert(label_hlc_description_long);

    //Warning: Most style options are set in Glade (style classes etc) and style.css

    //Initialize the UI thread that updates the view on connected / online vehicles as well as connected / online hlcs
    init_ui_thread();

    //Register the button callback for resetting the vehicle monitoring view (allows to delete old entries)
    button_reset_view->signal_clicked().connect(sigc::mem_fun(this, &MonitoringUi::reset_ui_thread));
}

MonitoringUi::~MonitoringUi()
{
    stop_ui_thread();
}



void MonitoringUi::init_ui_thread()
{
    //Set UI dispatcher function, create UI update thread
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

        // Left header - rows contains all relevant row names, row_restricted containts the row(s) (names) that are relevant to the user (and are thus shown)
        if(!vehicle_data.empty())
        {
            for (size_t i = 0; i < rows_restricted.size(); ++i)
            {
                Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(0, i + 1));

                if(!label)
                {
                    label = Gtk::manage(new Gtk::Label()); 
                    label->set_width_chars(25);
                    label->set_xalign(0);
                    //Some rows may be empty (serve as separator) - ignore those
                    if (rows_restricted[i] != "" && rows_restricted[i] != "nuc_connected")
                    {
                        label->set_text(
                            vehicle_data.begin()->second.at(rows_restricted[i])->get_name() + " [" + 
                            vehicle_data.begin()->second.at(rows_restricted[i])->get_unit() + "]"
                        );
                    }
                    else if (rows_restricted[i] == "nuc_connected")
                    {
                        //This is not part of the time series data, so we need a special case for this
                        //Show if the NUC with the ID of the vehicle is online (= sends data from autostart program to LCC)
                        label->set_text("NUC connected");
                    }
                    else 
                    {
                        //Set other label style to make the row smaller than the surrounding rows
                        label->get_style_context()->add_class("small_text");
                    }
                    label->show_all();
                    grid_vehicle_monitor->attach(*label, 0, i + 1, 1, 1);
                }
            }
        }
        // get all IDs of active vehicles 
        std::vector<unsigned int> vehicle_ids;
        for(const auto& entry : vehicle_data) 
        {
            vehicle_ids.push_back(entry.first);
        }

        //Get currently online HLCs / NUCs
        auto hlc_data = this->get_hlc_data();

        //Print actual information for each vehicle, using the const string vector rows_restricted to get the desired content
        for(const auto& entry: vehicle_data)
        {
            const auto vehicle_id = entry.first;

            auto vehicle_sensor_timeseries = entry.second;
            for (size_t i = 0; i < rows_restricted.size(); ++i)
            {
                //Ignore rows that serve as separator -> only set empty strings there
                if (rows_restricted[i] == "")
                {
                    //Add empty row, which only serves as a separator for better readability
                    Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(vehicle_id+1, i+1));

                    if(!label)
                    {
                        label = Gtk::manage(new Gtk::Label()); 
                        label->set_width_chars(10);
                        label->set_xalign(1);
                        label->get_style_context()->add_class("small_text");
                        label->show_all();
                        grid_vehicle_monitor->attach(*label, vehicle_id+1, i+1, 1, 1);
                    }
                    continue;
                }

                Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(vehicle_id+1, i+1));

                if(!label)
                {
                    label = Gtk::manage(new Gtk::Label()); 
                    label->set_width_chars(10);
                    label->set_xalign(1);
                    label->show_all();
                    grid_vehicle_monitor->attach(*label, vehicle_id+1, i+1, 1, 1);
                }

                //Special case for nuc connected, which is not in the time series (not part of vehicle data)
                if(rows_restricted[i] == "nuc_connected") 
                {
                    label->get_style_context()->remove_class("ok");
                    label->get_style_context()->remove_class("warn");
                    label->get_style_context()->remove_class("alert");

                    if (std::find(hlc_data.begin(), hlc_data.end(), vehicle_id) != hlc_data.end())
                    {
                        label->set_text("Online");
                        label->get_style_context()->add_class("ok");
                    }
                    else
                    {
                        label->set_text("Offline");
                        label->get_style_context()->add_class("warn");
                    }
                }

                //Ignore rows with non-existing data
                if(!vehicle_sensor_timeseries.count(rows_restricted[i])) continue;

                auto sensor_timeseries = vehicle_sensor_timeseries.at(rows_restricted[i]);

                if(sensor_timeseries->has_new_data(0.5))
                {
                    const auto value = sensor_timeseries->get_latest_value();
                    label->set_text(sensor_timeseries->format_value(value));

                    label->get_style_context()->remove_class("ok");
                    label->get_style_context()->remove_class("warn");
                    label->get_style_context()->remove_class("alert");

                    if(rows_restricted[i] == "clock_delta") 
                    {
                        if     (fabs(value) < 25)  label->get_style_context()->add_class("ok");
                        else if(fabs(value) < 50) label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 
                            
                            cpm::Logging::Instance().write(
                                2,
                                "Warning: Clock delta of vehicle %d too high. Stop and reboot...",
                                vehicle_id
                            );
                            deploy_functions->reboot_real_vehicle(vehicle_id, 5);
                            deploy_functions->stop_vehicles(vehicle_ids);
                        }
                    }
                    else if(rows_restricted[i] == "battery_level") 
                    {
                        int n = 100;
                        std::vector<double> values = sensor_timeseries->get_last_n_values(n);
                        auto max = std::max_element(values.begin(), values.end());

                        if     (fabs(*max) > 30)  label->get_style_context()->add_class("ok");
                        else if(fabs(*max) > 10)  label->get_style_context()->add_class("warn");
                        else
                        {  
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 
                            cpm::Logging::Instance().write(
                                1,
                                "Warning: Battery level of vehicle %d too low. Stopping vehicles ...", 
                                vehicle_id
                            );
                            deploy_functions->stop_vehicles(vehicle_ids);
                        }
                    }
                    else if(rows_restricted[i] == "speed") 
                    {
                        if     (fabs(value) < 3)  label->get_style_context()->add_class("ok");
                        else if(fabs(value) < 3.6)  label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 
                            cpm::Logging::Instance().write(
                                1,
                                "Warning: speed of vehicle %d too high. Stopping vehicles ...", 
                                vehicle_id
                            );
                            deploy_functions->stop_vehicles(vehicle_ids);
                        }
                    }
                    else if(rows_restricted[i] == "ips_dt") 
                    {
                        if      (value < 100) label->get_style_context()->add_class("ok");
                        else if (value < 500) label->get_style_context()->add_class("warn");
                        else                  
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 
                            cpm::Logging::Instance().write(
                                1,
                                "Warning: no IPS signal of vehicle %d. Age: %f ms. Stopping vehicles ...", 
                                vehicle_id, value
                            );
                            deploy_functions->stop_vehicles(vehicle_ids);
                        }
                    }
                    else if(rows_restricted[i] == "reference_deviation") 
                    {
                        // is vehicle on its reference trajectory? else stop 

                        auto pose_x = vehicle_sensor_timeseries.at("pose_x")->get_latest_value();
                        auto pose_y = vehicle_sensor_timeseries.at("pose_y")->get_latest_value();

                        VehicleTrajectories vehicleTrajectories = get_vehicle_trajectory();
                        VehicleTrajectories::iterator trajectory = vehicleTrajectories.find(vehicle_id);

                        // continue if no trajectory available, no reference deviation possible  
                        if(trajectory == vehicleTrajectories.end()) 
                        {
                            label->get_style_context()->add_class("ok");
                            label->set_text("--");
                            continue;
                        }

                        const auto& trajectory_points = trajectory->second;
                        std::vector<TrajectoryPoint> trajectory_segment;
                        for (const auto& trajectory_point : trajectory_points.trajectory_points())
                        {
                            trajectory_segment.push_back(trajectory_point);
                        }        
                        
                        if(trajectory_segment.size() > 2)
                        {
                            
                            //TrajectoryPoint current_trajectory_segment = trajectory_segment[0];
                            uint64_t dt = ULONG_MAX; 
                            double current_px = 0;
                            double current_py = 0;

                            for(size_t i = 2; i < trajectory_segment.size(); ++i)
                            {
                                const int n_interp = 20;
                                for (int interp_step = 1; interp_step < n_interp; ++interp_step)
                                {
                                    const uint64_t delta_t = 
                                        trajectory_segment[i].t().nanoseconds() 
                                        - trajectory_segment[i-1].t().nanoseconds();
                                    
                                    std::shared_ptr<TrajectoryInterpolation> interp = std::make_shared<TrajectoryInterpolation>(
                                        (delta_t * interp_step) / n_interp + trajectory_segment[i-1].t().nanoseconds(),  
                                        trajectory_segment[i-1],  
                                        trajectory_segment[i]
                                    );
                                    
                                    if((delta_t * interp_step) / n_interp + trajectory_segment[i-1].t().nanoseconds()-clock_gettime_nanoseconds() < dt)
                                    {
                                        dt = (delta_t * interp_step) / n_interp + trajectory_segment[i-1].t().nanoseconds()-clock_gettime_nanoseconds(); 
                                        current_px = interp->position_x;
                                        current_py = interp->position_y;
                                    }
                                }
                            }
                            // euclidian distance to reference 
                            double error = sqrt(pow(pose_x-current_px,2)+pow(pose_y-current_py,2));

                            label->set_text(std::to_string(error).substr(0,4));
                            if(error > 0.5) 
                            {
                                label->get_style_context()->add_class("alert");
                                if(!deploy_functions->diagnosis_switch) continue; 
                                cpm::Logging::Instance().write(
                                    1,
                                    "Warning: vehicle %d not on reference. Error: %f m and %" PRIu64 " ms. Stopping vehicles ...", 
                                    vehicle_id, error, dt
                                );
                                deploy_functions->stop_vehicles(vehicle_ids);
                            }
                            else if (error > 0.1)
                            {
                                label->get_style_context()->add_class("warn");
                            }
                            else 
                            {
                                label->get_style_context()->add_class("ok");
                            }
                        }
                        else 
                        {
                            label->get_style_context()->add_class("ok");
                            label->set_text("--");
                            continue;
                        }
                        
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

        //HLC entry update
        //Show amount in entry
        std::stringstream text_stream;
        text_stream << "HLCs online: " << hlc_data.size();
        label_hlc_description_short->set_text(text_stream.str().c_str());

        //Show list in entry tooltip (on mouse hover) - now not tooltip but real list
        std::stringstream list_stream;
        list_stream << "Online IDs: ";
        if(hlc_data.size() > 0)
        {
            for (size_t i = 0; i < hlc_data.size() - 1; ++i)
            {
                list_stream << static_cast<int>(hlc_data.at(i)) << ", ";
            }
            list_stream << static_cast<int>(hlc_data.at(hlc_data.size() - 1));
        }

        label_hlc_description_long->set_text(list_stream.str().c_str());
    });

    run_thread.store(true);
    ui_thread = std::thread(&MonitoringUi::ui_update_loop, this);
}



void MonitoringUi::reset_ui_thread()
{
    //Kill UI thread before clearing data
    stop_ui_thread();
    
    //Clear grid view, create new one
    viewport_monitoring->remove();
    grid_vehicle_monitor = Gtk::manage(new Gtk::Grid());
    grid_vehicle_monitor->set_name("grid_vehicle_monitor");
    viewport_monitoring->add(*grid_vehicle_monitor);
    grid_vehicle_monitor->show();

    //Reset data in underlying data structure
    this->reset_data();

    //Recreate UI thread
    init_ui_thread(); 
}

void MonitoringUi::stop_ui_thread()
{
    run_thread.store(false);

    if(ui_thread.joinable()) {
        ui_thread.join();
    }
}

void MonitoringUi::ui_update_loop()
{
    while (run_thread.load()) {
        update_dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}


Gtk::Box* MonitoringUi::get_parent()
{
    return parent;
}

void MonitoringUi::reset_vehicle_view()
{
    reset_ui_thread();
}