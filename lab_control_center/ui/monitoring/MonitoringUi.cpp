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
    std::function<void()> reset_data_callback,
    std::function<bool(std::string, uint64_t&, uint64_t&, uint64_t&, double&)> get_rtt_values,
    std::function<void()> kill_deployed_applications_callback)
{
    this->deploy_functions = deploy_functions_callback;
    this->get_vehicle_data = get_vehicle_data_callback;
    this->get_hlc_data = get_hlc_data_callback;
    this->get_vehicle_trajectory = get_vehicle_trajectory_command_callback;
    this->reset_data = reset_data_callback;
    this->get_rtt_values = get_rtt_values;
    this->kill_deployed_applications = kill_deployed_applications_callback; 

    builder = Gtk::Builder::create_from_file("ui/monitoring/monitoring_ui.glade");
    builder->get_widget("parent", parent);
    builder->get_widget("viewport_monitoring", viewport_monitoring);
    builder->get_widget("grid_vehicle_monitor", grid_vehicle_monitor);
    builder->get_widget("button_reset_view", button_reset_view);
    builder->get_widget("box_buttons", box_buttons);
    builder->get_widget("label_hlc_description_short", label_hlc_description_short);
    builder->get_widget("label_hlc_description_long", label_hlc_description_long);
    builder->get_widget("label_rtt_hlc_short", label_rtt_hlc_short);
    builder->get_widget("label_rtt_hlc_long", label_rtt_hlc_long);
    builder->get_widget("label_rtt_vehicle_short", label_rtt_vehicle_short);
    builder->get_widget("label_rtt_vehicle_long", label_rtt_vehicle_long);
    builder->get_widget("label_experiment_time", label_experiment_time);

    assert(parent);
    assert(viewport_monitoring);
    assert(grid_vehicle_monitor);
    assert(button_reset_view);
    assert(box_buttons);
    assert(label_hlc_description_short);
    assert(label_hlc_description_long);
    assert(label_rtt_hlc_short);
    assert(label_rtt_hlc_long);
    assert(label_rtt_vehicle_short);
    assert(label_rtt_vehicle_long);
    assert(label_experiment_time);

    //Warning: Most style options are set in Glade (style classes etc) and style.css

    //Initialize the UI thread that updates the view on connected / online vehicles as well as connected / online hlcs
    init_ui_thread();

    //Register the button callback for resetting the vehicle monitoring view (allows to delete old entries)
    button_reset_view->signal_clicked().connect(sigc::mem_fun(this, &MonitoringUi::reset_ui_thread));

    //Store start time of simulation when simulation is running - this is the default value (uninitialized)
    sim_start_time.store(0);
}

MonitoringUi::~MonitoringUi()
{
    stop_ui_thread();
}

void MonitoringUi::register_vehicle_to_hlc_mapping(std::function<std::pair<bool, std::map<uint32_t, uint8_t>>()> _get_vehicle_to_hlc_mapping)
{
    this->get_vehicle_to_hlc_mapping = _get_vehicle_to_hlc_mapping;
}

void MonitoringUi::register_crash_checker(std::shared_ptr<CrashChecker> _crash_checker)
{
    crash_checker = _crash_checker;
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

                    if (!get_vehicle_to_hlc_mapping)
                    {
                        label->set_text("Error in LCC");
                        label->get_style_context()->add_class("alert");
                    }
                    else
                    {
                        auto current_mapping = get_vehicle_to_hlc_mapping();

                        if (current_mapping.first)
                        {
                            //During a simulation, where a mapping exists
                            if (current_mapping.second.find(vehicle_id) == current_mapping.second.end())
                            {
                                //Was not matched
                                label->set_text("Not matched");
                                label->get_style_context()->add_class("warn");
                            }
                            else
                            {
                                auto hlc_id = current_mapping.second.at(vehicle_id);

                                //Find out if the programs are still running on the NUC
                                assert(crash_checker);
                                bool program_crashed = crash_checker->check_if_crashed(hlc_id);

                                bool nuc_crashed = std::find(hlc_data.begin(), hlc_data.end(), hlc_id) == hlc_data.end();

                                if (!nuc_crashed && !program_crashed)
                                {
                                    label->set_text("Online");
                                    label->get_style_context()->add_class("ok");
                                    if(error_timestamps[0][0] != 0) error_timestamps[0][0] = 0;
                                    if(error_triggered[0][0]) error_triggered[0][0] = false; 
                                }
                                else if (nuc_crashed && label->get_text() != "Offline") //Do not log this more than once
                                {
                                    label->set_text("Offline");
                                    label->get_style_context()->add_class("alert");
                                    cpm::Logging::Instance().write(
                                        1,
                                        "Warning: NUC %d disconnected. Stopping vehicles ...", 
                                        hlc_id
                                    );
                                    if(error_timestamps[0][0] == 0) 
                                    {
                                        // set error timestamp  
                                        error_timestamps[0][0] = cpm::get_time_ns(); 
                                        continue;
                                    }

                                    // an error occured before - do nothing if the error is not older than a threshold
                                    if(cpm::get_time_ns()-error_timestamps[0][0]<500000000) continue;
                                    
                                    if(!error_triggered[0][0])
                                    {
                                        cpm::Logging::Instance().write(
                                            1,
                                            "Warning: NUCs %d disconnected. Stopping experiment ...", 
                                            hlc_id
                                            );
                                        this->kill_deployed_applications();
                                        error_triggered[0][0] = true; 
                                    }
                                }
                                else if (program_crashed && label->get_text() != "Offline" && label->get_text() != "Prog. crash")
                                {
                                    label->set_text("Prog. crash");
                                    label->get_style_context()->add_class("alert");
                                    cpm::Logging::Instance().write(
                                        1,
                                        "Warning: NUC %d had a program crash. Stopping vehicles ...", 
                                        hlc_id
                                    );
                                    deploy_functions->stop_vehicles(vehicle_ids);
                                }
                                else
                                {
                                    //But still keep color in Offline case
                                    label->get_style_context()->add_class("alert");
                                }
                                
                            }
                        }
                        else
                        {
                            //No simulation
                            label->set_text("Not matched");
                            label->get_style_context()->add_class("ok");
                        }
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
                        if  (fabs(value) < 25)  
                        {
                            label->get_style_context()->add_class("ok");
                            // reset error timestamp 
                            if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                            if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
                        }
                        else if(fabs(value) < 50) label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 

                            if(error_timestamps[i][vehicle_id] == 0) 
                            {
                                // set error timestamp  
                                error_timestamps[i][vehicle_id] = cpm::get_time_ns(); 
                                continue;
                            }

                            // an error occured before - do nothing if the error is not older than a threshold
                            if(cpm::get_time_ns()-error_timestamps[i][vehicle_id]<500000000) continue;
                            
                            if(!error_triggered[i][vehicle_id])
                            {
                                cpm::Logging::Instance().write(
                                    1,
                                    "Warning: Clock delta of vehicle %d too high. Stopping experiment ...",
                                    vehicle_id
                                );
                                this->kill_deployed_applications();
                                error_triggered[i][vehicle_id] = true;
                            }
                        }
                    }
                    else if(rows_restricted[i] == "battery_level") 
                    {
                        int n = 100;
                        std::vector<double> values = sensor_timeseries->get_last_n_values(n);
                        auto max = std::max_element(values.begin(), values.end());

                        if     (fabs(*max) > 30)  
                        {
                            label->get_style_context()->add_class("ok");
                            // reset error timestamp 
                            if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                            if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
                        }
                        else if(fabs(*max) > 10)  label->get_style_context()->add_class("warn");
                        else
                        {  
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 

                            if(error_timestamps[i][vehicle_id] == 0) 
                            {
                                // set error timestamp  
                                error_timestamps[i][vehicle_id] = cpm::get_time_ns(); 
                                continue;
                            }
                            // an error occured before - do nothing if the error is not older than a threshold
                            if(cpm::get_time_ns()-error_timestamps[i][vehicle_id]<500000000) continue;
                            
                            if(!error_triggered[i][vehicle_id])
                            {
                                cpm::Logging::Instance().write(
                                    1,
                                    "Warning: Battery level of vehicle %d too low. Stopping experiment ...", 
                                    vehicle_id
                                );
                                this->kill_deployed_applications();
                                error_triggered[i][vehicle_id] = true;
                            }
                        }
                    }
                    else if(rows_restricted[i] == "speed") 
                    {
                        if     (fabs(value) < 3)  
                        {
                            label->get_style_context()->add_class("ok");
                            // reset error timestamp 
                            if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                            if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
                        }
                        else if(fabs(value) < 3.6)  label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 

                            if(error_timestamps[i][vehicle_id] == 0) 
                            {
                                // set error timestamp  
                                error_timestamps[i][vehicle_id] = cpm::get_time_ns(); 
                                continue;
                            }
                            // an error occured before - do nothing if the error is not older than a threshold
                            if(cpm::get_time_ns()-error_timestamps[i][vehicle_id]<500000000) continue;

                            if(!error_triggered[i][vehicle_id])
                            {
                                cpm::Logging::Instance().write(
                                    1,
                                    "Warning: speed of vehicle %d too high. Stopping experiment ...", 
                                    vehicle_id
                                );
                                this->kill_deployed_applications();
                                error_triggered[i][vehicle_id] = true;
                            }
                        }
                    }
                    else if(rows_restricted[i] == "ips_dt") 
                    {
                        if      (value < 100) 
                        {
                            label->get_style_context()->add_class("ok");
                            // reset error timestamp 
                            if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                            if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
                        }
                        else if (value < 500) label->get_style_context()->add_class("warn");
                        else                  
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 

                            if(error_timestamps[i][vehicle_id] == 0) 
                            {
                                // set error timestamp  
                                error_timestamps[i][vehicle_id] = cpm::get_time_ns(); 
                                continue;
                            }
                            // an error occured before - do nothing if the error is not older than a threshold
                            if(cpm::get_time_ns()-error_timestamps[i][vehicle_id]<2000000000) continue;

                            if(!error_triggered[i][vehicle_id])
                            {
                                cpm::Logging::Instance().write(
                                    1,
                                    "Warning: no IPS signal of vehicle %d. Age: %f ms. Stopping experiment ...", 
                                    vehicle_id, value
                                );
                                this->kill_deployed_applications();
                                error_triggered[i][vehicle_id] = true;
                            }
                        }
                    }
                    else if(rows_restricted[i] == "last_msg_state")
                    {
                        //Calculate diff
                        double t_now_ms = static_cast<double>(cpm::get_time_ns() * 1e-6);
                        double t_diff = t_now_ms - value;
                        std::stringstream text;
                        text << ceil(t_diff * 100) / 100; //Round to 2 values after comma
                        label->set_text(text.str().c_str());

                        //20 would be ideal (50Hz for vehicle data)
                        if     (fabs(t_diff) < 20)  label->get_style_context()->add_class("ok");
                        else if(fabs(t_diff) < 30) label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
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
                            // reset error timestamp 
                            if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                            if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
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

                            for(size_t j = 2; j < trajectory_segment.size(); ++j)
                            {
                                const int n_interp = 20;
                                for (int interp_step = 1; interp_step < n_interp; ++interp_step)
                                {
                                    const uint64_t delta_t = 
                                        trajectory_segment[j].t().nanoseconds() 
                                        - trajectory_segment[j-1].t().nanoseconds();
                                    
                                    std::shared_ptr<TrajectoryInterpolation> interp = std::make_shared<TrajectoryInterpolation>(
                                        (delta_t * interp_step) / n_interp + trajectory_segment[j-1].t().nanoseconds(),  
                                        trajectory_segment[j-1],  
                                        trajectory_segment[j]
                                    );
                                    
                                    if((delta_t * interp_step) / n_interp + trajectory_segment[j-1].t().nanoseconds()-cpm::get_time_ns() < dt)
                                    {
                                        dt = (delta_t * interp_step) / n_interp + trajectory_segment[j-1].t().nanoseconds()-cpm::get_time_ns(); 
                                        current_px = interp->position_x;
                                        current_py = interp->position_y;
                                    }
                                }
                            }
                            // euclidian distance to reference 
                            double error = sqrt(pow(pose_x-current_px,2)+pow(pose_y-current_py,2));

                            label->set_text(std::to_string(error).substr(0,4));
                            if(error > 0.15) 
                            {
                                label->get_style_context()->add_class("alert");
                                if(!deploy_functions->diagnosis_switch) continue;

                                if(error_timestamps[i][vehicle_id] == 0) 
                                {
                                    // set error timestamp  
                                    error_timestamps[i][vehicle_id] = cpm::get_time_ns(); 
                                    continue;
                                }
                                // an error occured before - do nothing if the error is not older than a threshold
                                if(cpm::get_time_ns()-error_timestamps[i][vehicle_id]<200000000) continue;

                                if(!error_triggered[i][vehicle_id])
                                {
                                    cpm::Logging::Instance().write(
                                        1,
                                        "Warning: vehicle %d not on reference. Error: %f m and %f ms. Stopping experiment ...", 
                                        vehicle_id, error, dt/1e6
                                    );
                                    this->kill_deployed_applications();
                                    error_triggered[i][vehicle_id] = true;
                                }
                            }
                            else if (error > 0.05)
                            {
                                label->get_style_context()->add_class("warn");
                            }
                            else 
                            {
                                label->get_style_context()->add_class("ok");
                                // reset error timestamp 
                                if(error_timestamps[i][vehicle_id] != 0) error_timestamps[i][vehicle_id] = 0; 
                                if(error_triggered[i][vehicle_id]) error_triggered[i][vehicle_id] = false;
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

        //RTT update - HLC
        uint64_t hlc_current_best_rtt, hlc_current_worst_rtt, hlc_all_time_worst_rtt = 0;
        double hlc_missed_rtt_percentage = 0.0;
        bool hlc_rtt_exists = get_rtt_values("hlc", hlc_current_best_rtt, hlc_current_worst_rtt, hlc_all_time_worst_rtt, hlc_missed_rtt_percentage);

        if (!hlc_rtt_exists)
        {
            label_rtt_hlc_short->set_text("HLC RTT (ms): ---");
            label_rtt_hlc_long->set_text("---");
        }
        else
        {
            //Possible TODO: Change background color depending on RTT 'quality' / allow different coloring for any of the three entries
            std::stringstream rtt_long;
            rtt_long 
            << "\tCurrent best / worst: " << static_cast<uint64_t>(hlc_current_best_rtt / 1e6) << " / "
            << static_cast<uint64_t>(hlc_current_worst_rtt / 1e6) << "\n"
            << "\tAll-time worst: " << static_cast<uint64_t>(hlc_all_time_worst_rtt / 1e6) << "\n"
            << "\tMissed (percent): " << static_cast<uint64_t>(hlc_missed_rtt_percentage * 100);
            label_rtt_hlc_long->set_text(rtt_long.str().c_str());

            std::stringstream rtt_short;
            rtt_short << "HLC RTT (ms): " << static_cast<uint64_t>(hlc_current_worst_rtt / 1e6);
            label_rtt_hlc_short->set_text(rtt_short.str().c_str());
        }

        //RTT update - vehicle
        uint64_t vehicle_current_best_rtt, vehicle_current_worst_rtt, vehicle_all_time_worst_rtt = 0;
        double vehicle_missed_rtt_percentage = 0.0;
        bool vehicle_rtt_exists = get_rtt_values("vehicle", vehicle_current_best_rtt, vehicle_current_worst_rtt, vehicle_all_time_worst_rtt, vehicle_missed_rtt_percentage);
        if (!vehicle_rtt_exists)
        {
            label_rtt_vehicle_short->set_text("Vehicle RTT (ms): ---");
            label_rtt_vehicle_long->set_text("---");
        }
        else
        {
            //Possible TODO: Change background color depending on RTT 'quality' / allow different coloring for any of the three entries
            std::stringstream rtt_long;
            rtt_long 
            << "\tCurrent best / worst: " << static_cast<uint64_t>(vehicle_current_best_rtt / 1e6) << " / "
            << static_cast<uint64_t>(vehicle_current_worst_rtt / 1e6) << "\n"
            << "\tAll-time worst: " << static_cast<uint64_t>(vehicle_all_time_worst_rtt / 1e6) << "\n"
            << "\tMissed (percent): " << static_cast<uint64_t>(vehicle_missed_rtt_percentage * 100);
            label_rtt_vehicle_long->set_text(rtt_long.str().c_str());

            std::stringstream rtt_short;
            rtt_short << "Vehicle RTT (ms): " << static_cast<uint64_t>(vehicle_current_worst_rtt / 1e6);
            label_rtt_vehicle_short->set_text(rtt_short.str().c_str());
        }

        //Update running time of simulation, if it is currently running
        auto sim_start = sim_start_time.load();
        if (sim_start > 0)
        {
            auto t_diff = cpm::get_time_ns() - sim_start;
            t_diff /= 1e9; //Convert to seconds

            //Now calculate hours, minutes and seconds
            auto t_sec = t_diff % 60;
            auto t_min = t_diff / 60;
            auto t_hr = t_min / 60;
            t_min = t_min % 60;

            std::stringstream sim_time_stream;
            sim_time_stream << "Exp time: ";

            if (t_hr > 0)
            {
                sim_time_stream << t_hr << "h ";
            }
            if (t_min > 0 || t_hr > 0)
            {
                sim_time_stream << t_min << "min ";
            }
            sim_time_stream << t_sec << "s";

            label_experiment_time->set_text(sim_time_stream.str().c_str());
        }
        else
        {
            label_experiment_time->set_text("Exp time: ---");
        }
        
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

void MonitoringUi::notify_sim_start()
{
    reset_vehicle_view();

    //Timer start (to determine how long the simulation has been running)
    sim_start_time.store(cpm::get_time_ns());
}

void MonitoringUi::notify_sim_stop()
{
    reset_vehicle_view();

    //Timer reset
    sim_start_time.store(0);
}