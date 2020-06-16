#include "MonitoringUi.hpp"
#include <numeric>
#include <cassert>


MonitoringUi::MonitoringUi(
    std::shared_ptr<Deploy> deploy_functions_callback, 
    std::function<VehicleData()> get_vehicle_data_callback, 
    std::function<std::vector<std::string>()> get_hlc_data_callback, 
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

    // for threads 
    thread_count.store(0);
}

MonitoringUi::~MonitoringUi()
{
    stop_ui_thread();

    //Join all old threads
    kill_all_threads();
}

void MonitoringUi::kill_all_threads()
{
    //Join all old threads 
    for (auto& thread : reboot_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
    reboot_threads.clear();
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
                    if (rows_restricted[i] != "")
                    {
                        label->set_text(
                            vehicle_data.begin()->second.at(rows_restricted[i])->get_name() + " [" + 
                            vehicle_data.begin()->second.at(rows_restricted[i])->get_unit() + "]"
                        );
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
                if(!vehicle_sensor_timeseries.count(rows_restricted[i])) continue;
                
                Gtk::Label* label = (Gtk::Label*)(grid_vehicle_monitor->get_child_at(vehicle_id+1, i+1));

                if(!label)
                {
                    label = Gtk::manage(new Gtk::Label()); 
                    label->set_width_chars(10);
                    label->set_xalign(1);
                    label->show_all();
                    grid_vehicle_monitor->attach(*label, vehicle_id+1, i+1, 1, 1);
                }

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
                        if     (fabs(value) < 50)  label->get_style_context()->add_class("ok");
                        else if(fabs(value) < 200) label->get_style_context()->add_class("warn");
                        else 
                        {
                            label->get_style_context()->add_class("alert");
                            if(!deploy_functions->diagnosis_switch) continue; 
                            cpm::Logging::Instance().write("Warning: Clock delta of vehicle %d too high. Restarting vehicle %d...", vehicle_id, vehicle_id);
                            
                            std::string reboot;
                            if(vehicle_id<10)
                            {
                                reboot = reboot_script + "0" + std::to_string(vehicle_id);
                            }
                            else
                            {
                                reboot = reboot_script + std::to_string(vehicle_id);
                            }
                            thread_count.fetch_add(1);
                            reboot_threads.push_back(std::thread([this, reboot] () {
                                    std::system(reboot.c_str());
                                    usleep(2000);
                                    this->notify_reboot_finished();
                                }
                            ));
                            deploy_functions->kill_vehicles({},vehicle_ids);
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
                            cpm::Logging::Instance().write("Warning: Battery level of vehicle %d too low. Shutting down ...", vehicle_id);
                            deploy_functions->kill_vehicles({},vehicle_ids);
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
                            cpm::Logging::Instance().write("Warning: speed of vehicle %d too high. Shutting down ...", vehicle_id);
                            deploy_functions->kill_vehicles({},vehicle_ids);
                        }
                    }
                    else if(rows_restricted[i] == "ips") 
                    {
                        label->get_style_context()->add_class("ok");
                        label->set_text("available");
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
                            continue;
                        }

                        const auto& trajectory_points = trajectory->second;
                        std::vector<TrajectoryPoint> trajectory_segment;
                        for (const auto& trajectory_point : trajectory_points.trajectory_points())
                        {
                            trajectory_segment.push_back(trajectory_point);
                        }        

                        if(trajectory_segment.size() > 1)
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
                                cpm::Logging::Instance().write("Warning: vehicle %d not on reference. Error: %f and %l. Shutting down ...", vehicle_id, error, dt);
                                deploy_functions->kill_vehicles({},vehicle_ids);
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
        auto hlc_data = this->get_hlc_data();

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
                list_stream << hlc_data.at(i) << ", ";
            }
            list_stream << hlc_data.at(hlc_data.size() - 1);
        }

        label_hlc_description_long->set_text(list_stream.str().c_str());
    });

    run_thread.store(true);
    ui_thread = std::thread(&MonitoringUi::ui_update_loop, this);
}

void MonitoringUi::notify_reboot_finished()
{
    //Just try to join all worker threads here
    std::lock_guard<std::mutex> lock(notify_callback_in_use);

    //This should never be the case
    //If this happens, the thread count has been initialized incorrectly
    if (thread_count.load() == 0)
    {
        std::cerr << "WARNING: Reboot thread count has not been initialized correctly!" << std::endl;
    }

    //Also count notify amount s.t one can check if the thread count has been set properly
    thread_count.fetch_sub(1);

    std::cout << thread_count.load() << std::endl;
    std::lock_guard<std::mutex> unlock(notify_callback_in_use);
    if (thread_count.load() == 0)
    {

        kill_all_threads();
    }
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