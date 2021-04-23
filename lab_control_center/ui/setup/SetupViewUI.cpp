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

#include "SetupViewUI.hpp"
#include <cstdlib>
#include <chrono>

/**
 * \file SetupViewUI.cpp
 * \ingroup lcc_ui
 */

using namespace std::placeholders;

SetupViewUI::SetupViewUI
    (
    std::shared_ptr<Deploy> _deploy_functions, 
    std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
    std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator, 
    std::function<VehicleData()> _get_vehicle_data,
    std::function<void(bool, bool)> _reset_timer,
    std::function<void()> _reset_vehicle_view,
    std::function<void()> _on_simulation_start,
    std::function<void()> _on_simulation_stop,
    std::function<void(bool)> _set_commonroad_tab_sensitive,
    std::function<void(std::vector<int32_t>)> _update_vehicle_ids_parameter,
    std::string _absolute_exec_path,
    unsigned int argc, 
    char *argv[]
    ) 
    :
    deploy_functions(_deploy_functions),
    vehicle_control(_vehicle_control),
    hlc_ready_aggregator(_hlc_ready_aggregator),
    get_vehicle_data(_get_vehicle_data),
    reset_timer(_reset_timer),
    reset_vehicle_view(_reset_vehicle_view),
    on_simulation_start(_on_simulation_start),
    on_simulation_stop(_on_simulation_stop),
    set_commonroad_tab_sensitive(_set_commonroad_tab_sensitive),
    update_vehicle_ids_parameter(_update_vehicle_ids_parameter)
{
    builder = Gtk::Builder::create_from_file("ui/setup/setup.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("setup_box", setup_box);
    builder->get_widget("script_path", script_path);
    builder->get_widget("script_params", script_params);
    builder->get_widget("button_choose_script", button_choose_script);
    
    builder->get_widget("button_select_none", button_select_none);
    builder->get_widget("button_select_all_simulated", button_select_all_simulated);

    builder->get_widget("switch_simulated_time", switch_simulated_time);

    builder->get_widget("switch_deploy_remote", switch_deploy_remote);

    builder->get_widget("switch_lab_mode", switch_lab_mode);
    builder->get_widget("switch_record_labcam", switch_record_labcam);
    builder->get_widget("switch_diagnosis", switch_diagnosis);

    builder->get_widget("button_deploy", button_deploy);
    builder->get_widget("button_kill", button_kill);

    builder->get_widget("vehicle_flowbox", vehicle_flowbox);

    assert(parent);
    assert(setup_box);
    assert(script_path);
    assert(script_params);
    assert(button_choose_script);

    assert(button_select_none);
    assert(button_select_all_simulated);
    
    assert(switch_simulated_time);
    assert(switch_deploy_remote);
    assert(switch_lab_mode);
    assert(switch_record_labcam);
    assert(switch_diagnosis);

    assert(button_deploy);
    assert(button_kill);

    assert(vehicle_flowbox);

    //Get number of vehicles
    unsigned int number_of_vehicles = cpm::cmd_parameter_int("number_of_vehicles", 20, argc, argv);

    //Create vehicle toggles
    for (unsigned int id = 1; id <= number_of_vehicles; ++id)
    {
        vehicle_toggles.emplace_back(std::make_shared<VehicleToggle>(id));
    }

    //Add vehicle toggles to vehicle flowbox
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_flowbox->add(*(vehicle_toggle->get_parent()));
        vehicle_toggle->set_selection_callback(std::bind(&SetupViewUI::vehicle_toggle_callback, this, _1, _2));
    }
#ifndef SIMULATION
    // Create labcam
    labcam = new LabCamIface();
#endif

    //Register button callbacks
    button_deploy->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::deploy_applications));
    button_kill->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::kill_deployed_applications));
    button_choose_script->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::open_file_explorer));
    button_select_all_simulated->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles_sim));
    button_select_none->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_no_vehicles));

    kill_grey_out_running.store(false);
    undo_kill_grey_out.store(false);

    //Extract other relevant parameters from command line
    cmd_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    //Set switch to current simulated time value - due to current design sim. time cannot be changed after the LCC has been started
    switch_simulated_time->set_active(cmd_simulated_time);
    switch_simulated_time->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_timer_set));

    //The IPS can be startet and restarted manually independent of the other components
    builder->get_widget("switch_lab_mode", switch_lab_mode);
    switch_lab_mode->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_ips_set));

    //The Diagnosis can be startet and restarted manually independent of the other components
    builder->get_widget("switch_diagnosis", switch_diagnosis);
    switch_diagnosis->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_diagnosis_set));

    //Take care of GUI thread and worker thread separately
    update_vehicle_toggles.store(false);
    ui_dispatcher.connect(sigc::mem_fun(*this, &SetupViewUI::ui_dispatch));

    //Create upload manager
    upload_manager = std::make_shared<Upload>(
        [this] () { return hlc_ready_aggregator->get_hlc_ids_uint8_t(); },
        deploy_functions,
        [this] () { set_sensitive(true); },
        [this] () { button_kill->set_sensitive(true); },
        [this] () { perform_post_kill_cleanup(); }
    );

    //Create crash checker (but don't start it yet, as no simulation is running)
    crash_checker = std::make_shared<CrashChecker>(
        deploy_functions,
        hlc_ready_aggregator,
        upload_manager
    );
    both_local_and_remote_deploy.store(false);

    //Set initial text of script path (from previous program execution, if that existed)
    //We use the default config location here
    auto last_exec_path = FileChooserUI::get_last_execution_path("script");
    if (last_exec_path.size() == 0)
    {
        //In case of an empty path, set the HLC folder location as default path
        //Construct the path to the folder by erasing all parts to the executable that are obsolete
        //Executable path: .../software/lab_control_center/build/lab_control_center
        //-> Remove everything up to the third-last slash
        last_exec_path = _absolute_exec_path;
        for (int i = 0; i < 3; ++i)
        {
            auto last_slash = last_exec_path.find_last_of('/');
            if (last_slash != std::string::npos)
            {
                last_exec_path = last_exec_path.substr(0, last_slash);
            }
        }

        last_exec_path.append("/high_level_controller/examples/");
    }
    script_path->set_text(last_exec_path);

    simulation_running.store(false);
    
    //Regularly check / update which real vehicles are currently turned on, to use them when the experiment is deployed
    is_deployed.store(false);
    vehicle_data_thread_running.store(true);
    check_real_vehicle_data_thread = std::thread([&]{
        // So we can later check if anything changed
        std::vector<unsigned int> old_active_vehicles;
        std::vector<unsigned int> active_vehicles;
        while(vehicle_data_thread_running.load())
        {
            active_vehicles = get_vehicle_ids_active();

            //Don't update data during experiment
            if (! is_deployed.load())
            {
            
                //Check if vehicle data has changed, flag all vehicles that are active and not simulated as real vehicles
                auto currently_simulated_vehicles = get_vehicle_ids_simulated();

                //Get currently active vehicles
                active_real_vehicles.clear();
                for (auto vehicle_entry : get_vehicle_data())
                {
                    auto id = vehicle_entry.first;

                    //Only consider data of non-simulated vehicles that is not older than 500ms (else: probably turned off)
                    //See if a signal of a real vehicle is among the past 25 signals (due to 50Hz -> 500ms); real is more important than simulated
                    bool is_real_vehicle = false;
                    for (bool const is_real : vehicle_entry.second.at("is_real")->get_last_n_values(25))
                    {
                        if (is_real) {
                            is_real_vehicle = true;
                            break;
                        }
                    }

                    if (vehicle_entry.second.at("pose_x")->has_new_data(0.5) && is_real_vehicle)
                    {
                        active_real_vehicles.push_back(id);

                        //Kill simulated vehicle if real vehicle was detected
                        if (std::find(currently_simulated_vehicles.begin(), currently_simulated_vehicles.end(), id) != currently_simulated_vehicles.end())
                        {
                            cpm::Logging::Instance().write(3, "Killing simulated vehicle %i, replaced by real vehicle", static_cast<int>(id));
                            deploy_functions->kill_sim_vehicle(id);
                        }
                    }
                }

                // If the active vehicle ids changed, update the parameter server
                if (active_vehicles != old_active_vehicles) {
                    // Transform to signed int for parameter server
                    // We do not check if this is safe, because ids are small
                    std::vector<int32_t> active_vehicles_signed(
                            active_vehicles.size());
                    std::transform(
                            active_vehicles.begin(),
                            active_vehicles.end(), 
                            active_vehicles_signed.begin(), [](unsigned int id) { return (int32_t)id;});
                    // Update the vehicle ids on the ParameterServer
                    update_vehicle_ids_parameter(active_vehicles_signed);
                }

                //The vehicle toggles must be updated in the UI thread
                update_vehicle_toggles.store(true);
                ui_dispatcher.emit();
            }

            old_active_vehicles = active_vehicles;
            
            //Sleep for a while, then update again
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    lcc_closed.store(false);
}

SetupViewUI::~SetupViewUI() {
    //Kill real vehicle data thread
    vehicle_data_thread_running.store(false);
    if(check_real_vehicle_data_thread.joinable())
    {
        check_real_vehicle_data_thread.join();
    }

    //Kill grey out thread for kill button, if it exists
    kill_grey_out_running.store(false);
    if (kill_grey_out_thread.joinable())
    {
        kill_grey_out_thread.join();
    }
}

void SetupViewUI::vehicle_toggle_callback(unsigned int vehicle_id, VehicleToggle::ToggleState state)
{
    if (state == VehicleToggle::ToggleState::Simulated)
    {
        deploy_functions->deploy_sim_vehicle(vehicle_id, switch_simulated_time->get_active());
    }
    else if (state == VehicleToggle::ToggleState::Off)
    {
        deploy_functions->kill_sim_vehicle(vehicle_id);
        reset_vehicle_view(); //Remove sim. vehicle entry in map view
    }
    else
    {
        deploy_functions->reboot_real_vehicle(vehicle_id, reboot_timeout);
        vehicle_toggles.at(vehicle_id - 1)->set_insensitive(reboot_timeout + 3); //Add some seconds because the whole reboot process might take a bit longer than the timeout
    }   
}


//Do the same as in the destructor, because there we do not get the desired results sadly
void SetupViewUI::on_lcc_close() {
    lcc_closed.store(true);
    kill_deployed_applications();
    deploy_functions->kill_ips();

    //Kill real vehicle data thread
    vehicle_data_thread_running.store(false);
    if(check_real_vehicle_data_thread.joinable())
    {
        check_real_vehicle_data_thread.join();
    }

    //Kill grey out thread for kill button, if it exists
    kill_grey_out_running.store(false);
    if (kill_grey_out_thread.joinable())
    {
        kill_grey_out_thread.join();
    }

    //Kill simulated vehicles
    deploy_functions->kill_sim_vehicles(get_vehicle_ids_simulated());
}

void SetupViewUI::switch_timer_set()
{
    reset_timer(switch_simulated_time->get_active(), true);
    //Restart simulated vehicles with new timer
    deploy_functions->kill_sim_vehicles(get_vehicle_ids_simulated());
    deploy_functions->deploy_sim_vehicles(get_vehicle_ids_simulated(), switch_simulated_time->get_active());

}

void SetupViewUI::switch_ips_set()
{
    if(switch_lab_mode->get_active())
    {
        std::cout << "STARTING IPS" << std::endl;
        deploy_functions->deploy_ips();
    }
    else
    {
        std::cout << "STOPPING IPS" << std::endl;
        deploy_functions->kill_ips();
    }
}

void SetupViewUI::switch_diagnosis_set()
{
    if(switch_diagnosis->get_active())
    {
        std::cout << "STARTING DIAGNOSIS" << std::endl;
        deploy_functions->diagnosis_switch = true;
    }
    else
    {
        std::cout << "STOPPING DIAGNOSIS" << std::endl;
        deploy_functions->diagnosis_switch = false;
    }
}

using namespace std::placeholders;
void SetupViewUI::open_file_explorer()
{
    //We do not want the user to interact with the UI while they are choosing a new scenario
    set_sensitive(false);

    //Filter to show only executables / .m files
    FileChooserUI::Filter application_filter;
    application_filter.name = "Application/Matlab";
    application_filter.mime_filter_types = std::vector<std::string> {"application/x-sharedlib", "text/x-matlab"};

    //Filter to show everything
    FileChooserUI::Filter all_filter;
    all_filter.name = "All";
    all_filter.pattern_filter_types = std::vector<std::string> {"*"};

    //Only create the window if we can get the main window
    if (get_main_window)
    {
        file_chooser_window = make_shared<FileChooserUI>(
            get_main_window(), 
            std::bind(&SetupViewUI::file_explorer_callback, this, _1, _2), 
            std::vector<FileChooserUI::Filter> { application_filter, all_filter },
            "script",
            std::string(script_path->get_text())
        );
    }
    else
    {
        std::cerr << "ERROR: Main window reference is missing, cannot create file chooser dialog";
        LCCErrorLogger::Instance().log_error("ERROR: Main window reference is missing, cannot create file chooser dialog");
    }
    
}

void SetupViewUI::file_explorer_callback(std::string file_string, bool has_file)
{
    if (has_file)
    {
        script_path->set_text(file_string.c_str());
    }

    //The user is now allowed to interact with the UI again
    set_sensitive(true);
}

void SetupViewUI::ui_dispatch()
{
    //Update vehicle toggles for real vehicles
    if (update_vehicle_toggles.exchange(false))
    {
        //Update vehicle toggles
        std::lock_guard<std::mutex> lock(active_real_vehicles_mutex);

        for (std::shared_ptr<VehicleToggle> toggle : vehicle_toggles)
        {
            unsigned int id = toggle->get_id();
            std::vector<unsigned int> active_sim_vehicles = get_vehicle_ids_simulated();
            bool is_sim = (std::find(active_sim_vehicles.begin(), active_sim_vehicles.end(), id) != active_sim_vehicles.end());
            bool is_real = (std::find(active_real_vehicles.begin(), active_real_vehicles.end(), id) != active_real_vehicles.end());
            
            if (is_real){
                //If we find it both in is_sim and is_real, the real vehicle is more important
                toggle->set_state(VehicleToggle::Real);
            } 
            else if (is_sim){
                toggle->set_state(VehicleToggle::Simulated);
            }
            else {
                toggle->set_state(VehicleToggle::Off);
            }
        }
    }

    //Kill has a timeout s.t. a kill button can not be "spammed"
    //But: grey-out should not be undone during remote simulation, because Deploy then already has control over when Kill should become sensitive again
    if (undo_kill_grey_out.exchange(false) && !(simulation_running.load() && switch_deploy_remote->get_active()))
    {
        button_kill->set_sensitive(true);
    }
}

void SetupViewUI::deploy_applications() {
    //Only allow the simulation to start if there are actually vehicles to control
    std::vector<unsigned int> vehicle_ids = get_vehicle_ids_active();
    auto simulation_possible = vehicle_ids.size() > 0;

    if (!simulation_possible)
    {
        cpm::Logging::Instance().write(1, "%s", "LCC Deploy: No vehicles are online, deploy was aborted");
        return;
    }

    //Grey out UI until kill is clicked
    set_sensitive(false);
    is_deployed.store(true);

    simulation_running.store(true);

    //Delete old script / HLC / recording... log entries in the log folder lcc_script_logs
    deploy_functions->delete_old_logs();

    //Reset old UI elements etc (call all functions that registered for this callback in main)
    reset_timer(switch_simulated_time->get_active(), false); //We do not need to send a stop signal here (might be falsely received by newly started participants)
    if(on_simulation_start)
    {
        on_simulation_start();
    }
    else
    {
        cpm::Logging::Instance().write(1, "%s", "Error in SetupViewUI: on_simulation_start callback missing!");
    }
    

    //Remember these also for crash check thread
    bool deploy_remote_toggled = switch_deploy_remote->get_active();
    bool lab_mode_on = switch_lab_mode->get_active();
    bool labcam_toggled = switch_record_labcam->get_active();

    // LabCam
#ifndef SIMULATION
    if(labcam_toggled){
        std::cout << "RECORDING LABCAM" << std::endl;
        // Use current time as file name of the recording
        auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::string file_name = ctime(&timenow);

        // Replace unwanted characters
        std::replace(file_name.begin(), file_name.end(), ' ', '_');
        std::replace(file_name.begin(), file_name.end(), '\n', '_');

        deploy_functions->deploy_labcam("/tmp/", file_name);
    }else{
        std::cout << "NOT RECORDING LABCAM" << std::endl;
    }

#endif
    
    // Recording
    deploy_functions->deploy_recording();

    //Make sure that the filepath exists. If it does not, warn the user about it, but proceed with deployment 
    //Reason: Some features might need to be used / tested where deploying anything but the script / middleware is sufficient
    bool file_exists = false;
    //First make sure that there is anything but spaces in the string
    std::string filepath_str = script_path->get_text().c_str();
    if (filepath_str.find_first_not_of(' ') != std::string::npos)
    {
        //Now also check if the path actually exists
        std::experimental::filesystem::path filepath = filepath_str;

        if (std::experimental::filesystem::exists(filepath))
        {
            //Update path to absolute path, s.t. deploy remote does not have any problems
            try
            {
                filepath_str = std::experimental::filesystem::absolute(filepath);
                //We do not want a directory
                file_exists = ! (std::experimental::filesystem::is_directory(filepath));
            }
            catch(const std::experimental::filesystem::filesystem_error& e)
            {
                std::stringstream error_stream;
                error_stream << "Could not convert given script path to absolute path, error is: " << e.what();
                cpm::Logging::Instance().write(1, "%s", error_stream.str().c_str());
            }
        }
    }

    //Also check if an empty string was passed - in this case, we only want to start the middleware
    //We only do this in case of local deployment (e.g. for debug purposes of locally running programs) - 
    //  for remote deployment, we require a valid script to be set
    bool start_middleware_without_hlc = (filepath_str.size() == 0);

    std::vector<uint8_t> remote_hlc_ids; //Remember IDs of all HLCs where software actually is deployed
    //Remote deployment of scripts on HLCs or local deployment depending on switch state
    if(deploy_remote_toggled && file_exists)
    {
        //Deploy on each HLC
        button_kill->set_sensitive(false);

        //Get current online vehicle and high_level_controller IDs
        std::vector<uint8_t> hlc_ids;
        if (hlc_ready_aggregator)
        {
            hlc_ids = hlc_ready_aggregator->get_hlc_ids_uint8_t();
        }
        else 
        {
            cpm::Logging::Instance().write(1, "%s", "No lookup function to get HLC IDs given, cannot deploy on HLCs");
            return;
        }

        //Match lowest vehicle ID to lowest HLC ID
        std::sort(vehicle_ids.begin(), vehicle_ids.end());
        std::sort(hlc_ids.begin(), hlc_ids.end());
        size_t min_hlc_vehicle = std::min(hlc_ids.size(), vehicle_ids.size());

        remote_hlc_ids = hlc_ids;
        remote_hlc_ids.erase(remote_hlc_ids.begin() + min_hlc_vehicle, remote_hlc_ids.end());

        //Deploy remote
        auto simulated_time = switch_simulated_time->get_active();
        std::string params = script_params->get_text().c_str();

        upload_manager->deploy_remote(simulated_time, filepath_str, params, hlc_ids, vehicle_ids);

        //Now those vehicles that could not be matched are treated as in the local case
        if (vehicle_ids.size() > hlc_ids.size())
        {
            std::vector<uint32_t> local_vehicles;
            local_vehicles.reserve(vehicle_ids.size() - hlc_ids.size());

            for (size_t i = hlc_ids.size(); i < vehicle_ids.size(); ++i)
            {
                local_vehicles.push_back(vehicle_ids.at(i));
            }

            both_local_and_remote_deploy.store(true);
            deploy_functions->deploy_separate_local_hlcs(switch_simulated_time->get_active(), local_vehicles, filepath_str, script_params->get_text().c_str());
        }
        //Remember vehicle to HLC mapping
        std::lock_guard<std::mutex> lock_map(vehicle_to_hlc_mutex);
        for (size_t i = 0; i < min_hlc_vehicle; ++i)
        {
            vehicle_to_hlc_map[vehicle_ids.at(i)] = hlc_ids.at(i);
        }
    }
    else if (file_exists || start_middleware_without_hlc)
    {
        deploy_functions->deploy_local_hlc(switch_simulated_time->get_active(), get_vehicle_ids_active(), filepath_str, script_params->get_text().c_str());
    }
    else
    {
        cpm::Logging::Instance().write(1, "%s", "Script path is empty / invalid / a directory, thus neither script nor middleware could be started");
    }
    

    //Start performing crash checks for deployed applications
    crash_checker->start_checking(file_exists, start_middleware_without_hlc, remote_hlc_ids, both_local_and_remote_deploy.load(), deploy_remote_toggled, lab_mode_on, labcam_toggled);
}

std::pair<bool, std::map<uint32_t, uint8_t>> SetupViewUI::get_vehicle_to_hlc_matching()
{
    std::lock_guard<std::mutex> lock_map(vehicle_to_hlc_mutex);
    return { simulation_running.load(), vehicle_to_hlc_map };
}

void SetupViewUI::kill_deployed_applications() {
    //Make sure that this button cannot be "spammed"
    std::unique_lock<std::mutex> lock(kill_button_mutex);
    button_kill->set_sensitive(false);
    kill_grey_out_running.store(true);
    undo_kill_grey_out.store(false);
    if (kill_grey_out_thread.joinable())
    {
        kill_grey_out_thread.join();
    }
    kill_grey_out_thread = std::thread(
        [&] ()
        {
            int count = 0;
            //Wait for 3 seconds, be interruptible on class destruction
            while(count < 30 && kill_grey_out_running.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                ++count;
            }

            //Undo button grey-out
            undo_kill_grey_out.store(true);
            ui_dispatcher.emit();
        }
    );
    lock.unlock();
    
    //Kill button functionality if no simulation was performed before
    if (! simulation_running.load())
    {
        //Do only a part of the logic below
        deploy_functions->stop_vehicles(get_vehicle_ids_active());
        perform_post_kill_cleanup(); //Even this part is not really required

        return;
    }

    //Kill crash check first, or else we get undesired error messages
    crash_checker->stop_checking();
    //Remember mapping
    std::unique_lock<std::mutex> lock_map(vehicle_to_hlc_mutex);
    vehicle_to_hlc_map.clear();
    simulation_running.store(false);
    lock_map.unlock();


    // Stop LabCam
#ifndef SIMULATION
    deploy_functions->kill_labcam();
    //labcam->stopRecording();
#endif

    is_deployed.store(false);

    //Kill scripts locally or remotely - do not perform remote kill (with new UI window creations etc) if the whole lcc is in the process of being destructed
    if(switch_deploy_remote->get_active() && !lcc_closed.load())
    {
        //Performs post_kill_cleanup after remote kill
        upload_manager->kill_remote();

        //Also kill potential local HLC
        if (both_local_and_remote_deploy.exchange(false))
        {
            deploy_functions->kill_separate_local_hlcs();
        }
    }
    else 
    {
        deploy_functions->kill_local_hlc();
        perform_post_kill_cleanup();
    }

    deploy_functions->stop_vehicles(get_vehicle_ids_active());
    
    // Recording
    deploy_functions->kill_recording();

    //The rest is done in perform_post_kill_cleanup when the UI window closed (when all threads are killed) 
    //But only if threads are used, so only in case of remote deployment
    //For local deployment, perform_post_kill_cleanup is called directly

}

void SetupViewUI::perform_post_kill_cleanup()
{
    //Reset old UI elements etc
    reset_timer(switch_simulated_time->get_active(), true);

    //TODO: USE ASSERT INSTEAD?
    //(call all functions that registered for this callback in main) -> callback becomes unnecessary if cleanup was called due to LCC close
    if(on_simulation_stop && !lcc_closed.load())
    {
        on_simulation_stop();
    }
    else if (!on_simulation_stop)
    {
        cpm::Logging::Instance().write(1, "%s", "Error in SetupViewUI: on_simulation_stop callback missing!");
    }

    //Undo grey out
    set_sensitive(true);
}

std::vector<unsigned int> SetupViewUI::get_vehicle_ids_active() {
    //Vector to store sim. and real vehicles
    std::vector<unsigned int> active_vehicle_ids;

    auto simulated_vehicle_ids = get_vehicle_ids_simulated();

    //Add real vehicle IDs
    std::unique_lock<std::mutex> lock(active_real_vehicles_mutex);
    //Reserve for better efficiency before inserting
    active_vehicle_ids.reserve(simulated_vehicle_ids.size() + active_real_vehicles.size());
    active_vehicle_ids.insert(active_vehicle_ids.end(), active_real_vehicles.begin(), active_real_vehicles.end());
    lock.unlock();

    //Add simulated vehicle IDs
    active_vehicle_ids.insert(active_vehicle_ids.end(), simulated_vehicle_ids.begin(), simulated_vehicle_ids.end());

    return active_vehicle_ids;
}

//Works because a copy is created
std::vector<unsigned int> SetupViewUI::get_vehicle_ids_real() {
    std::lock_guard<std::mutex> lock(active_real_vehicles_mutex);
    return active_real_vehicles;
}

bool SetupViewUI::is_active_real(unsigned int vehicle_id)
{
    std::lock_guard<std::mutex> lock(active_real_vehicles_mutex);
    return std::find(active_real_vehicles.begin(), active_real_vehicles.end(), vehicle_id) != active_real_vehicles.end();
}

std::vector<unsigned int> SetupViewUI::get_vehicle_ids_simulated() {
    std::vector<unsigned int> active_vehicle_ids;

    for (auto& vehicle_toggle : vehicle_toggles)
    {
        if (vehicle_toggle->get_state() == VehicleToggle::Simulated)
        {
            active_vehicle_ids.push_back(vehicle_toggle->get_id());
        }
    }

    return active_vehicle_ids;
}

void SetupViewUI::set_sensitive(bool is_sensitive) {
    script_path->set_sensitive(is_sensitive);
    script_params->set_sensitive(is_sensitive);

    button_choose_script->set_sensitive(is_sensitive);
    
    button_select_none->set_sensitive(is_sensitive);
    button_select_all_simulated->set_sensitive(is_sensitive);

    button_deploy->set_sensitive(is_sensitive);

    switch_record_labcam->set_sensitive(is_sensitive);
    switch_lab_mode->set_sensitive(is_sensitive);
    switch_diagnosis->set_sensitive(is_sensitive);
    switch_deploy_remote->set_sensitive(is_sensitive);
    switch_simulated_time->set_sensitive(is_sensitive);

    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_sensitive(is_sensitive);
    }

    assert(set_commonroad_tab_sensitive);
    set_commonroad_tab_sensitive(is_sensitive);
}

void SetupViewUI::select_all_vehicles_sim()
{
    auto real_vehicles = get_vehicle_ids_real();
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        auto real_ptr = std::find(real_vehicles.begin(), real_vehicles.end(), vehicle_toggle->get_id());
        if (real_ptr == real_vehicles.end())
        {
            vehicle_toggle->set_state(VehicleToggle::ToggleState::Simulated);
            deploy_functions->deploy_sim_vehicle(vehicle_toggle->get_id(), switch_simulated_time->get_active());
        }
    }
}

void SetupViewUI::select_no_vehicles()
{
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        if (vehicle_toggle->get_state() == VehicleToggle::ToggleState::Simulated)
        {
            vehicle_toggle->set_state(VehicleToggle::ToggleState::Off);
            deploy_functions->kill_sim_vehicle(vehicle_toggle->get_id());
        }
    }

    reset_vehicle_view(); //Remove sim. vehicle entry in map view
}

void SetupViewUI::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
    upload_manager->set_main_window_callback(_get_main_window);
    crash_checker->set_main_window_callback(_get_main_window);
}

Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}

std::shared_ptr<CrashChecker> SetupViewUI::get_crash_checker()
{
    return crash_checker;
}
