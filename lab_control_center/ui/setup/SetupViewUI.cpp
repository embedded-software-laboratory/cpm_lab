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

using namespace std::placeholders;

SetupViewUI::SetupViewUI
    (
    std::shared_ptr<Deploy> _deploy_functions, 
    std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
    std::function<std::vector<uint8_t>()> _get_hlc_ids, 
    std::function<VehicleData()> _get_vehicle_data,
    std::function<void(bool, bool)> _reset_timer,
    std::function<void()> _on_simulation_start,
    std::function<void()> _on_simulation_stop,
    std::function<void(bool)> _set_commonroad_tab_sensitive,
    unsigned int argc, 
    char *argv[]
    ) 
    :
    deploy_functions(_deploy_functions),
    vehicle_control(_vehicle_control),
    get_hlc_ids(_get_hlc_ids),
    get_vehicle_data(_get_vehicle_data),
    reset_timer(_reset_timer),
    on_simulation_start(_on_simulation_start),
    on_simulation_stop(_on_simulation_stop),
    set_commonroad_tab_sensitive(_set_commonroad_tab_sensitive)
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
        _get_hlc_ids,
        deploy_functions,
        [this] () { set_sensitive(true); },
        [this] () { button_kill->set_sensitive(true); },
        [this] () { perform_post_kill_cleanup(); }
    );

    //Set initial text of script path (from previous program execution, if that existed)
    script_path->set_text(FileChooserUI::get_last_execution_path());
    
    //Regularly check / update which real vehicles are currently turned on, to use them when the experiment is deployed
    is_deployed.store(false);
    vehicle_data_thread_running.store(true);
    check_real_vehicle_data_thread = std::thread([&]{
        while(vehicle_data_thread_running.load())
        {
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

                //The vehicle toggles must be updated in the UI thread
                update_vehicle_toggles.store(true);
                ui_dispatcher.emit();
            }

            //Sleep for a while, then update again
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    create_rtt_thread();
    lcc_closed.store(false);
}

SetupViewUI::~SetupViewUI() {
    destroy_rtt_thread();
    kill_crash_check_thread();

    //Kill real vehicle data thread
    vehicle_data_thread_running.store(false);
    if(check_real_vehicle_data_thread.joinable())
    {
        check_real_vehicle_data_thread.join();
    }
}

void SetupViewUI::create_rtt_thread()
{
    //Create thread to measure RTT regularly
    run_rtt_thread.store(true);
    check_rtt_thread = std::thread(
        [&](){
            while(run_rtt_thread.load())
            {
                auto rtt = cpm::RTTTool::Instance().measure_rtt();

                //Check "best" RTT
                //if (rtt.first > )

                std::cout << "RTT measurement: " << rtt.first << "ns (best), " << rtt.second << "ns ('worst')" << std::endl;

                //No waiting required, the functions itself already includes over 0.5s of waiting times
            }
        }
    );
}

void SetupViewUI::destroy_rtt_thread()
{
    run_rtt_thread.store(false);
    if (check_rtt_thread.joinable())
    {
        check_rtt_thread.join();
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

    //Join all old threads
    destroy_rtt_thread();
    kill_crash_check_thread();

    //Kill real vehicle data thread
    vehicle_data_thread_running.store(false);
    if(check_real_vehicle_data_thread.joinable())
    {
        check_real_vehicle_data_thread.join();
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
            std::vector<FileChooserUI::Filter> { application_filter, all_filter }
        );
    }
    else
    {
        cpm::Logging::Instance().write(
            1, 
            "%s", 
            "ERROR: Main window reference is missing, cannot create file chooser dialog"
        );
    }
    
}

void SetupViewUI::file_explorer_callback(std::string file_string, bool has_file)
{
    if (has_file)
    {
        script_path->set_text(file_string.c_str());
    }
}

void SetupViewUI::ui_dispatch()
{
    //Update vehicle toggles for real vehicles or take care of upload window
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

    //Create dialog window that informs the user about a crash of one of the programs they started (besides vehicles, this can be seen in the UI already)
    std::unique_lock<std::mutex> lock_crashes(crashed_mutex);
    if (newly_crashed_participants.size() > 0)
    {
        std::stringstream crash_report;
        crash_report << "Newly crashed programs:\n";
        for (auto& program : newly_crashed_participants)
        {
            crash_report << "\t" << program << "\n";
        }
        crash_report << "\nPreviously crashed programs:\n";
        for (auto& program : already_crashed_participants)
        {
            crash_report << "\t" << program << "\n";
        }

        //Remember previous crashes, clear for new crashed - do this both in UI and check thread, as they interoperate non-deterministically
        for (auto& participant : newly_crashed_participants)
        {
            already_crashed_participants.insert(participant);
        }
        newly_crashed_participants.clear();

        //Close window if it is still open
        if (crash_dialog)
        {
            crash_dialog->close();
            crash_dialog.reset();
        }

        //Create new window
        crash_dialog = std::make_shared<Gtk::MessageDialog>(
            get_main_window(),
            crash_report.str(),
            false,
            Gtk::MessageType::MESSAGE_INFO,
            Gtk::ButtonsType::BUTTONS_CLOSE,
            true
        );
    
        //Connect new window with parent, show window
        crash_dialog->set_transient_for(get_main_window());
        crash_dialog->property_destroy_with_parent().set_value(true);
        crash_dialog->show();

        //Callback for closing
        crash_dialog->signal_response().connect(
            [this] (auto response)
            {
                if (response == Gtk::ResponseType::RESPONSE_CLOSE)
                {
                    crash_dialog->close();
                }
            }
        );
    }
    lock_crashes.unlock();
}

void SetupViewUI::deploy_applications() {
    //Grey out UI until kill is clicked
    set_sensitive(false);
    is_deployed.store(true);

    //We do not want this msg overhead during simulation
    destroy_rtt_thread();

    //Create log folder for all applications that are started on this machine
    deploy_functions->create_log_folder("lcc_script_logs");

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
    if(lab_mode_on && labcam_toggled){
        std::cerr << "RECORDING LABCAM" << std::endl;
        auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); 
        labcam->startRecording("/tmp/", ctime(&timenow));
    }else{
        std::cerr << "NOT RECORDING LABCAM" << std::endl;
    }

#endif
    
    // Recording
    deploy_functions->deploy_recording();

    //Remote deployment of scripts on HLCs or local deployment depending on switch state
    if(deploy_remote_toggled)
    {
        //Deploy on each HLC
        button_kill->set_sensitive(false);

        //Get current online vehicle and high_level_controller IDs
        std::vector<unsigned int> vehicle_ids = get_vehicle_ids_active();
        std::vector<uint8_t> hlc_ids;
        if (get_hlc_ids)
        {
            hlc_ids = get_hlc_ids();
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

        //Deploy remote
        auto simulated_time = switch_simulated_time->get_active();
        std::string path = script_path->get_text().c_str();
        std::string params = script_params->get_text().c_str();

        upload_manager->deploy_remote(simulated_time, path, params, hlc_ids, vehicle_ids);
    }
    else
    {
        deploy_functions->deploy_local_hlc(switch_simulated_time->get_active(), get_vehicle_ids_active(), script_path->get_text().c_str(), script_params->get_text().c_str());
    }

    //Deploy crash check thread
    crash_check_running.store(true);
    thread_deploy_crash_check = std::thread(
        [this, deploy_remote_toggled, lab_mode_on, labcam_toggled] () {
            //Give programs time to actually start
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            while(crash_check_running.load())
            {
                auto crashed_participants = deploy_functions->check_for_crashes(deploy_remote_toggled, lab_mode_on, labcam_toggled);

                if (crashed_participants.size() > 0)
                {                    
                    std::lock_guard<std::mutex> lock(crashed_mutex);

                    //Remember previous crashes, clear for new crashed
                    for (auto& participant : newly_crashed_participants)
                    {
                        already_crashed_participants.insert(participant);
                    }
                    newly_crashed_participants.clear();

                    //Store all new crashes so that they can be shown in the UI separately (New and old crashes)
                    bool new_crash_detected = false;
                    for (auto& participant : crashed_participants)
                    {
                        if (already_crashed_participants.find(participant) == already_crashed_participants.end())
                        {
                            newly_crashed_participants.push_back(participant);
                            new_crash_detected = true;
                        }
                    }

                    //If a new crash was detected, notify the UI thread. It will check the size of newly_crashed participants and create a dialog if is greater than zero
                    if (new_crash_detected)
                    {
                        //Log the new crash
                        std::stringstream program_stream;
                        program_stream << "New: ";
                        for (auto& entry : newly_crashed_participants)
                        {
                            program_stream << entry << " | ";
                        }

                        if (already_crashed_participants.size() > 0)
                        {
                            program_stream << " - Previous: ";
                            for (auto& entry : already_crashed_participants)
                            {
                                program_stream << entry << " | ";
                            }
                        }
                        
                        cpm::Logging::Instance().write("The following programs crashed during simulation: %s", program_stream.str().c_str());

                        ui_dispatcher.emit();
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    );
}

void SetupViewUI::kill_crash_check_thread()
{
    crash_check_running.store(false);
    if (thread_deploy_crash_check.joinable())
    {
        thread_deploy_crash_check.join();
    }
}

void SetupViewUI::kill_deployed_applications() {
    //Kill crash check first, or else we get undesired error messages
    kill_crash_check_thread();

    if (! lcc_closed.load())
    {
        //Re-create RTT thread
        create_rtt_thread();
    }

    // Stop LabCam
#ifndef SIMULATION
    labcam->stopRecording();
#endif

    is_deployed.store(false);

    //Kill scripts locally or remotely - do not perform remote kill (with new UI window creations etc) if the whole lcc is in the process of being destructed
    if(switch_deploy_remote->get_active() && !lcc_closed.load())
    {
        //Performs post_kill_cleanup after remote kill
        upload_manager->kill_remote();
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
    //(call all functions that registered for this callback in main)
    if(on_simulation_stop)
    {
        on_simulation_stop();
    }
    else
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
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_state(VehicleToggle::ToggleState::Simulated);
        deploy_functions->deploy_sim_vehicle(vehicle_toggle->get_id(), switch_simulated_time->get_active());
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
}

void SetupViewUI::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
    upload_manager->set_main_window_callback(_get_main_window);
}

Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}