#include "SetupViewUI.hpp"
#include <cstdlib>
#include <chrono>

using namespace std::placeholders;

SetupViewUI::SetupViewUI
    (
    std::shared_ptr<Deploy> _deploy_functions, 
    std::shared_ptr<VehicleAutomatedControl> _vehicle_control, 
    std::function<std::vector<uint8_t>()> _get_hlc_ids,
    std::function<void(bool)> _reset_timer,
    std::function<void()> _reset_time_series_aggregator,
    std::function<void()> _reset_trajectories,
    std::function<void()> _reset_vehicle_view,
    std::function<void()> _reset_visualization_commands,
    std::function<void()> _reset_logs,
    unsigned int argc, 
    char *argv[]
    ) 
    :
    deploy_functions(_deploy_functions),
    vehicle_control(_vehicle_control),
    get_hlc_ids(_get_hlc_ids),
    reset_timer(_reset_timer),
    reset_time_series_aggregator(_reset_time_series_aggregator),
    reset_trajectories(_reset_trajectories),
    reset_vehicle_view(_reset_vehicle_view),
    reset_visualization_commands(_reset_visualization_commands),
    reset_logs(_reset_logs)
{
    builder = Gtk::Builder::create_from_file("ui/setup/setup.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("setup_box", setup_box);
    builder->get_widget("script_path", script_path);
    builder->get_widget("script_params", script_params);
    builder->get_widget("button_choose_script", button_choose_script);
    
    builder->get_widget("button_select_none", button_select_none);
    builder->get_widget("button_select_all_simulated", button_select_all_simulated);
    builder->get_widget("button_select_all_real", button_select_all_real);

    builder->get_widget("switch_simulated_time", switch_simulated_time);

    builder->get_widget("switch_deploy_remote", switch_deploy_remote);

    builder->get_widget("switch_lab_mode", switch_lab_mode);
    builder->get_widget("switch_record_labcam", switch_record_labcam);

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
    assert(button_select_all_real);
    
    assert(switch_simulated_time);
    assert(switch_deploy_remote);
    assert(switch_lab_mode);
    assert(switch_record_labcam);

    assert(button_deploy);
    assert(button_kill);

    assert(vehicle_flowbox);

    //Create vehicle toggles
    for (unsigned int id = 1; id <= 10; ++id)
    {
        vehicle_toggles.emplace_back(std::make_shared<VehicleToggle>(id));
    }

    //Add vehicle toggles to vehicle flowbox
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_flowbox->add(*(vehicle_toggle->get_parent()));
    }

    // Create labcam
    labcam = new LabCamIface();

    //Register button callbacks
    button_deploy->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::deploy_applications));
    button_kill->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::kill_deployed_applications));
    button_choose_script->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::open_file_explorer));
    button_select_all_real->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles_real));
    button_select_all_simulated->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles_sim));
    button_select_none->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_no_vehicles));

    //Extract relevant parameters from command line
    cmd_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);

    //Set switch to current simulated time value - due to current design sim. time cannot be changed after the LCC has been started
    switch_simulated_time->set_active(cmd_simulated_time);
    switch_simulated_time->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_timer_set));

    //The IPS can be startet and restarted manually independent of the other components
    builder->get_widget("switch_lab_mode", switch_lab_mode);
    switch_lab_mode->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_ips_set));

    //Take care of GUI thread and worker thread separately
    ui_dispatcher.connect(sigc::mem_fun(*this, &SetupViewUI::ui_dispatch));
    thread_count.store(0);
    notify_count = 0;
    participants_available.store(false);
    kill_called.store(false);
}

SetupViewUI::~SetupViewUI() {
    //TODO: Klappt nicht -> ergo auch bei deploy vorher clearen? (tmux kill-server)
    kill_deployed_applications();

    //Join all old threads
    kill_all_threads();
}

void SetupViewUI::switch_timer_set()
{
    reset_timer(switch_simulated_time->get_active());
}

void SetupViewUI::switch_ips_set()
{
    if(switch_lab_mode->get_active())
    {
        deploy_functions->deploy_ips();
    }
    else
    {
        deploy_functions->kill_ips();
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
        cpm::Logging::Instance().write("%s", "ERROR: Main window reference is missing, cannot create file chooser dialog");
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
    std::lock_guard<std::mutex> lock_msg(error_msg_mutex);
    if (error_msg.size() > 0)
    {
        for (auto &msg : error_msg)
        {
            upload_window->add_error_message(msg);
        }
        error_msg.clear();
    }
    else 
    {
        //The only current job for ui_dispatch is to close the upload window shown after starting the upload threads, when all threads have been closed
        //Plus now, kill is not grayed out anymore
        if (upload_threads.size() != 0 && upload_window)
        {
            upload_window->close();
            button_kill->set_sensitive(true);
        }

        //Join all old threads
        kill_all_threads();

        //If kill caused the UI dispatch, clean up after everything has been killed
        if (kill_called.load())
        {
            perform_post_kill_cleanup();
            kill_called.store(false);
        }

        //Free the UI if the upload was not successful
        if (!participants_available.load())
        {
            set_sensitive(true);
        }
    }
}

void SetupViewUI::notify_upload_finished(uint8_t hlc_id, bool upload_success)
{
    //Just try to join all worker threads here
    std::lock_guard<std::mutex> lock(notify_callback_in_use);

    //This should never be the case
    //If this happens, the thread count has been initialized incorrectly
    if (thread_count.load() == 0)
    {
        std::cerr << "WARNING: Upload thread count has not been initialized correctly!" << std::endl;
    }

    //Trigger error msg if the upload failed
    if (!upload_success)
    {
        std::lock_guard<std::mutex> lock_msg(error_msg_mutex);
        std::stringstream error_msg_stream;
        error_msg_stream << "ERROR: Connection or upload failed for HLC ID " << static_cast<int>(hlc_id) << std::endl;
        error_msg.push_back(error_msg_stream.str());
        ui_dispatcher.emit();
    }

    //Also count notify amount s.t one can check if the thread count has been set properly
    thread_count.fetch_sub(1);
    ++notify_count;

    std::cout << thread_count.load() << std::endl;
    if (thread_count.load() == 0)
    {
        notify_count = 0;

        //Close upload window again, but only after a while
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ui_dispatcher.emit();
    }
    else if (notify_count == upload_threads.size())
    {
        std::cerr << "WARNING: Upload thread count has not been initialized correctly!" << std::endl;

        notify_count = 0;

        //Close upload window again, but only after a while
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ui_dispatcher.emit();
    }
}

void SetupViewUI::kill_all_threads()
{
    //Join all old threads - gets called from destructor, kill and when the last thread finished (in the ui thread dispatcher)
    for (auto& thread : upload_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
    upload_threads.clear();
}

bool SetupViewUI::check_if_online(uint8_t hlc_id)
{
    //Check if the HLC is still online (in get_hlc_ids)
    std::vector<uint8_t> hlc_ids = get_hlc_ids();
    return std::find(hlc_ids.begin(), hlc_ids.end(), static_cast<uint8_t>(hlc_id)) != hlc_ids.end();
}

void SetupViewUI::deploy_applications() {
    //Grey out UI until kill is clicked
    set_sensitive(false);

    //Create log folder for all applications that are started on this machine
    deploy_functions->create_log_folder("lcc_script_logs");

    //Reset old UI elements (difference to kill: Also reset the Logs)
    //Kill timer in UI as well, as it should not show invalid information
    //Reset all relevant UI parts
    reset_timer(switch_simulated_time->get_active());
    usleep(100000); //Make sure that the stop signal does not arrive at newly created participants (IS THIS SAFE ENOUGH?)
    reset_time_series_aggregator();
    reset_trajectories();
    reset_vehicle_view();
    reset_visualization_commands();
    
    //We also reset the log file here - if you want to use it, make sure to rename it before you start a new simulation!
    reset_logs();

    // LabCam
    if(switch_record_labcam->get_active() && switch_lab_mode->get_active()){
        std::cerr << "RECORDING LABCAM" << std::endl;
        auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); 
        labcam->startRecording("/tmp/", ctime(&timenow));
    }else{
        std::cerr << "NOT RECORDING LABCAM" << std::endl;
    }

    //Remote deployment of scripts on HLCs or local deployment depending on switch state
    if(switch_deploy_remote->get_active())
    {
        //Deploy on each HLC
        button_kill->set_sensitive(false);

        //Get current online vehicle and hlc IDs
        std::vector<unsigned int> vehicle_ids = get_vehicle_ids_active();
        std::vector<uint8_t> hlc_ids;
        if (get_hlc_ids)
        {
            hlc_ids = get_hlc_ids();
        }
        else 
        {
            std::cerr << "No lookup function to get HLC IDs given, cannot deploy on HLCs" << std::endl;
            return;
        }

        //Show window indicating that the upload process currently takes place
        //An error message is shown if no HLC is online - in that case, take additional action here as well: Just show the window and deploy nothing
        if (get_main_window)
        {
            upload_window = make_shared<UploadWindow>(get_main_window(), vehicle_ids, hlc_ids);
        }
        else
        {
            cpm::Logging::Instance().write("%s", "ERROR: Main window reference is missing, cannot create upload dialog");
        }

        //Do not deploy anything remotely if no HLCs are online or if no vehicles were selected
        if (hlc_ids.size() == 0 || vehicle_ids.size() == 0)
        {
            //Waits a few seconds before the window is closed again 
            //Window still needs UI dispatcher (else: not shown because UI gets unresponsive), so do this by using a thread + atomic variable (upload_failed)
            participants_available.store(false); //No HLCs available
            thread_count.store(1);
            upload_threads.push_back(std::thread(
                [this] () {
                    usleep(3000000);
                    this->notify_upload_finished(0, true);
                }
            ));
            return;
        }

        //Deploy simulated vehicles locally
        deploy_functions->deploy_sim_vehicles(get_vehicle_ids_simulated(), switch_simulated_time->get_active());
        
        //Match lowest vehicle ID to lowest HLC ID
        std::sort(vehicle_ids.begin(), vehicle_ids.end());
        std::sort(hlc_ids.begin(), hlc_ids.end());
        size_t min_hlc_vehicle = std::min(hlc_ids.size(), vehicle_ids.size());

        //Deploy on each HLC individually, using different threads
        participants_available.store(true); //HLCs are available
        thread_count.store(min_hlc_vehicle);
        for (size_t i = 0; i < min_hlc_vehicle; ++i)
        {
            //Deploy on hlc with given vehicle id(s)
            std::stringstream vehicle_id_stream;
            vehicle_id_stream << vehicle_ids.at(i);

            //Create variables for the thread
            unsigned int hlc_id = static_cast<unsigned int>(hlc_ids.at(i));
            std::string vehicle_string = vehicle_id_stream.str();

            //Create thread
            upload_threads.push_back(std::thread(
                [this, hlc_id, vehicle_string] () {
                    bool deploy_worked = deploy_functions->deploy_remote_hlc(
                        hlc_id, 
                        vehicle_string, 
                        switch_simulated_time->get_active(), 
                        script_path->get_text().c_str(), 
                        script_params->get_text().c_str(), 
                        remote_deploy_timeout,
                        std::bind(&SetupViewUI::check_if_online, this, hlc_id)
                    );
                    this->notify_upload_finished(hlc_id, deploy_worked);
                }
            ));
        }
    }
    else
    {
        deploy_functions->deploy_sim_vehicles(get_vehicle_ids_simulated(), switch_simulated_time->get_active());

        deploy_functions->deploy_local_hlc(switch_simulated_time->get_active(), get_vehicle_ids_active(), script_path->get_text().c_str(), script_params->get_text().c_str());
    }
}

void SetupViewUI::kill_deployed_applications() {

    // Stop LabCam
    labcam->stopRecording();

    //Kill scripts locally or remotely
    if(switch_deploy_remote->get_active())
    {
        std::vector<uint8_t> hlc_ids;
        if (get_hlc_ids)
        {
            hlc_ids = get_hlc_ids();
        }
        else 
        {
            std::cerr << "No lookup function to get HLC IDs given, cannot kill on HLCs" << std::endl;
            return;
        }

        //Show window indicating that the upload process currently takes place
        //An error message is shown if no HLC is online - in that case, take additional action here as well: Just show the window and deploy nothing
        if (get_main_window)
        {
            upload_window = make_shared<UploadWindow>(get_main_window(), std::vector<unsigned int>(), hlc_ids);
            upload_window->set_text("Killing on remote HLCs...");
        }
        else
        {
            cpm::Logging::Instance().write("%s", "ERROR: Main window reference is missing, cannot create upload dialog");
        }
        
        //Let the UI dispatcher know that kill-related actions need to be performed after all threads have finished
        kill_called.store(true);


        //If a HLC went offline in between, we assume that it crashed and thus just use this script on all remaining running HLCs
        thread_count.store(hlc_ids.size());
        for (const auto& hlc_id : hlc_ids)
        {
            //Create thread
            upload_threads.push_back(std::thread(
                [this, hlc_id] () {
                    bool kill_worked = deploy_functions->kill_remote_hlc(
                        hlc_id, 
                        remote_kill_timeout,
                        std::bind(&SetupViewUI::check_if_online, this, hlc_id)
                    );
                    this->notify_upload_finished(hlc_id, kill_worked);
                }
            ));
        }
    }
    else 
    {
        deploy_functions->kill_local_hlc();
        perform_post_kill_cleanup();
    }

    deploy_functions->kill_vehicles(get_vehicle_ids_simulated(), get_vehicle_ids_active());

    //The rest is done in perform_post_kill_cleanup when the UI window closed (when all threads are killed) 
    //But only if threads are used, so only in case of remote deployment
    //For local deployment, perform_post_kill_cleanup is called directly
}

void SetupViewUI::perform_post_kill_cleanup()
{
    //Kill timer in UI as well, as it should not show invalid information
    //TODO: Reset Logs? They might be interesting even after the simulation was stopped, so that should be done separately/never (there's a log limit)/at start?
    //Reset all relevant UI parts
    reset_timer(switch_simulated_time->get_active());
    reset_time_series_aggregator();
    reset_trajectories();
    reset_vehicle_view();
    reset_visualization_commands();

    //Undo grey out
    set_sensitive(true);
}

std::vector<unsigned int> SetupViewUI::get_vehicle_ids_active() {
    std::vector<unsigned int> active_vehicle_ids;

    for (auto& vehicle_toggle : vehicle_toggles)
    {
        if (vehicle_toggle->get_state() != VehicleToggle::Off)
        {
            active_vehicle_ids.push_back(vehicle_toggle->get_id());
        }
    }

    return active_vehicle_ids;
}

std::vector<unsigned int> SetupViewUI::get_vehicle_ids_real() {
    std::vector<unsigned int> active_vehicle_ids;

    for (auto& vehicle_toggle : vehicle_toggles)
    {
        if (vehicle_toggle->get_state() == VehicleToggle::On)
        {
            active_vehicle_ids.push_back(vehicle_toggle->get_id());
        }
    }

    return active_vehicle_ids;
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
    button_select_all_real->set_sensitive(is_sensitive);

    button_deploy->set_sensitive(is_sensitive);

    switch_simulated_time->set_sensitive(is_sensitive);

    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_sensitive(is_sensitive);
    }
}

void SetupViewUI::select_all_vehicles_real()
{
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_state(VehicleToggle::ToggleState::On);
    }
}

void SetupViewUI::select_all_vehicles_sim()
{
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_state(VehicleToggle::ToggleState::Simulated);
    }
}

void SetupViewUI::select_no_vehicles()
{
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_toggle->set_state(VehicleToggle::ToggleState::Off);
    }
}

void SetupViewUI::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}

Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}