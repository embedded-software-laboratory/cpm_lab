#include "SetupViewUI.hpp"
#include <cstdlib>

using namespace std::placeholders;

SetupViewUI::SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, std::shared_ptr<VehicleAutomatedControl> _vehicle_control, std::function<std::vector<uint8_t>()> _get_hlc_ids, unsigned int argc, char *argv[]) :
    timer_ui(_timer_ui),
    vehicle_control(_vehicle_control),
    get_hlc_ids(_get_hlc_ids)
{
    builder = Gtk::Builder::create_from_file("ui/setup/setup.glade");

    builder->get_widget("setup_box", parent);
    builder->get_widget("script_path", script_path);
    builder->get_widget("script_params", script_params);
    builder->get_widget("button_choose_script", button_choose_script);
    
    builder->get_widget("button_select_none", button_select_none);
    builder->get_widget("button_select_all_simulated", button_select_all_simulated);
    builder->get_widget("button_select_all_real", button_select_all_real);

    builder->get_widget("switch_simulated_time", switch_simulated_time);

    builder->get_widget("switch_deploy_remote", switch_deploy_remote);

    builder->get_widget("switch_lab_mode", switch_lab_mode);

    builder->get_widget("button_deploy", button_deploy);
    builder->get_widget("button_kill", button_kill);

    builder->get_widget("vehicle_flowbox", vehicle_flowbox);

    assert(parent);
    assert(script_path);
    assert(script_params);
    assert(button_choose_script);

    assert(button_select_none);
    assert(button_select_all_simulated);
    assert(button_select_all_real);
    
    assert(switch_simulated_time);

    assert(switch_deploy_remote);

    assert(switch_lab_mode);

    assert(button_deploy);
    assert(button_kill);

    assert(vehicle_flowbox);

    //Create vehicle toggles
    for (unsigned int id = 1; id <= 6; ++id)
    {
        vehicle_toggles.emplace_back(std::make_shared<VehicleToggle>(id));
    }

    //Add vehicle toggles to vehicle flowbox
    for (auto& vehicle_toggle : vehicle_toggles)
    {
        vehicle_flowbox->add(*(vehicle_toggle->get_parent()));
    }

    //Register button callbacks
    button_deploy->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::deploy_applications));
    button_kill->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::kill_deployed_applications));
    button_choose_script->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::open_file_explorer));
    button_select_all_real->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles_real));
    button_select_all_simulated->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles_sim));
    button_select_none->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_no_vehicles));

    //Extract relevant parameters from command line
    cmd_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    cmd_domain_id = cpm::cmd_parameter_int("dds_domain", 0, argc, argv);
    cmd_dds_initial_peer = cpm::cmd_parameter_string("dds_initial_peer", "", argc, argv);

    //Set switch to current simulated time value - due to current design sim. time cannot be changed after the LCC has been started
    switch_simulated_time->set_active(cmd_simulated_time);
    switch_simulated_time->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_timer_set));

    //The IPS can be startet and restarted manually independent of the other components
    builder->get_widget("switch_lab_mode", switch_lab_mode);
    switch_lab_mode->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_ips_set));

    //Take care of GUI thread and worker thread separately
    ui_dispatcher.connect(sigc::mem_fun(*this, &SetupViewUI::ui_dispatch));
    notify_count = 0;
}

SetupViewUI::~SetupViewUI() {
    //TODO: Klappt nicht -> ergo auch bei deploy vorher clearen? (tmux kill-server)
    kill_deployed_applications();

    //Join all old threads
    kill_all_threads();
}

void SetupViewUI::switch_timer_set()
{
    timer_ui->reset(switch_simulated_time->get_active());
}

void SetupViewUI::switch_ips_set()
{
    if(switch_lab_mode->get_active())
    {
        deploy_ips();
    }
    else
    {
        kill_ips();
    }
}

using namespace std::placeholders;
void SetupViewUI::open_file_explorer()
{
    std::vector<std::string> filter_name{"Application", "Matlab script"}; 
    std::vector<std::string> filter_type{"application/x-sharedlib", "text/x-matlab"};
    file_chooser_window = make_shared<FileChooserUI>(std::bind(&SetupViewUI::file_explorer_callback, this, _1, _2), filter_name, filter_type);
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
    //The only current job for ui_dispatch is to close the upload window shown after starting the upload threads, when all threads have been closed
    if (upload_threads.size() != 0 && upload_window)
    {
        upload_window->close();
    }

    //Join all old threads
    kill_all_threads();
}

void SetupViewUI::notify_upload_finished()
{
    //Just try to join all worker threads here
    std::lock_guard<std::mutex> lock(notify_callback_in_use);
    ++notify_count;
    std::cout << notify_count << std::endl;
    if (notify_count == upload_threads.size())
    {
        //Clear values for future use
        notify_count = 0;

        //Close upload window again
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

void SetupViewUI::deploy_applications() {
    //Grey out UI until kill is clicked
    set_sensitive(false);

    //Deploy simulated vehicles locally
    deploy_sim_vehicles();

    //Remote deployment of scripts on HLCs or local deployment depending on switch state
    if(switch_deploy_remote->get_active())
    {
        //Deploy on each HLC
        //Get current online vehicle and hlc IDs
        std::vector<unsigned int> vehicle_ids = get_active_vehicle_ids();
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
        
        //Match lowest vehicle ID to lowest HLC ID
        std::sort(vehicle_ids.begin(), vehicle_ids.end());
        std::sort(hlc_ids.begin(), hlc_ids.end());
        size_t min_hlc_vehicle = std::min(hlc_ids.size(), vehicle_ids.size());

        //Show window indicating that the upload process currently takes place
        upload_window = make_shared<UploadWindow>(vehicle_ids, hlc_ids);

        //Deploy on each HLC individually, using different threads
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
                    this->deploy_remote_hlc(hlc_id, vehicle_string);
                    this->notify_upload_finished();
                }
            ));
        }
    }
    else
    {
        deploy_hlc_scripts();

        deploy_middleware();
    }
}

void SetupViewUI::kill_deployed_applications() {
    //Kill timer in UI as well, as it should not show invalid information
    timer_ui->reset(switch_simulated_time->get_active());

    kill_hlc_scripts();

    kill_middleware();

    kill_vehicles();

    //Join all old threads
    kill_all_threads();

    //Undo grey out
    set_sensitive(true);
}

void SetupViewUI::deploy_hlc_scripts() {
    //TODO: Put into separate function
    std::string sim_time_string;
    if (switch_simulated_time->get_active())
    {
        sim_time_string = "true";
    }
    else 
    {
        sim_time_string = "false";
    }

    //Check if old session already exists - if so, kill it
    if (session_exists("hlc"))
    {
        kill_session("hlc");
    }

    std::vector<unsigned int> vehicle_ids = get_active_vehicle_ids();
    if (vehicle_ids.size() > 0)
    {
        std::stringstream vehicle_ids_stream;
        for (size_t index = 0; index < vehicle_ids.size() - 1; ++index)
        {
            vehicle_ids_stream << vehicle_ids.at(index) << ",";
        }
        vehicle_ids_stream << vehicle_ids.at(vehicle_ids.size() - 1);

        //Get script info, generate command
        std::string script_string(script_path->get_text().c_str());
        std::string script_path_string;
        std::string script_name_string;
        get_path_name(script_string, script_path_string, script_name_string);
        std::string parameters(script_params->get_text().c_str());
        std::stringstream command;

        auto matlab_type_pos = script_name_string.rfind(".m");
        if (matlab_type_pos != std::string::npos)
        {
            script_name_string = script_name_string.substr(0, matlab_type_pos);

            //Case: Matlab script
            command 
            << "tmux new-session -d "
            << "-s \"hlc\" "
            << "'source ~/dev/software/hlc/environment_variables.bash;"
            << "/opt/MATLAB/R2019a/bin/matlab -nodisplay -nosplash -logfile matlab.log -nodesktop -r \""
            << "cd " << script_path_string
            << "; " << script_name_string << "(1, " << vehicle_ids_stream.str() << ")\""
            << " >stdout_hlc.txt 2>stderr_hlc.txt'";
        }
        else if (script_name_string.find(".") == std::string::npos)
        {
            //Case: Any executable 
            command 
            << "tmux new-session -d "
            << "-s \"hlc\" "
            << "\"source ~/dev/software/hlc/environment_variables.bash;"
            << "cd " << script_path_string << ";./" << script_name_string
            << " --node_id=hlc"
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids_stream.str()
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        command 
            << " " << parameters << " >stdout_hlc.txt 2>stderr_hlc.txt\"";
        }
        else 
        {
            std::cout << "Warning: Could not run unknown script: Neither matlab nor C++ executable" << std::endl;
            return;
        }

        std::cout << command.str() << std::endl;

        //Execute command
        system(command.str().c_str());
    }
}

void SetupViewUI::kill_hlc_scripts() {
    kill_session("hlc");
}

void SetupViewUI::deploy_middleware() {
    std::string sim_time_string;
    if (switch_simulated_time->get_active())
    {
        sim_time_string = "true";
    }
    else 
    {
        sim_time_string = "false";
    }

    //Check if old session already exists - if so, kill it
    if (session_exists("middleware"))
    {
        kill_session("middleware");
    }

    //TODO Pass vehicle_ids vector as function parameter
    std::stringstream vehicle_ids_stream;
    std::vector<unsigned int> vehicle_ids = get_active_vehicle_ids();
    if (vehicle_ids.size() > 0)
    {
        for (size_t index = 0; index < vehicle_ids.size() - 1; ++index)
        {
            vehicle_ids_stream << vehicle_ids.at(index) << ",";
        }
        vehicle_ids_stream << vehicle_ids.at(vehicle_ids.size() - 1);

        //Generate command
        std::stringstream command;
        command 
            << "tmux new-session -d "
            << "-s \"middleware\" "
            << "\"source ~/dev/software/hlc/environment_variables.bash;cd ~/dev/software/hlc/middleware/build/;./middleware"
            << " --node_id=middleware"
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids_stream.str()
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        command 
            << " >stdout_middleware.txt 2>stderr_middleware.txt\"";

        //Execute command
        system(command.str().c_str());
    }
}

void SetupViewUI::kill_middleware() {
    kill_session("middleware");
}

void SetupViewUI::deploy_sim_vehicles() {
    for (const unsigned int id : get_vehicle_ids_simulated())
    {
        deploy_sim_vehicle(id);
    }
}

void SetupViewUI::deploy_sim_vehicle(unsigned int id) {
    std::string sim_time_string;
    if (switch_simulated_time->get_active())
    {
        sim_time_string = "true";
    }
    else 
    {
        sim_time_string = "false";
    }

    std::stringstream session_name;
    session_name << "vehicle_" << id;

    //Check if old session already exists - if so, kill it
    if (session_exists(session_name.str()))
    {
        kill_session(session_name.str());
    }

    //Generate command
    std::stringstream command;
    command 
        << "tmux new-session -d "
        << "-s \"" << session_name.str() << "\" "
        << "\"cd ~/dev/software/vehicle_raspberry_firmware/build_x64_sim;./vehicle_rpi_firmware "
        << "--simulated_time=" << sim_time_string
        << " --vehicle_id=" << id
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command 
        << " >stdout_vehicle" << id << ".txt 2>stderr_vehicle" << id << ".txt\"";

    //Execute command
    system(command.str().c_str());
}

void SetupViewUI::kill_vehicles() {
    for (const unsigned int id : get_vehicle_ids_simulated())
    {
        kill_vehicle(id);
    }

    //Also make all vehicles stop immediately, so that they do not continue to drive for a while   
    for (const auto id : get_active_vehicle_ids())
    {
        vehicle_control->stop_vehicle(static_cast<uint8_t>(id));
    }
}

void SetupViewUI::kill_vehicle(unsigned int id) {
    std::stringstream vehicle_id;
    vehicle_id << "vehicle_" << id;
    
    kill_session(vehicle_id.str());
}

void SetupViewUI::deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids) 
{
    // //TODO: WORK WITH TEMPLATE STRINGS AND PUT LOGIC INTO SEPARATE CLASS

    //Get the IP address from the current id
    std::stringstream ip_stream;
    ip_stream << "192.168.1.2";
    if (hlc_id < 10)
    {
        ip_stream << "0";
    }
    ip_stream << hlc_id;

    //Get the path of the script that is to be called
    std::string script_string(script_path->get_text().c_str());

    //Put all relevant arguments together
    //Script-arguments from user
    std::string parameters(script_params->get_text().c_str());

    std::string sim_time_string;
    if (switch_simulated_time->get_active())
    {
        sim_time_string = "true";
    }
    else 
    {
        sim_time_string = "false";
    }

    //Default arguments + user arguments
    std::stringstream script_argument_stream;
    std::stringstream middleware_argument_stream;
    auto matlab_type_pos = script_string.rfind(".m");
    if (matlab_type_pos != std::string::npos)
    {
        //Case: Matlab script - TODO: This is only to test one of my scripts, find standard param order (simulated time is here the )
        script_argument_stream << "1," << vehicle_ids;
    }
    else if (script_string.find(".") == std::string::npos)
    {        
        //Case: Any executable 
        script_argument_stream
            << " --node_id=hlc_" << vehicle_ids
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            script_argument_stream 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
    }
    //Settings for Middleware
    middleware_argument_stream
        << " --node_id=middleware_" << vehicle_ids
        << " --simulated_time=" << sim_time_string
        << " --vehicle_ids=" << vehicle_ids
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        middleware_argument_stream 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }

    //Copy all relevant data over to the remote system
    std::stringstream copy_command;
    //Okay, do this using a template script instead, I think that's better in this case
    copy_command << "bash /home/cpm/dev/software/LabControlCenter/bash/copy_to_remote.bash --ip=" << ip_stream.str() 
        << " --script_path=" << script_string 
        << " --script_arguments='" << script_argument_stream.str() << "'"
        << " --middleware_arguments='" << middleware_argument_stream.str() << "'";

    execute_command(copy_command.str().c_str());
}

std::vector<unsigned int> SetupViewUI::get_active_vehicle_ids() {
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

std::vector<unsigned int> SetupViewUI::get_vehicle_ids_realtime() {
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

void SetupViewUI::deploy_ips() {
    //Check if old session already exists - if so, kill it
    if (session_exists("ips_pipeline"))
    {
        kill_session("ips_pipeline");
    }

    //Generate command
    std::stringstream command_ips;
    command_ips 
        << "tmux new-session -d "
        << "-s \"ips_pipeline\" "
        << "\"cd ~/dev/software/ips2/;./build/ips_pipeline "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_ips 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_ips 
        << " >stdout_ips.txt 2>stderr_ips.txt\"";

    if (session_exists("ips_basler"))
    {
        kill_session("ips_basler");
    }

    //Generate command
    std::stringstream command_basler;
    command_basler 
        << "tmux new-session -d "
        << "-s \"ips_basler\" "
        << "\"cd ~/dev/software/ips2/;./build/BaslerLedDetection "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_basler 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_basler 
        << " >stdout_basler.txt 2>stderr_basler.txt\"";

    //Execute command
    system(command_ips.str().c_str());
    system(command_basler.str().c_str());
}

void SetupViewUI::kill_ips() {
    kill_session("ips_pipeline");
    kill_session("ips_basler");
}

void SetupViewUI::set_sensitive(bool is_sensitive) {
    script_path->set_sensitive(is_sensitive);
    script_params->set_sensitive(is_sensitive);
    
    button_select_none->set_sensitive(is_sensitive);
    button_select_all_simulated->set_sensitive(is_sensitive);
    button_select_all_real->set_sensitive(is_sensitive);

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

bool SetupViewUI::session_exists(std::string session_id)
{
    std::string running_sessions = execute_command("tmux ls");
    session_id += ":";
    return running_sessions.find(session_id) != std::string::npos;
}

void SetupViewUI::kill_session(std::string session_id)
{
    std::stringstream command;
    command 
        << "tmux kill-session -t \"" << session_id << "\"";

    //Execute command
    system(command.str().c_str());
}

std::string SetupViewUI::execute_command(const char* cmd) {
    //Code from stackoverflow
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("Could not use popen - deployment failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

void SetupViewUI::get_path_name(std::string& in, std::string& out_path, std::string& out_name)
{
    auto last_slash_pos = in.rfind("/");
    if (last_slash_pos != std::string::npos)
    {
        out_path = in.substr(0, last_slash_pos + 1);
        out_name = in.substr(last_slash_pos + 1, in.size() - last_slash_pos);
    }
    else 
    {
        out_name = in.substr(0, in.size() - last_slash_pos);
    }
}


Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}