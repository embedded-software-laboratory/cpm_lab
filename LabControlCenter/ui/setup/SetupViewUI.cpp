#include "SetupViewUI.hpp"
#include <cstdlib>

using namespace std::placeholders;

SetupViewUI::SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, unsigned int argc, char *argv[]) :
    timer_ui(_timer_ui)
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
    
    builder->get_widget("switch_launch_middleware", switch_launch_middleware);

    builder->get_widget("switch_launch_ips", switch_launch_ips);

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

    assert(switch_launch_middleware);

    assert(switch_launch_ips);

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
    builder->get_widget("switch_launch_ips", switch_launch_ips);
    switch_launch_ips->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_ips_set));
}

SetupViewUI::~SetupViewUI() {
    //TODO: Klappt nicht -> ergo auch bei deploy vorher clearen? (tmux kill-server)
    kill_deployed_applications();
}

void SetupViewUI::switch_timer_set()
{
    timer_ui->reset(switch_simulated_time->get_active());
}

void SetupViewUI::switch_ips_set()
{
    if(switch_launch_ips->get_active())
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

void SetupViewUI::deploy_applications() {
    //Grey out UI until kill is clicked
    set_sensitive(false);

    deploy_hlc_scripts();

    if (switch_launch_middleware->get_active()) {
        deploy_middleware();
    }

    deploy_sim_vehicles();
}

void SetupViewUI::kill_deployed_applications() {
    //Kill timer in UI as well, as it should not show invalid information
    timer_ui->reset(switch_simulated_time->get_active());

    kill_hlc_scripts();

    if (switch_launch_middleware->get_active()) {
        kill_middleware();
    }

    kill_vehicles();

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

    //Generate command
    std::stringstream command;
    command 
        << "tmux new-session -d "
        << "-s \"vehicle_" << id << "\" "
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
    for (const unsigned int id : get_active_vehicle_ids())
    {
        kill_vehicle(id);
    } 
}

void SetupViewUI::kill_vehicle(unsigned int id) {
    std::stringstream vehicle_id;
    vehicle_id << "vehicle_" << id;
    
    kill_session(vehicle_id.str());
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

    switch_launch_middleware->set_sensitive(is_sensitive);

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