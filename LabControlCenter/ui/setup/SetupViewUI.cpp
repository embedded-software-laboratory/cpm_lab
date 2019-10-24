#include "SetupViewUI.hpp"
#include <cstdlib>

using namespace std::placeholders;

SetupViewUI::SetupViewUI(std::shared_ptr<TimerViewUI> _timer_ui, int argc, char *argv[]) :
    timer_ui(_timer_ui)
{
    builder = Gtk::Builder::create_from_file("ui/setup/setup.glade");

    builder->get_widget("setup_box", parent);
    builder->get_widget("script_path", script_path);
    builder->get_widget("script_name", script_name);
    builder->get_widget("toggle_vehicle_1", toggle_vehicle_1);
    builder->get_widget("toggle_vehicle_2", toggle_vehicle_2);
    builder->get_widget("toggle_vehicle_3", toggle_vehicle_3);
    builder->get_widget("toggle_vehicle_4", toggle_vehicle_4);
    builder->get_widget("toggle_vehicle_5", toggle_vehicle_5);
    builder->get_widget("toggle_vehicle_6", toggle_vehicle_6);
    builder->get_widget("button_select_all_vehicles", button_select_all_vehicles);
    builder->get_widget("button_select_no_vehicles", button_select_no_vehicles);
    builder->get_widget("button_choose_script", button_choose_script);
    builder->get_widget("switch_simulated_time", switch_simulated_time);
    builder->get_widget("switch_launch_simulated_vehicles", switch_launch_simulated_vehicles);
    builder->get_widget("switch_launch_cloud_discovery", switch_launch_cloud_discovery);
    builder->get_widget("switch_launch_ips", switch_launch_ips);
    builder->get_widget("switch_launch_middleware", switch_launch_middleware);
    builder->get_widget("button_deploy", button_deploy);
    builder->get_widget("button_kill", button_kill);

    assert(parent);
    assert(script_path);
    assert(script_name);
    assert(toggle_vehicle_1);
    assert(toggle_vehicle_2);
    assert(toggle_vehicle_3);
    assert(toggle_vehicle_4);
    assert(toggle_vehicle_5);
    assert(toggle_vehicle_6);
    assert(button_select_all_vehicles);
    assert(button_select_no_vehicles);
    assert(button_choose_script);
    assert(switch_simulated_time);
    assert(switch_launch_simulated_vehicles);
    assert(switch_launch_cloud_discovery);
    assert(switch_launch_ips);
    assert(switch_launch_middleware);
    assert(button_deploy);
    assert(button_kill);

    //Register button callbacks
    button_deploy->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::deploy_applications));
    button_kill->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::kill_deployed_applications));
    button_choose_script->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::open_file_explorer));
    button_select_all_vehicles->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_all_vehicles));
    button_select_no_vehicles->signal_clicked().connect(sigc::mem_fun(this, &SetupViewUI::select_no_vehicles));

    //Extract relevant parameters from command line
    cmd_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    cmd_domain_id = cpm::cmd_parameter_int("dds_domain", 0, argc, argv);
    cmd_dds_initial_peer = cpm::cmd_parameter_string("dds_initial_peer", "", argc, argv);

    //Set switch to current simulated time value - due to current design sim. time cannot be changed after the LCC has been started
    switch_simulated_time->set_active(cmd_simulated_time);
    switch_simulated_time->property_active().signal_changed().connect(sigc::mem_fun(this, &SetupViewUI::switch_timer_set));
}

SetupViewUI::~SetupViewUI() {
    //TODO: Klappt nicht -> ergo auch bei deploy vorher clearen? (tmux kill-server)
    kill_deployed_applications();
}

void SetupViewUI::switch_timer_set()
{
    timer_ui->reset(switch_simulated_time->get_active());
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
        auto last_slash_pos = file_string.rfind("/");
        if (last_slash_pos != std::string::npos)
        {
            script_path->set_text(file_string.substr(0, last_slash_pos + 1));
            script_name->set_text(file_string.substr(last_slash_pos + 1, file_string.size() - last_slash_pos));
        }
        else 
        {
            script_name->set_text(file_string.substr(0, file_string.size() - last_slash_pos));
        }
    }
}

void SetupViewUI::deploy_applications() {
    //Grey out UI until kill is clicked
    set_sensitive(false);

    deploy_hlc_scripts();

    if (switch_launch_middleware->get_active()) {
        deploy_middleware();
    }

    if (switch_launch_simulated_vehicles->get_active()) {
        deploy_vehicles();
    }

    if (switch_launch_ips->get_active()) {
        deploy_ips();
    }

    if (switch_launch_cloud_discovery->get_active()) {
        deploy_cloud_discovery();
    }
}

void SetupViewUI::kill_deployed_applications() {
    //Kill timer in UI as well, as it should not show invalid information
    timer_ui->reset(switch_simulated_time->get_active());

    kill_hlc_scripts();

    if (switch_launch_middleware->get_active()) {
        kill_middleware();
    }

    if (switch_launch_simulated_vehicles->get_active()) {
        kill_vehicles();
    }

    if (switch_launch_ips->get_active()) {
        kill_ips();
    }

    if (switch_launch_cloud_discovery->get_active()) {
        kill_cloud_discovery();
    }

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

    std::vector<int> vehicle_ids = get_active_vehicle_ids();
    if (vehicle_ids.size() > 0)
    {
        std::stringstream vehicle_ids_stream;
        for (size_t index = 0; index < vehicle_ids.size() - 1; ++index)
        {
            vehicle_ids_stream << vehicle_ids.at(index) << ",";
        }
        vehicle_ids_stream << vehicle_ids.at(vehicle_ids.size() - 1);

        //Get script info, generate command
        std::string script_path_string(script_path->get_text().c_str());
        std::string script_name_string(script_name->get_text().c_str());
        std::stringstream command;

        auto matlab_type_pos = script_name_string.rfind(".m");
        if (matlab_type_pos != std::string::npos)
        {
            script_name_string = script_name_string.substr(0, matlab_type_pos);

            //Case: Matlab script
            command 
            << "tmux new-session -d "
            << "-s \"hlc\" "
            << "$'source ~/dev/software/hlc/environment_variables.bash;"
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
            << " >stdout_hlc.txt 2>stderr_hlc.txt\"";
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
    std::stringstream command;
    command 
        << "tmux kill-session -t \"hlc\"";
    system(command.str().c_str());
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
    std::vector<int> vehicle_ids = get_active_vehicle_ids();
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
    //Generate command
    std::stringstream command;
    command 
        << "tmux kill-session -t \"middleware\"";

    //Execute command
    system(command.str().c_str());
}

void SetupViewUI::deploy_vehicles() {
    for (const int id : get_active_vehicle_ids())
    {
        deploy_vehicle(id);
    }
}

void SetupViewUI::deploy_vehicle(int id) {
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
    for (const int id : get_active_vehicle_ids())
    {
        kill_vehicle(id);
    } 
}

void SetupViewUI::kill_vehicle(int id) {
    std::stringstream command;
    command 
        << "tmux kill-session -t \"vehicle_" << id << "\"";
    system(command.str().c_str());
}

void SetupViewUI::deploy_ips() {
    if (!switch_simulated_time->get_active()) 
    {
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
}

void SetupViewUI::kill_ips() {
    if (!switch_simulated_time->get_active()) 
    {
        //Generate command
        std::stringstream command_ips;
        command_ips 
            << "tmux kill-session -t \"ips_pipeline\"";

        //Generate command
        std::stringstream command_basler;
        command_basler
            << "tmux kill-session -t \"ips_basler\"";

        //Execute command
        system(command_ips.str().c_str());
        system(command_basler.str().c_str());
    }
}

void SetupViewUI::deploy_cloud_discovery() {
    std::string command = "tmux new-session -d -s \"rticlouddiscoveryservice\" \"rticlouddiscoveryservice -transport 25598\"";
    system(command.c_str());
}

void SetupViewUI::kill_cloud_discovery() {
    std::string command = "tmux kill-session -t \"rticlouddiscoveryservice\"";
    system(command.c_str());
}

std::vector<int> SetupViewUI::get_active_vehicle_ids() {
    std::vector<int> active_vehicle_ids;

    if (toggle_vehicle_1->get_active())
        active_vehicle_ids.push_back(1);
    if (toggle_vehicle_2->get_active())
        active_vehicle_ids.push_back(2);
    if (toggle_vehicle_3->get_active())
        active_vehicle_ids.push_back(3);
    if (toggle_vehicle_4->get_active())
        active_vehicle_ids.push_back(4);
    if (toggle_vehicle_5->get_active())
        active_vehicle_ids.push_back(5);
    if (toggle_vehicle_6->get_active())
        active_vehicle_ids.push_back(6);

    return active_vehicle_ids;
}

void SetupViewUI::set_sensitive(bool is_sensitive) {
    script_path->set_sensitive(is_sensitive);
    script_name->set_sensitive(is_sensitive);
    toggle_vehicle_1->set_sensitive(is_sensitive);
    toggle_vehicle_2->set_sensitive(is_sensitive);
    toggle_vehicle_3->set_sensitive(is_sensitive);
    toggle_vehicle_4->set_sensitive(is_sensitive);
    toggle_vehicle_5->set_sensitive(is_sensitive);
    toggle_vehicle_6->set_sensitive(is_sensitive);
    button_select_all_vehicles->set_sensitive(is_sensitive);
    button_select_no_vehicles->set_sensitive(is_sensitive);
    switch_simulated_time->set_sensitive(is_sensitive);
    switch_launch_simulated_vehicles->set_sensitive(is_sensitive);
    switch_launch_cloud_discovery->set_sensitive(is_sensitive);
    switch_launch_ips->set_sensitive(is_sensitive);
    switch_launch_middleware->set_sensitive(is_sensitive);
}

void SetupViewUI::select_all_vehicles()
{
    toggle_vehicle_1->set_active(true);
    toggle_vehicle_2->set_active(true);
    toggle_vehicle_3->set_active(true);
    toggle_vehicle_4->set_active(true);
    toggle_vehicle_5->set_active(true);
    toggle_vehicle_6->set_active(true);
}

void SetupViewUI::select_no_vehicles()
{
    toggle_vehicle_1->set_active(false);
    toggle_vehicle_2->set_active(false);
    toggle_vehicle_3->set_active(false);
    toggle_vehicle_4->set_active(false);
    toggle_vehicle_5->set_active(false);
    toggle_vehicle_6->set_active(false);
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


Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}