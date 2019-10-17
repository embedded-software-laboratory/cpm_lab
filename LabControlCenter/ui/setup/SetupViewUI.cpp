#include "SetupViewUI.hpp"
#include <cstdlib>


SetupViewUI::SetupViewUI(int argc, char *argv[])
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

    //Extract relevant parameters from command line
    cmd_simulated_time = cpm::cmd_parameter_bool("simulated_time", false, argc, argv);
    cmd_domain_id = cpm::cmd_parameter_int("dds_domain", 0, argc, argv);
    cmd_dds_initial_peer = cpm::cmd_parameter_string("dds_initial_peer", "", argc, argv);
}

SetupViewUI::~SetupViewUI() {
    //TODO: Klappt nicht -> ergo auch bei deploy vorher clearen? (tmux kill-server)
    kill_deployed_applications();
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

}

void SetupViewUI::kill_hlc_scripts() {

}

void SetupViewUI::deploy_middleware() {

}

void SetupViewUI::kill_middleware() {

}

void SetupViewUI::deploy_vehicles() {
    std::string sim_time_string;
    if (switch_simulated_time->get_active())
    {
        sim_time_string = "true";
    }
    else 
    {
        sim_time_string = "false";
    }

    std::stringstream command;
    command 
        << "tmux new-session -d -s \"vehicle_1\" \"cd ~/dev/software/vehicle_raspberry_firmware/build_x64_sim;./vehicle_rpi_firmware --simulated_time="
        << sim_time_string
        << " --vehicle_id=1 --dds_domain="
        << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command << " --dds_initial_peer="
            << cmd_dds_initial_peer;
    }
    command << " >stdout_1.txt 2>stderr_1.txt\"";

        std::cout << command.str() << std::endl;
    // std::string cmd_out = execute_command(command.c_str());
    // std::cout << cmd_out << std::endl;
    system(command.str().c_str());
}

void SetupViewUI::kill_vehicles() {
    std::string command = "tmux kill-session -t \"vehicle_1\"";
    system(command.c_str());
}

void SetupViewUI::deploy_ips() {

}

void SetupViewUI::kill_ips() {

}

void SetupViewUI::deploy_cloud_discovery() {
    std::string command = "tmux new-session -d -s \"rticlouddiscoveryservice\" \"rticlouddiscoveryservice -transport 25598\"";
    system(command.c_str());
}

void SetupViewUI::kill_cloud_discovery() {
    std::string command = "tmux kill-session -t \"rticlouddiscoveryservice\"";
    system(command.c_str());
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