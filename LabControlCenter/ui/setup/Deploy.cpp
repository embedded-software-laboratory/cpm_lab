#include "Deploy.hpp"

Deploy::Deploy(unsigned int _cmd_domain_id, std::string _cmd_dds_initial_peer, std::function<void(uint8_t)> _stop_vehicle) :
    cmd_domain_id(_cmd_domain_id),
    cmd_dds_initial_peer(_cmd_dds_initial_peer),
    stop_vehicle(_stop_vehicle)
{

}

void Deploy::deploy_local_hlc(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params) 
{
    std::string sim_time_string = bool_to_string(use_simulated_time);

    //Check if old session already exists - if so, kill it
    kill_session("high_level_controller");

    if (active_vehicle_ids.size() > 0)
    {
        std::stringstream vehicle_ids_stream;
        for (size_t index = 0; index < active_vehicle_ids.size() - 1; ++index)
        {
            vehicle_ids_stream << active_vehicle_ids.at(index) << ",";
        }
        vehicle_ids_stream << active_vehicle_ids.at(active_vehicle_ids.size() - 1);

        //Get script info, generate command
        std::string script_path_string;
        std::string script_name_string;
        get_path_name(script_path, script_path_string, script_name_string);
        std::stringstream command;

        auto matlab_type_pos = script_name_string.rfind(".m");
        if (matlab_type_pos != std::string::npos)
        {
            script_name_string = script_name_string.substr(0, matlab_type_pos);

            //Case: Matlab script
            command 
            << "tmux new-session -d "
            << "-s \"high_level_controller\" "
            << "'. ~/dev/software/LabControlCenter/bash/environment_variables_local.bash;"
            << "matlab -logfile matlab.log"
            << " -sd \"" << script_path_string
            << "\" -batch \"" << script_name_string << "(" << script_params << (script_params.size() > 0 ? "," : "") << vehicle_ids_stream.str() << ")\""
            << " >~/dev/lcc_script_logs/stdout_hlc.txt 2>~/dev/lcc_script_logs/stderr_hlc.txt'";
        }
        else if (script_name_string.find(".") == std::string::npos)
        {
            //Case: Any executable 
            command 
            << "tmux new-session -d "
            << "-s \"high_level_controller\" "
            << "\". ~/dev/software/LabControlCenter/bash/environment_variables_local.bash;"
            << "cd " << script_path_string << ";./" << script_name_string
            << " --node_id=high_level_controller"
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids_stream.str()
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        command 
            << " " << script_params << " >~/dev/lcc_script_logs/stdout_hlc.txt 2>~/dev/lcc_script_logs/stderr_hlc.txt\"";
        }
        else 
        {
            std::cout << "Warning: Could not run unknown script: Neither matlab nor C++ executable" << std::endl;
            return;
        }

        std::cout << command.str() << std::endl;

        //Execute command
        system(command.str().c_str());

        //Check if old session already exists - if so, kill it
        kill_session("middleware");

        //Generate command
        std::stringstream middleware_command;
        middleware_command 
            << "tmux new-session -d "
            << "-s \"middleware\" "
            << "\". ~/dev/software/LabControlCenter/bash/environment_variables_local.bash;cd ~/dev/software/middleware/build/;./middleware"
            << " --node_id=middleware"
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids_stream.str()
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            middleware_command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        middleware_command 
            << " >~/dev/lcc_script_logs/stdout_middleware.txt 2>~/dev/lcc_script_logs/stderr_middleware.txt\"";

        //Execute command
        system(middleware_command.str().c_str());
    }
}

void Deploy::kill_local_hlc() 
{
    kill_session("high_level_controller");
    kill_session("middleware");
}

void Deploy::deploy_sim_vehicles(std::vector<unsigned int> simulated_vehicle_ids, bool use_simulated_time) 
{
    for (const unsigned int id : simulated_vehicle_ids)
    {
        deploy_sim_vehicle(id, use_simulated_time);
    }
}

void Deploy::deploy_sim_vehicle(unsigned int id, bool use_simulated_time) 
{
    std::string sim_time_string = bool_to_string(use_simulated_time);

    std::stringstream session_name;
    session_name << "vehicle_" << id;

    //Check if old session already exists - if so, kill it
    kill_session(session_name.str());

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
        << " >~/dev/lcc_script_logs/stdout_vehicle" << id << ".txt 2>~/dev/lcc_script_logs/stderr_vehicle" << id << ".txt\"";

    //Execute command
    //TODO: (nach Besprechung, ob das so okay ist) - nutze fork/execl/kill um das abbrechen zu können (merke PIDs, breche bei Kill ab)
    system(command.str().c_str());
}

void Deploy::kill_vehicles(std::vector<unsigned int> simulated_vehicle_ids, std::vector<unsigned int> active_vehicle_ids) 
{
    for (const unsigned int id : simulated_vehicle_ids)
    {
        kill_vehicle(id);
    }

    //Also make all vehicles stop immediately, so that they do not continue to drive for a while   
    for (const auto id : active_vehicle_ids)
    {
        stop_vehicle(static_cast<uint8_t>(id));
    }
}

void Deploy::kill_vehicle(unsigned int id) 
{
    std::stringstream vehicle_id;
    vehicle_id << "vehicle_" << id;
    
    kill_session(vehicle_id.str());
}

bool Deploy::deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids, bool use_simulated_time, std::string script_path, std::string script_params, unsigned int timeout_seconds, std::function<bool()> is_online) 
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

    std::string sim_time_string = bool_to_string(use_simulated_time);

    //Default arguments + user arguments
    std::stringstream script_argument_stream;
    std::stringstream middleware_argument_stream;
    auto matlab_type_pos = script_path.rfind(".m");
    if (matlab_type_pos != std::string::npos)
    {
        //Case: Matlab script
        script_argument_stream << script_params << (script_params.size() > 0 ? "," : "") << vehicle_ids;
    }
    else if (script_path.find(".") == std::string::npos)
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
    middleware_argument_stream 
            << " " << script_params;

    //Copy all relevant data over to the remote system
    std::stringstream copy_command;
    //Okay, do this using a template script instead, I think that's better in this case
    copy_command << "~/dev/software/LabControlCenter/bash/copy_to_remote.bash --ip=" << ip_stream.str() 
        << " --script_path=" << script_path 
        << " --script_arguments='" << script_argument_stream.str() << "'"
        << " --middleware_arguments='" << middleware_argument_stream.str() << "'";

    //Spawn and manage new process
    return spawn_and_manage_process(copy_command.str().c_str(), timeout_seconds, is_online);
}

bool Deploy::kill_remote_hlc(unsigned int hlc_id, unsigned int timeout_seconds, std::function<bool()> is_online) 
{
    //Get the IP address from the current id
    std::stringstream ip_stream;
    ip_stream << "192.168.1.2";
    if (hlc_id < 10)
    {
        ip_stream << "0";
    }
    ip_stream << hlc_id;

    //Kill the middleware and script tmux sessions running on the remote system
    std::stringstream kill_command;
    kill_command << "~/dev/software/LabControlCenter/bash/remote_kill.bash --ip=" << ip_stream.str();

    //Spawn and manage new process
    return spawn_and_manage_process(kill_command.str().c_str(), timeout_seconds, is_online);
}

void Deploy::deploy_ips() 
{
    //Check if old session already exists - if so, kill it
    kill_session("ips_pipeline");

    //Generate command
    std::stringstream command_ips;
    command_ips 
        << "tmux new-session -d "
        << "-s \"ips_pipeline\" "
        << "\"cd ~/dev/software/indoor_positioning_system/;./build/ips_pipeline "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_ips 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_ips 
        << " >~/dev/lcc_script_logs/stdout_ips.txt 2>~/dev/lcc_script_logs/stderr_ips.txt\"";

    //Kill previous ips basler session if it still exists
    kill_session("ips_basler");

    //Generate command
    std::stringstream command_basler;
    command_basler 
        << "tmux new-session -d "
        << "-s \"ips_basler\" "
        << "\"cd ~/dev/software/indoor_positioning_system/;./build/BaslerLedDetection "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_basler 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_basler 
        << " >~/dev/lcc_script_logs/stdout_basler.txt 2>~/dev/lcc_script_logs/stderr_basler.txt\"";

    //Execute command
    system(command_ips.str().c_str());
    system(command_basler.str().c_str());
}

void Deploy::kill_ips() {
    kill_session("ips_pipeline");
    kill_session("ips_basler");
}



void Deploy::deploy_recording() 
{
    //if old session already exists, kill it
    kill_session(recording_session);

    // Update recording config
    std::string config_path_in = std::getenv("HOME");
    config_path_in.append("/dev/software/dds_record/rti_recording_config_template.xml");
    std::ifstream xml_config_template(config_path_in);
    
    std::string xml_config_str;
    {
        std::stringstream buffer;
        if (xml_config_template.is_open()) {
            buffer << xml_config_template.rdbuf();
        }
        xml_config_template.close();
        xml_config_str = buffer.str();
    }

    xml_config_str = std::regex_replace(
        xml_config_str,
        std::regex("TEMPLATE_NDDSHOME"),
        std::getenv("NDDSHOME")
    );
    auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); 
    std::ostringstream timenow_ss;
    timenow_ss << std::put_time(std::localtime(&timenow), "%Y_%m_%d_%H_%M_%S");
    xml_config_str = std::regex_replace(
        xml_config_str,
        std::regex("TEMPLATE_RECORDING_FOLDER"),
        timenow_ss.str()
    );
    xml_config_str = std::regex_replace(
        xml_config_str,
        std::regex("TEMPLATE_DOMAIN_ID"),
        std::to_string(cmd_domain_id)
    );
    xml_config_str = std::regex_replace(
        xml_config_str,
        std::regex("TEMPLATE_DISCOVERY_URL"),
        cmd_dds_initial_peer
    );

    std::string config_path_out = "/tmp/rti_recording_config.xml";
    std::ofstream xml_config(config_path_out);
    xml_config << xml_config_str;

    xml_config.close();
    
    //Generate command
    std::stringstream command;
    command 
        << "tmux new-session -d "
        << "-s \"" << recording_session << "\" "
        << "rtirecordingservice "
        << "-cfgFile " << config_path_out << " "
        << "-cfgName cpm_recorder";
    
    std::cout << command.str() << std::endl;
    //Execute command
    system(command.str().c_str());
}

void Deploy::kill_recording() 
{
    kill_session(recording_session);
}



bool Deploy::session_exists(std::string session_id)
{
    std::string running_sessions = execute_command("tmux ls");
    session_id += ":";
    return running_sessions.find(session_id) != std::string::npos;
}

void Deploy::kill_session(std::string session_id)
{
    if (session_exists(session_id))
    {
        std::stringstream command;
        command 
            << "tmux kill-session -t \"" << session_id << "\"";

        //Execute command
        system(command.str().c_str());
    }
}

std::string Deploy::execute_command(const char* cmd) 
{
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

void Deploy::get_path_name(std::string& in, std::string& out_path, std::string& out_name)
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

std::string Deploy::bool_to_string(bool var)
{
    if (var)
    {
        return "true";
    }
    else 
    {
        return "false";
    }
}

void Deploy::create_log_folder(std::string name)
{
    //Generate command
    std::stringstream command_folder;
    command_folder 
        << "rm -rf ~/dev/" << name << ";"
        << "mkdir -p ~/dev/" << name;

    //Execute command
    system(command_folder.str().c_str());
}

bool Deploy::spawn_and_manage_process(const char* cmd, unsigned int timeout_seconds, std::function<bool()> is_online)
{
    std::cout << "Executing" << std::endl;
    //Spawn and manage new process
    int process_id = execute_command_get_pid(cmd);
    auto start_time = std::chrono::high_resolution_clock::now();

    //Regularly check status during execution until timeout - exit early if everything worked as planned, else run until error / timeout and return error
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() < static_cast<int64_t>(timeout_seconds))
    {
        std::cout << "Waiting" << std::endl;
        //Check current program state
        PROCESS_STATE state = get_child_process_state(process_id);

        if (state == PROCESS_STATE::DONE)
        {
            return true;
        }
        else if (state == PROCESS_STATE::ERROR)
        {
            kill_process(process_id);
            return false;
        }
        else if (! is_online())
        {
            //The HLC is no longer online, so abort
            kill_process(process_id);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //Now kill the process, as it has not yet finished its execution
    std::cout << "Killing" << std::endl;
    kill_process(process_id);
    return false;
}

int Deploy::execute_command_get_pid(const char* cmd)
{
    int process_id = fork();
    if (process_id == 0)
    {
        //Tell the child to set its group process ID to its process ID, or else things like kill(-pid) to kill a ping-while-loop won't work
        setpgid(0, 0);
        
        //Actions to take within the new child process
        //ACHTUNG: NUTZE chmod u+x für die Files, sonst: permission denied
        execl("/bin/sh", "bash", "-c", cmd, NULL);

        //Error if execlp returns
        std::cerr << "Exec error: " << errno << "!" << std::endl;

        exit(1);
    }
    else if (process_id > 0)
    {
        //We are in the parent process and got the child's PID
        return process_id;
    }
    else 
    {
        //We could not spawn a new process - usually, the program should not just break at this point, unless that behaviour is desired
        //TODO: Change behaviour
        std::cerr << "There was an error during the creation of a child process for program execution" << std::endl;
        exit(1);
    }
}

Deploy::PROCESS_STATE Deploy::get_child_process_state(int process_id)
{
    int process_status;
    pid_t result = waitpid(process_id, &process_status, WNOHANG);

    if (result == 0)
    {
        return PROCESS_STATE::RUNNING;
    }
    else if (result == -1)
    {
        return PROCESS_STATE::ERROR;
    }
    else
    {
        return PROCESS_STATE::DONE;
    }
    
}

void Deploy::kill_process(int process_id)
{
    //Tell the process to terminate - this way, it can terminate gracefully
    //We mostly use bash, were whole process groups might be created - to kill those, we need the negative id
    if (process_id > 0)
    {
        process_id *= (-1);
    }

    kill(process_id, SIGTERM);

    //Wait for the process to terminate
    std::this_thread::sleep_for(std::chrono::seconds(3));

    //Check if the process terminated as desired
    PROCESS_STATE state = get_child_process_state(process_id);

    //If the process has not yet terminated, force a termination and clean up
    if (state != PROCESS_STATE::DONE)
    {
        kill(process_id, SIGKILL);
        int status;
        waitpid(process_id, &status, 0); //0 -> no flags here
    }
}