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

#include "Deploy.hpp"

Deploy::Deploy(unsigned int _cmd_domain_id, std::string _cmd_dds_initial_peer, std::function<void(uint8_t)> _stop_vehicle) :
    cmd_domain_id(_cmd_domain_id),
    cmd_dds_initial_peer(_cmd_dds_initial_peer),
    stop_vehicle(_stop_vehicle)
{

}

Deploy::~Deploy()
{
    std::lock_guard<std::mutex> lock(vehicle_reboot_threads_mutex);
    for (auto& thread : vehicle_reboot_threads)
    {
        if (thread.second.joinable())
        {
            thread.second.join();
        }
        else 
        {
            std::cerr << "Warning: Shutting down with reboot thread that cannot be joined" << std::endl;
        }
    }
    vehicle_reboot_threads.clear();
}

void Deploy::deploy_local_hlc(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params) 
{
    std::string sim_time_string = bool_to_string(use_simulated_time);

    //Check if old session already exists - if so, kill it
    kill_session(hlc_session);

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
            << "-s \"" << hlc_session << "\" "
            << "'. ~/dev/software/lab_control_center/bash/environment_variables_local.bash;"
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
            << "-s \"" << hlc_session << "\" "
            << "\". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;"
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
        kill_session(middleware_session);

        //Generate command
        std::stringstream middleware_command;
        middleware_command 
            << "tmux new-session -d "
            << "-s \"" << middleware_session << "\" "
            << "\". ~/dev/software/lab_control_center/bash/environment_variables_local.bash;cd ~/dev/software/middleware/build/;./middleware"
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
    kill_session(hlc_session);
    kill_session(middleware_session);
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
        << "\"cd ~/dev/software/mid_level_controller/build_x64_sim;./vehicle_rpi_firmware "
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

void Deploy::stop_vehicles(std::vector<unsigned int> vehicle_ids)
{
    // make all vehicles stop immediately, so that they do not continue to drive for a while   
    for (const auto id : vehicle_ids)
    {
        stop_vehicle(static_cast<uint8_t>(id));
    }
}


void Deploy::kill_sim_vehicles(std::vector<unsigned int> simulated_vehicle_ids) 
{
    for (const unsigned int id : simulated_vehicle_ids)
    {
        kill_sim_vehicle(id);
    }
}

void Deploy::kill_sim_vehicle(unsigned int id) 
{
    std::stringstream vehicle_id;
    vehicle_id << "vehicle_" << id;
    
    kill_session(vehicle_id.str());
}

void Deploy::reboot_real_vehicle(unsigned int vehicle_id, unsigned int timeout_seconds) 
{
    //Kill old reboot threads that are done before adding a new one
    join_finished_reboot_threads();

    //Get the IP address from the current vehicle_id (192.168.1.1XX)
    std::stringstream ip_stream;
    ip_stream << "192.168.1.1";
    if (vehicle_id < 10)
    {
        ip_stream << "0";
    }
    ip_stream << vehicle_id;
    std::string ip = ip_stream.str();

    std::lock_guard<std::mutex> lock(vehicle_reboot_threads_mutex);
    //Only create a reboot thread if no such thread already exists
    if (vehicle_reboot_threads.find(vehicle_id) == vehicle_reboot_threads.end())
    {
        std::unique_lock<std::mutex> lock(reboot_done_mutex);
        reboot_thread_done[vehicle_id] = false;
        lock.unlock();

        vehicle_reboot_threads[vehicle_id] = std::thread(
            [this, vehicle_id, ip, timeout_seconds] () {
                //Create and send the vehicle kill command via SSH (Open pw is cpmcpmcpm, can be in Git)
                //We want a too long connect timeout to be able to detect connection errors (if it takes too long, assume that connection was not possible)
                std::stringstream command_kill_real_vehicle;
                command_kill_real_vehicle 
                    << "sshpass -p cpmcpmcpm ssh -o StrictHostKeyChecking=no -o ConnectTimeout=" << (timeout_seconds + 10) << " -t pi@" << ip << " \"sudo reboot now\"";
                bool msg_success = spawn_and_manage_process(command_kill_real_vehicle.str().c_str(), timeout_seconds, 
                    [] () { 
                        //Ignore the check if the vehicle is still online
                        return true; 
                    }
                );

                if(!msg_success)
                {
                    cpm::Logging::Instance().write(
                        2, 
                        "Could not reboot vehicle %u (timeout or connection lost)", 
                        vehicle_id
                    );
                }

                std::lock_guard<std::mutex> lock(reboot_done_mutex);
                reboot_thread_done[vehicle_id] = true;
            }
        );
    }
}

void Deploy::join_finished_reboot_threads()
{
    std::lock_guard<std::mutex> lock(vehicle_reboot_threads_mutex);
    for (auto thread_ptr = vehicle_reboot_threads.begin(); thread_ptr != vehicle_reboot_threads.end(); /*Do not increment here*/)
    {
        std::lock_guard<std::mutex> lock(reboot_done_mutex);
        if (reboot_thread_done[thread_ptr->first] && thread_ptr->second.joinable())
        {
            thread_ptr->second.join();
            thread_ptr = vehicle_reboot_threads.erase(thread_ptr);
        }
        else
        {
            ++thread_ptr;
        }
    }
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
    copy_command << "~/dev/software/lab_control_center/bash/copy_to_remote.bash --ip=" << ip_stream.str() 
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
    kill_command << "~/dev/software/lab_control_center/bash/remote_kill.bash --ip=" << ip_stream.str();

    //Spawn and manage new process
    return spawn_and_manage_process(kill_command.str().c_str(), timeout_seconds, is_online);
}

void Deploy::deploy_ips() 
{
    //Check if old session already exists - if so, kill it
    kill_session(ips_session);

    //Generate command
    std::stringstream command_ips;
    command_ips 
        << "tmux new-session -d "
        << "-s \"" << ips_session << "\" "
        << "\"cd ~/dev/software/indoor_positioning_system/;./build/ips_pipeline "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_ips 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_ips 
        << " >~/dev/lcc_script_logs/stdout_ips.txt 2>~/dev/lcc_script_logs/stderr_ips.txt\"";

    //Kill previous ips basler session if it still exists
    kill_session(basler_session);

    //Generate command
    std::stringstream command_basler;
    command_basler 
        << "tmux new-session -d "
        << "-s \"" << basler_session << "\" "
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
    kill_session(ips_session);
    kill_session(basler_session);
}



void Deploy::deploy_recording() 
{
    //if old session already exists, kill it
    kill_session(recording_session);

    // Update recording config
    std::string config_path_in = std::getenv("HOME");
    config_path_in.append("/dev/software/lab_control_center/recording/rti_recording_config_template.xml");
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
    // extract ip from initial peer
    std::smatch ip_matched;
    std::regex ip_regex ("\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}");
    std::string ip_string;
    bool is_ip_contained = std::regex_search (cmd_dds_initial_peer,ip_matched,ip_regex);
    assert(is_ip_contained);
    ip_string = ip_matched.str(0);
    xml_config_str = std::regex_replace(
        xml_config_str,
        std::regex("TEMPLATE_IP"),
        ip_string
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

std::vector<std::string> Deploy::check_for_crashes(bool deploy_remote, bool has_local_hlc, bool lab_mode_on, bool check_for_recording)
{
    std::vector<std::string> crashed_participants;
    if (!(deploy_remote) || has_local_hlc)
    {
        if(! session_exists(hlc_session)) crashed_participants.push_back("HLC");
        if(! session_exists(middleware_session)) crashed_participants.push_back("Middleware");
    }
    if (lab_mode_on)
    {
        if(! session_exists(ips_session)) crashed_participants.push_back("IPS");
        if(! session_exists(basler_session)) crashed_participants.push_back("Basler LED detection");
    }
    if (check_for_recording)
    {
        if(! session_exists(recording_session)) crashed_participants.push_back("Recording");
    }

    return crashed_participants;
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
    cpm::Logging::Instance().write(3, "Executing '%s'", cmd);

    //Spawn and manage new process
    int process_id = execute_command_get_pid(cmd);
    auto start_time = std::chrono::high_resolution_clock::now();

    auto time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    //Regularly check status during execution until timeout - exit early if everything worked as planned, else run until error / timeout and return error
    while (time_passed_ms < static_cast<int64_t>(timeout_seconds) * 1000)
    {
        //Check current program state
        PROCESS_STATE state = get_child_process_state(process_id);

        if (state == PROCESS_STATE::DONE)
        {
            cpm::Logging::Instance().write(3, "Success: execution of '%s'", cmd);
            return true;
        }
        else if (state == PROCESS_STATE::ERROR)
        {
            kill_process(process_id);
            cpm::Logging::Instance().write(2, "Error state in execution of '%s'", cmd);
            return false;
        }
        else if (! is_online())
        {
            //The HLC is no longer online, so abort
            kill_process(process_id);
            cpm::Logging::Instance().write(2, "No longer online - stopped execution of '%s'", cmd);
            return false;
        }

        //Use longer sleep time until short before end of timeout
        time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        auto remaining_time = static_cast<int64_t>(timeout_seconds) * 1000 - time_passed_ms;
        if (remaining_time > 1000)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else if (remaining_time > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(remaining_time));
        }
        
        time_passed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
    }

    //Now kill the process, as it has not yet finished its execution
    //std::cout << "Killing" << std::endl;
    kill_process(process_id);
    cpm::Logging::Instance().write(2, "Could not execute in time: '%s'", cmd);
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
        cpm::Logging::Instance().write(1, "Execl error in Deploy class: %s, for execution of '%s'", std::strerror(errno), cmd);

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
        cpm::Logging::Instance().write(1, "Error in Deploy class: Could not create child process for '%s'", cmd);
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