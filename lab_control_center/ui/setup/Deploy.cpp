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

Deploy::Deploy(
    unsigned int _cmd_domain_id, 
    std::string _cmd_dds_initial_peer, 
    std::function<void(uint8_t)> _stop_vehicle, 
    std::shared_ptr<ProgramExecutor> _program_executor,
    std::string _absolute_exec_path
) :
    cmd_domain_id(_cmd_domain_id),
    cmd_dds_initial_peer(_cmd_dds_initial_peer),
    stop_vehicle(_stop_vehicle),
    program_executor(_program_executor)
{
    //Construct the path to the folder by erasing all parts to the executable that are obsolete
    //Executable path: .../software/lab_control_center/build/lab_control_center
    //-> Remove everything up to the third-last slash
    software_folder_path = _absolute_exec_path;
    for (int i = 0; i < 3; ++i)
    {
        auto last_slash = software_folder_path.find_last_of('/');
        if (last_slash != std::string::npos)
        {
            software_folder_path = software_folder_path.substr(0, last_slash);
        }
    }

    software_top_folder_path = software_folder_path;
    auto last_slash = software_top_folder_path.find_last_of('/');
    if (last_slash != std::string::npos)
    {
        software_top_folder_path = software_top_folder_path.substr(0, last_slash);
    }    

    //Create the log folder for the first time (or delete an outdated version of it)
    //Gets re-created with every deploy in the Setup class
    create_log_folder("lcc_script_logs");
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
        //Get list of vehicle IDs as string to pass as argument to the scripts
        std::stringstream vehicle_ids_stream;
        for (size_t index = 0; index < active_vehicle_ids.size() - 1; ++index)
        {
            vehicle_ids_stream << active_vehicle_ids.at(index) << ",";
        }
        vehicle_ids_stream << active_vehicle_ids.at(active_vehicle_ids.size() - 1);

        //Check if an empty string was passed - in this case, we only want to start the middleware
        bool start_middleware_without_hlc = (script_path.size() == 0);

        if (! start_middleware_without_hlc)
        {
            //Start local HLC
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
                << "'. " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;"
                << "matlab -logfile matlab.log"
                << " -sd \"" << script_path_string
                << "\" -batch \"" << script_name_string << "(" << script_params << (script_params.size() > 0 ? "," : "") << vehicle_ids_stream.str() << ")\""
                << " >" << software_top_folder_path << "/lcc_script_logs/stdout_hlc.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_hlc.txt'";
            }
            else if (script_name_string.find(".") == std::string::npos)
            {
                //Case: Any executable 
                command 
                << "tmux new-session -d "
                << "-s \"" << hlc_session << "\" "
                << "\". " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;"
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
                << " " << script_params << " >" << software_top_folder_path << "/lcc_script_logs/stdout_hlc.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_hlc.txt\"";
            }
            else 
            {
                cpm::Logging::Instance().write(
                        1, 
                        "%s",
                        "Warning: Could not run unknown script: Neither matlab nor C++ executable"
                );
                
                return;
            }

            //std::cout << command.str() << std::endl;

            //Execute command
            program_executor->execute_command(command.str());
        }

        //Check if old session already exists - if so, kill it
        kill_session(middleware_session);

        //Generate command to start the middleware
        std::stringstream middleware_command;
        middleware_command 
            << "tmux new-session -d "
            << "-s \"" << middleware_session << "\" "
            << "\". " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;cd " << software_folder_path << "/middleware/build/;./middleware"
            << " --node_id=middleware"
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << vehicle_ids_stream.str()
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            middleware_command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        middleware_command 
            << " >" << software_top_folder_path << "/lcc_script_logs/stdout_middleware.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_middleware.txt\"";

        //Execute command
        program_executor->execute_command(middleware_command.str());
    }
}

void Deploy::kill_local_hlc() 
{
    kill_session(hlc_session);
    kill_session(middleware_session);
}

void Deploy::deploy_separate_local_hlcs(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params) 
{
    std::string sim_time_string = bool_to_string(use_simulated_time);

    //Check if old session already exists - if so, kill it
    kill_separate_local_hlcs();

    for ( unsigned int vehicle_id : active_vehicle_ids ) {

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
            << "-s \"high_level_controller_"
            << std::to_string(vehicle_id) 
            << "\" "
            << "'. " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;"
            << "matlab -logfile matlab.log"
            << " -sd \"" << script_path_string
            << "\" -batch \"" << script_name_string << "(" << script_params << (script_params.size() > 0 ? "," : "") << std::to_string(vehicle_id) << ")\""
            << " >" << software_top_folder_path << "/lcc_script_logs/stdout_hlc.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_hlc.txt'";
        }
        else if (script_name_string.find(".") == std::string::npos)
        {
            //Case: Any executable 
            command 
            << "tmux new-session -d "
            << "-s \"high_level_controller_"
            << std::to_string(vehicle_id) << "\" "
            << "\". " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;"
            << "cd " << script_path_string << ";./" << script_name_string
            << " --node_id=high_level_controller_"
            << std::to_string(vehicle_id) 
            << " --simulated_time=" << sim_time_string
            << " --vehicle_ids=" << std::to_string(vehicle_id)
            << " --dds_domain=" << cmd_domain_id;
        if (cmd_dds_initial_peer.size() > 0) {
            command 
                << " --dds_initial_peer=" << cmd_dds_initial_peer;
        }
        command 
            << " " << script_params << " >" << software_top_folder_path << "/lcc_script_logs/stdout_hlc"
            << std::to_string(vehicle_id) 
            << ".txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_hlc"
            << std::to_string(vehicle_id) 
            << ".txt\"";
        }
        else 
        {
            cpm::Logging::Instance().write(
                1, 
                "%s",
                "Warning: Could not run unknown script: Neither matlab nor C++ executable"
            );
            return;
        }

        //std::cout << command.str() << std::endl;

        // Debugging only
        //std::string test_string = command.str();

        //Document, that we started this HLC
        deployed_local_hlcs.push_back(vehicle_id);

        //Execute command
        program_executor->execute_command(command.str());
    }

    //Check if old session already exists - if so, kill it
    kill_session("middleware");

    std::stringstream vehicle_ids_stream;
    for (size_t index = 0; index < active_vehicle_ids.size() - 1; ++index)
    {
        vehicle_ids_stream << active_vehicle_ids.at(index) << ",";
    }
    vehicle_ids_stream << active_vehicle_ids.at(active_vehicle_ids.size() - 1);

    //Generate command
    std::stringstream middleware_command;
    middleware_command 
        << "tmux new-session -d "
        << "-s \"middleware\" "
        << "\". " << software_folder_path << "/lab_control_center/bash/environment_variables_local.bash;cd " << software_folder_path << "/middleware/build/;./middleware"
        << " --node_id=middleware"
        << " --simulated_time=" << sim_time_string
        << " --vehicle_ids=" << vehicle_ids_stream.str()
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        middleware_command 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    middleware_command 
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_middleware.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_middleware.txt\"";

    //Execute command
    program_executor->execute_command(middleware_command.str());
}

void Deploy::kill_separate_local_hlcs() 
{
    for( unsigned int hlc : deployed_local_hlcs ) {
        std::string session_name = "high_level_controller_";
        session_name += std::to_string(hlc);
        kill_session(session_name);
    }
    kill_session("middleware");
    deployed_local_hlcs.clear();
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
        << "\"cd " << software_folder_path << "/mid_level_controller/build_x64_sim;./vehicle_rpi_firmware "
        << "--simulated_time=" << sim_time_string
        << " --vehicle_id=" << id
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command 
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_vehicle" << id << ".txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_vehicle" << id << ".txt\"";

    //Execute command
    program_executor->execute_command(command.str());
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
    join_finished_vehicle_reboot_threads();

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
        std::unique_lock<std::mutex> lock(vehicle_reboot_done_mutex);
        vehicle_reboot_thread_done[vehicle_id] = false;
        lock.unlock();

        vehicle_reboot_threads[vehicle_id] = std::thread(
            [this, vehicle_id, ip, timeout_seconds] () {
                //Create and send the vehicle kill command via SSH (Open pw is cpmcpmcpm, can be in Git)
                //We want a too long connect timeout to be able to detect connection errors (if it takes too long, assume that connection was not possible)
                std::stringstream command_kill_real_vehicle;
                command_kill_real_vehicle 
                    << "sshpass -p cpmcpmcpm ssh -o StrictHostKeyChecking=no -o ConnectTimeout=" << (timeout_seconds + 10) << " -t pi@" << ip << " \"sudo reboot now\""
                    << " >" << software_top_folder_path << "/lcc_script_logs/stdout_vehicle_reboot.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_vehicle_reboot.txt";
                bool msg_success = program_executor->execute_command(command_kill_real_vehicle.str().c_str(), timeout_seconds);

                if(!msg_success)
                {
                    cpm::Logging::Instance().write(
                        2, 
                        "Could not reboot vehicle %u (timeout or connection lost)", 
                        vehicle_id
                    );
                }

                std::lock_guard<std::mutex> lock(vehicle_reboot_done_mutex);
                vehicle_reboot_thread_done[vehicle_id] = true;
            }
        );
    }
}

void Deploy::reboot_hlcs(std::vector<uint8_t> hlc_ids, unsigned int timeout_seconds) 
{
    //Kill old reboot threads that are done before adding a new one
    join_finished_hlc_reboot_threads();

    for (auto hlc_id_uint8_t : hlc_ids)
    {
        //Prevents conversion errors to string, because uint8_t tends to get interpreted as a character
        unsigned int hlc_id = static_cast<unsigned int>(hlc_id_uint8_t);

        //Get the IP address from the current id (192.168.1.2XX)
        std::stringstream ip_stream;
        ip_stream << "192.168.1.2";
        if (hlc_id < 10)
        {
            ip_stream << "0";
        }
        ip_stream << hlc_id;
        std::string ip = ip_stream.str();

        std::lock_guard<std::mutex> lock(hlc_reboot_threads_mutex);
        //Only create a reboot thread if no such thread already exists
        if (hlc_reboot_threads.find(hlc_id) == hlc_reboot_threads.end())
        {
            std::unique_lock<std::mutex> lock(hlc_reboot_done_mutex);
            hlc_reboot_thread_done[hlc_id] = false;
            lock.unlock();

            hlc_reboot_threads[hlc_id] = std::thread(
                [this, hlc_id, ip, timeout_seconds] () {
                    //Create and send the hlc reboot command
                    //We want a too long connect timeout to be able to detect connection errors (if it takes too long, assume that connection was not possible)
                    std::stringstream command_reboot_hlc;
                    command_reboot_hlc 
                        << "sshpass ssh -o ConnectTimeout=" << (timeout_seconds + 10) << " -t guest@" << ip << " \"sudo reboot\""
                        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_hlc_reboot.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_hlc_reboot.txt";
                    bool msg_success = program_executor->execute_command(command_reboot_hlc.str().c_str(), timeout_seconds);

                    if(!msg_success)
                    {
                        cpm::Logging::Instance().write(
                            2, 
                            "Could not reboot HLC %u (timeout or connection lost)", 
                            hlc_id
                        );
                    }

                    std::lock_guard<std::mutex> lock(hlc_reboot_done_mutex);
                    hlc_reboot_thread_done[hlc_id] = true;
                }
            );
        }
    }
}

void Deploy::join_finished_vehicle_reboot_threads()
{
    std::lock_guard<std::mutex> lock(vehicle_reboot_threads_mutex);
    for (auto thread_ptr = vehicle_reboot_threads.begin(); thread_ptr != vehicle_reboot_threads.end(); /*Do not increment here*/)
    {
        std::lock_guard<std::mutex> lock(vehicle_reboot_done_mutex);
        if (vehicle_reboot_thread_done[thread_ptr->first] && thread_ptr->second.joinable())
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

void Deploy::join_finished_hlc_reboot_threads()
{
    std::lock_guard<std::mutex> lock(hlc_reboot_threads_mutex);
    for (auto thread_ptr = hlc_reboot_threads.begin(); thread_ptr != hlc_reboot_threads.end(); /*Do not increment here*/)
    {
        std::lock_guard<std::mutex> lock(vehicle_reboot_done_mutex);
        if (hlc_reboot_thread_done[thread_ptr->first] && thread_ptr->second.joinable())
        {
            thread_ptr->second.join();
            thread_ptr = hlc_reboot_threads.erase(thread_ptr);
        }
        else
        {
            ++thread_ptr;
        }
    }
}

bool Deploy::deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids, bool use_simulated_time, std::string script_path, std::string script_params, unsigned int timeout_seconds) 
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
    copy_command << software_folder_path << "/lab_control_center/bash/copy_to_remote.bash --ip=" << ip_stream.str() 
        << " --script_path=" << script_path 
        << " --script_arguments='" << script_argument_stream.str() << "'"
        << " --middleware_arguments='" << middleware_argument_stream.str() << "'"
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_remote_hlc_deploy.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_remote_hlc_deploy.txt";

    //Spawn and manage new process
    return program_executor->execute_command(copy_command.str().c_str(), timeout_seconds);
}

bool Deploy::kill_remote_hlc(unsigned int hlc_id, unsigned int timeout_seconds) 
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
    kill_command << software_folder_path << "/lab_control_center/bash/remote_kill.bash --ip=" << ip_stream.str()
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_remote_hlc_kill.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_remote_hlc_kill.txt";

    //Spawn and manage new process
    return program_executor->execute_command(kill_command.str().c_str(), timeout_seconds);
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
        << "\"cd " << software_folder_path << "/indoor_positioning_system/;./build/ips_pipeline "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_ips 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_ips 
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_ips.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_ips.txt\"";

    //Kill previous ips basler session if it still exists
    kill_session(basler_session);

    //Generate command
    std::stringstream command_basler;
    command_basler 
        << "tmux new-session -d "
        << "-s \"" << basler_session << "\" "
        << "\"cd " << software_folder_path << "/indoor_positioning_system/;./build/BaslerLedDetection "
        << " --dds_domain=" << cmd_domain_id;
    if (cmd_dds_initial_peer.size() > 0) {
        command_basler 
            << " --dds_initial_peer=" << cmd_dds_initial_peer;
    }
    command_basler 
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_basler.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_basler.txt\"";

    //Execute command
    program_executor->execute_command(command_ips.str());
    program_executor->execute_command(command_basler.str());
}

void Deploy::kill_ips() {
    kill_session(ips_session);
    kill_session(basler_session);
}


void Deploy::deploy_labcam(std::string path, std::string file_name){
    //Check if old session already exists - if so, kill it
    kill_session(labcam_session);

    //Generate command
    std::stringstream command;
    command
        << "tmux new-session -d "
        << "-s \"" << labcam_session << "\" "
        << "\"cd " << software_folder_path << "/lab_control_center/build/labcam;./labcam_recorder "
        << " --path=" << path
        << " --file_name=" << file_name
        << " >" << software_top_folder_path << "/lcc_script_logs/stdout_labcam.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_labcam.txt\"";
    
    //Execute command
    program_executor->execute_command(command.str().c_str());
}


void Deploy::kill_labcam() {
    kill_session(labcam_session);
}



void Deploy::deploy_recording() 
{
    //if old session already exists, kill it
    kill_session(recording_session);

    // Update recording config
    std::string config_path_in = software_folder_path;
    config_path_in.append("/lab_control_center/recording/rti_recording_config_template.xml");
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
        << "-cfgName cpm_recorder" << " "
        << ">" << software_top_folder_path << "/lcc_script_logs/stdout_recording.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_recording.txt";
    
    //std::cout << command.str() << std::endl;
    //Execute command
    program_executor->execute_command(command.str());
}

void Deploy::kill_recording() 
{
    kill_session(recording_session);
}



bool Deploy::session_exists(std::string session_id)
{
    std::string running_sessions = program_executor->get_command_output("tmux ls");
    session_id += ":";

    if (running_sessions.find("ERROR") != std::string::npos)
    {
        cpm::Logging::Instance().write(
            1, 
            "%s",
            "Could not determine running sessions, assuming no crash..."
        );
        return true;
    }

    return running_sessions.find(session_id) != std::string::npos;
}

std::vector<std::string> Deploy::check_for_crashes(bool script_started,bool deploy_remote, bool has_local_hlc, bool lab_mode_on, bool check_for_recording)
{
    //std::cout << "Gets called with: " << script_started << ", " << deploy_remote << ", " << has_local_hlc << ", " << lab_mode_on << ", " << check_for_recording << std::endl;
    std::vector<std::string> crashed_participants;
    if ((!(deploy_remote) || has_local_hlc))
    {
        if (script_started)
        {
            if(! session_exists(hlc_session)) crashed_participants.push_back("HLC");
        }

        if(! session_exists(middleware_session)) crashed_participants.push_back("Middleware");
    }
    if (deploy_remote && has_local_hlc && script_started)
    {
        for( unsigned int local_hlc : deployed_local_hlcs ) {
            std::string tmp_session_name = hlc_session+"_"+std::to_string(local_hlc);
            if(! session_exists(tmp_session_name)){
		    crashed_participants.push_back(tmp_session_name);
	    }
        }
        if(! session_exists(middleware_session)) crashed_participants.push_back("Middleware");
    }
    if (lab_mode_on)
    {
        if(! session_exists(ips_session)) crashed_participants.push_back("IPS");
        if(! session_exists(basler_session)) crashed_participants.push_back("Basler LED detection");
    }
    if (check_for_recording)
    {
        if(! session_exists(recording_session)) crashed_participants.push_back("DDS Recording");
        if(! session_exists(labcam_session)) crashed_participants.push_back("LabCam");
    }

    return crashed_participants;
}

void Deploy::kill_session(std::string session_id)
{
    if (session_exists(session_id))
    {
        std::stringstream command;
        command 
            << "tmux kill-session -t \"" << session_id << "\""
            << " >" << software_top_folder_path << "/lcc_script_logs/stdout_tmux_kill.txt 2>" << software_top_folder_path << "/lcc_script_logs/stderr_tmux_kill.txt";

        //Execute command
        program_executor->execute_command(command.str());
    }
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
        << "rm -rf " << software_top_folder_path << "/" << name << ";"
        << "mkdir -p " << software_top_folder_path << "/" << name;

    //Execute command
    program_executor->execute_command(command_folder.str());
}
