#pragma once

/**
 * \brief Namespace for deploy / kill templates (bash commands)
 * Was removed from SetupViewUI because this function should in our opinion not be responsible for these actions
 */

#include <atomic>
#include <array>
#include <cstdio> //For popen
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

class Deploy 
{
public:
    Deploy(unsigned int _cmd_domain_id, std::string _cmd_dds_initial_peer, std::function<void(uint8_t)> _stop_vehicle);

    //Specific local deploy functions
    void deploy_local_hlc(bool use_simulated_time, std::vector<unsigned int> active_vehicle_ids, std::string script_path, std::string script_params);

    void deploy_sim_vehicles(std::vector<unsigned int> simulated_vehicle_ids, bool use_simulated_time);
    void deploy_sim_vehicle(unsigned int id, bool use_simulated_time);

    void kill_local_hlc();
    void kill_vehicles(std::vector<unsigned int> simulated_vehicle_ids, std::vector<unsigned int> active_vehicle_ids);
    void kill_vehicle(unsigned int id);

    void deploy_ips();
    void kill_ips();

    //Specific remote deploy functions
    /**
     * \brief Deploy the script specified in the UI with the given parameters on the HLC with the given ID. The script is responsible for one or multiple vehicle ids
     * \param hlc_id The ID of the HLC
     * \param vehicle_ids One or multiple vehicle IDs, comma-separated
     */
    void deploy_remote_hlc(unsigned int hlc_id, std::string vehicle_ids, bool use_simulated_time, std::string script_path, std::string script_params);
    void kill_remote_hlc(unsigned int hlc_id);

private:
    //These values/functions are set once at startup, never change and are thus stored in this class
    unsigned int cmd_domain_id; 
    std::string cmd_dds_initial_peer;
    std::function<void(uint8_t)> stop_vehicle; //Callback function to send a vehicle stop signal / control to the specified vehicle

    //Helper functions
    void get_path_name(std::string& in, std::string& out_path, std::string& out_name);
    bool session_exists(std::string session_id);
    void kill_session(std::string session_id);

     //Function to execute a shell command and get its output
    std::string execute_command(const char* cmd);
};