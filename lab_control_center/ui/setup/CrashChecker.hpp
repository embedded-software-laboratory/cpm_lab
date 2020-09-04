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

#pragma once

#include <gtkmm/builder.h>
#include <gtkmm.h>

#include "ui/setup/Deploy.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/SimpleTimer.hpp"
#include "RemoteProgramCheck.hpp"

#include <dds/pub/ddspub.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <vector>

/**
 * \brief This class is responsible for checking if background tasks that are supposed to be running are still running - else, a warning is displayed
 */
class CrashChecker
{
private:
    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    //Access to deploy functions
    std::shared_ptr<Deploy> deploy_functions;

    //Access to GUI thread to display warning dialog
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    void ui_dispatch(); //dispatcher callback for the UI thread
    
    //Watcher thread that checks if the locally deployed programs still run - else, an error message is displayed
    std::thread thread_deploy_crash_check;
    std::mutex crashed_mutex;
    std::unordered_set<std::string> already_crashed_participants;
    std::vector<std::string> newly_crashed_participants;
    std::atomic_bool crash_check_running;
    std::shared_ptr<Gtk::MessageDialog> crash_dialog;
    void kill_crash_check_thread();

    //Data structures for remote program crash check
    std::mutex map_mutex;
    std::map<std::string, bool> script_running;
    std::map<std::string, bool> middleware_running;
    std::vector<uint8_t> hlc_ids_copy;
    std::atomic_uint8_t check_msg_count;
    std::atomic_bool is_first_run;
    std::shared_ptr<cpm::SimpleTimer> remote_crash_check;

    //DDS participants for remote program crash check
    dds::topic::Topic<RemoteProgramCheck> program_check_topic;
    dds::pub::DataWriter<RemoteProgramCheck> program_check_writer;
    cpm::AsyncReader<RemoteProgramCheck> program_check_reader;

    //Further helper functions
    void send_remote_check_msg();
    std::vector<std::string> check_for_remote_crashes(std::vector<uint8_t>& remote_hlc_ids);
    void update_crashed_participants(std::vector<std::string> crashed_participants);

public:
    /**
     * \brief Constructor
     * \param _deploy_functions Needed to call deploy / upload on NUCs
     */
    CrashChecker(
        std::shared_ptr<Deploy> _deploy_functions
    );

    ~CrashChecker();

    /**
     * \brief Start checking if the deployed applications are still running
     * \param remote_hlc_ids If deployed remotely: IDs of HLCs on which the software was deployed
     * \param lab_mode_on Whether the IPS etc. should be running
     * \param labcam_toggled If true, the labcam program should be running too
     */
    void start_checking(std::vector<uint8_t> remote_hlc_ids, bool lab_mode_on, bool labcam_toggled);

    /**
     * \brief Stop checking for crashes
     */
    void stop_checking();

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);
};