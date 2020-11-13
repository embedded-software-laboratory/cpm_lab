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
#include "ui/setup/Upload.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_time_ns.hpp"
#include "cpm/SimpleTimer.hpp"

#include "src/HLCReadyAggregator.hpp"

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
 * It also reports wrong script/program locations set in the LCC for the main script
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
    std::mutex hlc_id_mutex;
    std::vector<uint8_t> running_remote_hlcs;
    std::vector<uint8_t> crashed_remote_hlcs;

    //Access to remote program check
    std::shared_ptr<HLCReadyAggregator> hlc_ready_aggregator;
    std::shared_ptr<Upload> upload_manager;
    uint64_t upload_success_time = 0;
    const uint64_t remote_waiting_time = 5e9; //Report remote crashes only after 5 seconds (messages seem to "stutter" / are not yet correct directly after the upload)

    //Further helper functions
    void send_remote_check_msg();
    std::vector<std::string> check_for_remote_crashes();
    void update_crashed_participants(std::vector<std::string> crashed_participants);

public:
    /**
     * \brief Constructor
     * \param _deploy_functions Needed to find out if local programs are still running
     * \param _hlc_ready_aggregator Required to find out if remote programs are still running
     * \param _upload_manager Required to check if an upload was finished, for remote checking
     */
    CrashChecker(
        std::shared_ptr<Deploy> _deploy_functions,
        std::shared_ptr<HLCReadyAggregator> _hlc_ready_aggregator,
        std::shared_ptr<Upload> _upload_manager
    );

    ~CrashChecker();

    /**
     * \brief Start checking if the deployed applications are still running
     * \param script_used If a script + middleware was deployed (else only check for crashes of IPS etc) 
     * \param remote_hlc_ids If deployed remotely: IDs of HLCs on which the software was deployed
     * \param has_local_hlc True if local HLCs are used on top of remote ones
     * \param lab_mode_on Whether the IPS etc. should be running
     * \param labcam_toggled If true, the labcam program should be running too
     */
    void start_checking(bool script_used, std::vector<uint8_t> remote_hlc_ids, bool has_local_hlc, bool lab_mode_on, bool labcam_toggled);

    /**
     * \brief Stop checking for crashes
     */
    void stop_checking();

    /**
     * \brief This function can be used to check if the program deployed on an HLC with the given ID crashed (remote)
     * \param hlc_id ID of the HLC
     * \return True if the checker is running and the program crashed, else false
     */
    bool check_if_crashed(uint8_t hlc_id);

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);
};
