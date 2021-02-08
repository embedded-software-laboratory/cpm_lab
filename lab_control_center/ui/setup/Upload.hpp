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
#include "ui/setup/UploadWindow.hpp"

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
#include <vector>

#include "LCCErrorLogger.hpp"

/**
 * \brief This class is responsible for managing upload tasks to the NUCs, as well as showing (in the UI) that an upload is performed / how it worked out
 */
class Upload
{
private:
    //Function to get the main window
    std::function<Gtk::Window&()> get_main_window;

    //Function to get a list of all currently online HLCs
    std::function<std::vector<uint8_t>()> get_hlc_ids;

    //Access to deploy functions
    std::shared_ptr<Deploy> deploy_functions;

    //Functions to access UI tools of calling class
    std::function<void()> undo_ui_greyout;
    std::function<void()> undo_kill_button_greyout;
    std::function<void()> on_kill_finished_callback;

    const unsigned int remote_deploy_timeout = 30; //Wait for 30s until the deployment is aborted (for each thread)
    const unsigned int remote_kill_timeout = 2; //Wait for 2 seconds until kill is aborted

    //Loading window while HLC scripts are being updated
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::vector<std::thread> upload_threads; //threads that are responsible for uploading scripts to the HLCs
    std::mutex upload_threads_mutex;
    std::shared_ptr<UploadWindow> upload_window; //window that shows an upload message
    void ui_dispatch(); //dispatcher callback for the UI thread
    /**
     * \brief Notify function that gets called by the upload threads when they have finished their work
     * \param hlc_id ID the thread was responsible for
     * \param upload_success Whether the upload was successful
     * \param is_kill Was added to distinguish between upload and kill, s.t. the system sets "upload done" only to true after deploying, but to false after killing the running programs on the HLC (required for the crash checker)
     */
    void notify_upload_finished(uint8_t hlc_id, bool upload_success, bool is_kill = false);
    void join_upload_threads(); //function to join all threads
    bool check_if_online(uint8_t hlc_id); //Check if the HLC is still online
    std::atomic_uint8_t thread_count; //thread counter, set before thread creation so that, if they finish before the next one is created, still threads are only joined after all upload threads that need to be created have finished their work
    size_t notify_count; //counter for notify_upload_finished; if it does not match thread_count after all threads have called it, print an error message (means that there was a setup mistake made at thread creation)
    std::mutex notify_callback_in_use; //the notify_upload_finished function should only be accessible by one thread at once, thus use this mutex
    std::atomic_bool participants_available; //Used by deploy and ui_dispatch in case the upload fails because no HLC was online or no vehicle was selected
    //Horrible way to log an error message, because the UI cannot be accessed directly - if error_msg.size() > 0, emit just triggers that an error msg is added
    std::mutex error_msg_mutex;
    std::vector<std::string> error_msg;
    std::atomic_bool kill_called; //Must be known to the UI functions - undo grey out of the UI elements after the notification window is closed

    //Can be retrieved to find out if the upload was finished
    std::atomic_bool upload_done;

public:
    /**
     * \brief Constructor
     * \param _get_hlc_ids Function to get IDs of currently online HLCs
     * \param _deploy_functions Needed to call deploy / upload on NUCs -> TODO: Maybe take this function portion from deploy instead
     */
    Upload(
        std::function<std::vector<uint8_t>()> _get_hlc_ids,
        std::shared_ptr<Deploy> _deploy_functions,
        std::function<void()> _undo_ui_greyout,
        std::function<void()> _undo_kill_button_greyout,
        std::function<void()> _on_kill_finished_callback
    );

    ~Upload();

    /**
     * \brief
     * \param TODO
     * \param sorted_hlc_ids HLC IDs (sorted) for HLCs to deploy on
     * \param sorted_vehicle_ids Vehicle IDs (sorted) for vehicle to deploy for (matched to sorted HLC IDs as good as possible)
     * \param TODO
     */
    void deploy_remote(
        bool simulated_time,
        std::string script_path,
        std::string script_params,
        std::vector<uint8_t> sorted_hlc_ids, 
        std::vector<uint32_t> sorted_vehicle_ids
    );

    /**
     * \brief Kill on all set hlc ids
     */
    void kill_remote();

    /**
     * \brief True if an upload was requested, not killed and if all upload threads have finished
     */
    bool upload_finished();

    /**
     * \brief Set the callback function that returns a reference to the application's main window
     * \param _get_main_window Returns reference to the main window, which is "needed" for window creation (works without it, but not without Gtk complaining in the terminal)
     */
    void set_main_window_callback(std::function<Gtk::Window&()> _get_main_window);
};