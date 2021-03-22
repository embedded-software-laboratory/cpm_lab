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
 * \ingroup lcc_ui
 */
class Upload
{
private:
    //! Callback function to get the main window, required as reference for the upload window
    std::function<Gtk::Window&()> get_main_window;

    //! Callback function to get a list of all currently online HLCs
    std::function<std::vector<uint8_t>()> get_hlc_ids;

    //! Object that allows access to deploy functions
    std::shared_ptr<Deploy> deploy_functions;

    //Callback functions to access UI tools of calling class
    //! Callback that allows to undo the UI greyout performed in the calling object
    std::function<void()> undo_ui_greyout;
    //! Callback that allows to undo the kill button greyout performed in the calling object
    std::function<void()> undo_kill_button_greyout;
    //! Callback that tells the calling object that the remote kill operation was finished
    std::function<void()> on_kill_finished_callback;

    //! Wait up to 30s until the upload for the scripts deployment is aborted (for each thread)
    const unsigned int remote_deploy_timeout = 30;
    //! Wait up to 2s until the kill command is aborted (for each thread)
    const unsigned int remote_kill_timeout = 2;

    //Managing the upload window
    //! To communicate between thread and GUI
    Glib::Dispatcher ui_dispatcher;
    //! Threads that are responsible for uploading scripts to the HLCs (one thread for each upload)
    std::vector<std::thread> upload_threads;
    //! Mutex for access to upload_threads
    std::mutex upload_threads_mutex;
    //! Window shown during remote upload / kill, displays messages for the user
    std::shared_ptr<UploadWindow> upload_window;
    /**
     * \brief Dispatcher callback for the UI thread. For managing the upload window for HLC scripts.
     * Also calls undo_kill_button_greyout and on_kill_finished_callback if used in context of kill_remote.
     * Calls undo_ui_greyout if the upload was not successful.
     */
    void ui_dispatch();

    /**
     * \brief Notify function that gets called by an upload thread when it has finished its work
     * \param hlc_id HLC (ID) the thread was responsible for
     * \param upload_success Whether the upload was successful
     * \param is_kill Was added to distinguish between upload and kill, s.t. the system sets "upload done" only to true after deploying, but to false after killing the running programs on the HLC (required for the crash checker)
     */
    void notify_upload_finished(uint8_t hlc_id, bool upload_success, bool is_kill = false);

    /**
     * \brief Function to join all threads (upload_threads).
     * Gets called from destructor, kill and when the last upload thread finished its work (in that case from ui_dispatch).
     */
    void join_upload_threads();

    /**
     * \brief Check if the HLC is still online
     * \param hlc_id ID of the HLC to check
     */
    bool check_if_online(uint8_t hlc_id); 

    /**
     * \brief Thread counter, set before thread creation so that threads 
     * are only joined after all upload threads that need to be created have finished their work.
     */
    std::atomic_uint8_t thread_count;
    /**
     * \brief Counter for notify_upload_finished; if it does not match the size of upload_threads 
     * after all threads have called it, print an error message (means that there was a setup mistake made at thread creation)
     */
    size_t notify_count;
    //! The notify_upload_finished function should only be accessible by one thread at once, thus use this mutex
    std::mutex notify_callback_in_use;
    //! Used by deploy and ui_dispatch in case the upload fails because no HLC was online or no vehicle was selected
    std::atomic_bool participants_available;
    //! For access to error_msg
    std::mutex error_msg_mutex;
    //! Horrible way to log an error message, because the UI cannot be accessed directly - if error_msg.size() > 0, the dispatcher was called to add it to the upload window
    std::vector<std::string> error_msg;
    //! Must be known to the UI functions - undo grey out of the UI elements after the notification window is closed
    std::atomic_bool kill_called; 

    //! Can be retrieved to find out if the upload was finished
    std::atomic_bool upload_done;

public:
    /**
     * \brief Constructor
     * \param _get_hlc_ids Function to get IDs of currently online HLCs
     * \param _deploy_functions Needed to call deploy / upload on NUCs
     * \param _undo_ui_greyout Required to undo UI greyout in SetupViewUI if the upload and thus remote deployment fails
     * \param _undo_kill_button_greyout Required to undo the kill button greyout in SetupViewUI after a successful upload
     * \param _on_kill_finished_callback Callback that gets called when all kill threads for killing remote HLCs are done
     */
    Upload(
        std::function<std::vector<uint8_t>()> _get_hlc_ids,
        std::shared_ptr<Deploy> _deploy_functions,
        std::function<void()> _undo_ui_greyout,
        std::function<void()> _undo_kill_button_greyout,
        std::function<void()> _on_kill_finished_callback
    );

    /**
     * \brief Destructor, joins all upload threads
     */
    ~Upload();

    /**
     * \brief Deploy the selected script with the given parameters remotely, using simulated or real time, on the given HLCs.
     * The middleware is started there as well.
     * The vehicle ID for each HLC (to know which vehicle to control) is given as well.
     * (The first ID is associated with the first HLC ID and so on).
     * \param simulated_time True if simulated time should be used, else real time is used
     * \param script_path Path of the script to upload to the HLCs
     * \param script_params Optional command line parameters for the script to start.
     * \param sorted_hlc_ids HLC IDs (sorted) for HLCs to deploy on
     * \param sorted_vehicle_ids Vehicle IDs (sorted) for vehicle to deploy for (matched to sorted HLC IDs as good as possible)
     */
    void deploy_remote(
        bool simulated_time,
        std::string script_path,
        std::string script_params,
        std::vector<uint8_t> sorted_hlc_ids, 
        std::vector<uint32_t> sorted_vehicle_ids
    );

    /**
     * \brief Kill script and middleware on all currently online HLCs, using get_hlc_ids.
     * If a HLC went offline in between, we assume that it crashed and thus stop only on all remaining (online) HLCs.
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