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

#include "Upload.hpp"

Upload::Upload(
        std::function<std::vector<uint8_t>()> _get_hlc_ids,
        std::shared_ptr<Deploy> _deploy_functions,
        std::function<void()> _undo_ui_greyout,
        std::function<void()> _undo_kill_button_greyout,
        std::function<void()> _on_kill_finished_callback
    ) :
    get_hlc_ids(_get_hlc_ids),
    deploy_functions(_deploy_functions),
    undo_ui_greyout(_undo_ui_greyout),
    undo_kill_button_greyout(_undo_kill_button_greyout),
    on_kill_finished_callback(_on_kill_finished_callback)
{
    ui_dispatcher.connect(sigc::mem_fun(*this, &Upload::ui_dispatch));
    thread_count.store(0);
    notify_count = 0;
    participants_available.store(false);
    kill_called.store(false);
    upload_done.store(false);
}

Upload::~Upload()
{
    join_upload_threads();
}

void Upload::set_main_window_callback(std::function<Gtk::Window&()> _get_main_window)
{
    get_main_window = _get_main_window;
}

void Upload::deploy_remote(
        bool simulated_time,
        std::string script_path,
        std::string script_params,
        std::vector<uint8_t> sorted_hlc_ids, 
        std::vector<uint32_t> sorted_vehicle_ids
    )
{
    size_t min_hlc_vehicle = std::min(sorted_hlc_ids.size(), sorted_vehicle_ids.size());

    //Show window indicating that the upload process currently takes place
    //An error message is shown if no HLC is online - in that case, take additional action here as well: Just show the window and deploy nothing
    if (get_main_window)
    {
        upload_window = std::make_shared<UploadWindow>(get_main_window(), sorted_vehicle_ids, sorted_hlc_ids);
    }
    else
    {
        cpm::Logging::Instance().write(
            1,
            "%s", 
            "ERROR: Main window reference is missing, cannot create upload dialog"
        );
    }

    //Do not deploy anything remotely if no HLCs are online or if no vehicles were selected
    if (sorted_hlc_ids.size() == 0 || sorted_vehicle_ids.size() == 0)
    {
        //Waits a few seconds before the window is closed again 
        //Window still needs UI dispatcher (else: not shown because UI gets unresponsive), so do this by using a thread + atomic variable (upload_failed)
        participants_available.store(false); //No HLCs available
        thread_count.store(1);
        std::lock_guard<std::mutex> lock(upload_threads_mutex);
        upload_threads.push_back(std::thread(
            [this] () {
                usleep(1000000);
                this->notify_upload_finished(0, true, true); //We do not want upload finished to be set here as well
            }
        ));
        return;
    }

    //Deploy on each HLC individually, using different threads
    participants_available.store(true); //HLCs are available
    thread_count.store(min_hlc_vehicle);
    for (size_t i = 0; i < min_hlc_vehicle; ++i)
    {
        //Deploy on high_level_controller with given vehicle id(s)
        std::stringstream vehicle_id_stream;
        vehicle_id_stream << sorted_vehicle_ids.at(i);

        //Create variables for the thread
        unsigned int hlc_id = static_cast<unsigned int>(sorted_hlc_ids.at(i));
        std::string vehicle_string = vehicle_id_stream.str();

        //Create thread
        std::lock_guard<std::mutex> lock(upload_threads_mutex);
        upload_threads.push_back(std::thread(
            [this, hlc_id, vehicle_string, simulated_time, script_path, script_params] () {
                bool deploy_worked = deploy_functions->deploy_remote_hlc(
                    hlc_id, 
                    vehicle_string, 
                    simulated_time,
                    script_path,
                    script_params,
                    remote_deploy_timeout,
                    std::bind(&Upload::check_if_online, this, hlc_id)
                );
                this->notify_upload_finished(hlc_id, deploy_worked);
            }
        ));
    }
}

void Upload::kill_remote()
{
    upload_done.store(false);

    std::vector<uint8_t> hlc_ids;
    if (get_hlc_ids)
    {
        hlc_ids = get_hlc_ids();
    }
    else 
    {
        std::cerr << "No lookup function to get HLC IDs given, cannot kill on HLCs" << std::endl;
        return;
    }

    //Show window indicating that the upload process currently takes place
    //An error message is shown if no HLC is online - in that case, take additional action here as well: Just show the window and deploy nothing
    if (get_main_window)
    {
        upload_window = std::make_shared<UploadWindow>(get_main_window(), std::vector<unsigned int>(), hlc_ids);
        upload_window->set_text("Killing on remote HLCs...");
    }
    else
    {
        cpm::Logging::Instance().write(
            1,
            "%s", 
            "ERROR: Main window reference is missing, cannot create upload dialog"
        );
    }
    
    //Let the UI dispatcher know that kill-related actions need to be performed after all threads have finished
    kill_called.store(true);

    //Do not try to kill if no HLCs are online
    if (hlc_ids.size() == 0)
    {
        //Waits a few seconds before the window is closed again 
        //Window still needs UI dispatcher (else: not shown because UI gets unresponsive), so do this by using a thread + atomic variable (upload_failed)
        participants_available.store(false); //No HLCs available
        thread_count.store(1);
        std::lock_guard<std::mutex> lock(upload_threads_mutex);
        upload_threads.push_back(std::thread(
            [this] () {
                usleep(1000000);
                this->notify_upload_finished(0, true, true);
            }
        ));
        return;
    }


    //If a HLC went offline in between, we assume that it crashed and thus just use this script on all remaining running HLCs
    thread_count.store(hlc_ids.size());
    for (const auto& hlc_id : hlc_ids)
    {
        //Create thread
        std::lock_guard<std::mutex> lock(upload_threads_mutex);
        upload_threads.push_back(std::thread(
            [this, hlc_id] () {
                bool kill_worked = deploy_functions->kill_remote_hlc(
                    hlc_id, 
                    remote_kill_timeout,
                    std::bind(&Upload::check_if_online, this, hlc_id)
                );
                this->notify_upload_finished(hlc_id, kill_worked, true);
            }
        ));
    }
}

void Upload::join_upload_threads()
{
    //Join all old threads - gets called from destructor, kill and when the last thread finished (in the ui thread dispatcher)
    std::lock_guard<std::mutex> lock(upload_threads_mutex);
    for (auto& thread : upload_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
        else 
        {
            std::cerr << "Warning: Shutting down with upload thread that cannot be joined" << std::endl;
        }
    }
    upload_threads.clear();
}

void Upload::ui_dispatch()
{
    //Take care of upload window etc
    std::lock_guard<std::mutex> lock_msg(error_msg_mutex);
    if (error_msg.size() > 0)
    {
        for (auto &msg : error_msg)
        {
            upload_window->add_error_message(msg);
        }
        error_msg.clear();
    }
    else 
    {
        //The only current job for ui_dispatch is to close the upload window shown after starting the upload threads, when all threads have been closed
        //Plus now, kill is not grayed out anymore
        std::unique_lock<std::mutex> lock(upload_threads_mutex);
        if (upload_threads.size() != 0 && upload_window)
        {
            upload_window->close();

            if (undo_kill_button_greyout)
                undo_kill_button_greyout();
            else
                std::cerr << "ERROR: Callback for undoing kill button grey-out missing in Upload class" << std::endl;
        }
        lock.unlock();

        //Join all old threads
        join_upload_threads();

        //If kill caused the UI dispatch, clean up after everything has been killed
        if (kill_called.exchange(false))
        {
            on_kill_finished_callback();
        }

        //Free the UI if the upload was not successful
        if (!participants_available.load())
        {
            if (undo_ui_greyout)
                undo_ui_greyout();
            else
                std::cerr << "ERROR: Callback for undoing ui grey-out missing in Upload class" << std::endl;
        }
    }
}

void Upload::notify_upload_finished(uint8_t hlc_id, bool upload_success, bool is_kill)
{
    //Just try to join all worker threads here
    std::lock_guard<std::mutex> lock(notify_callback_in_use);

    //This should never be the case
    //If this happens, the thread count has been initialized incorrectly
    if (thread_count.load() == 0)
    {
        std::cerr << "WARNING: Upload thread count has not been initialized correctly!" << std::endl;
    }

    //Trigger error msg if the upload failed
    if (!upload_success)
    {
        std::lock_guard<std::mutex> lock_msg(error_msg_mutex);
        std::stringstream error_msg_stream;
        error_msg_stream << "ERROR: Connection or upload failed for HLC ID " << static_cast<int>(hlc_id) << std::endl;
        error_msg.push_back(error_msg_stream.str());
        ui_dispatcher.emit();
    }

    //Also count notify amount s.t one can check if the thread count has been set properly
    thread_count.fetch_sub(1);
    ++notify_count;

    if (thread_count.load() == 0)
    {
        //Upload was finished
        notify_count = 0;

        if (!is_kill) upload_done.store(true); //Only relevant for deploy, must stay false after kill until next deploy

        //Close upload window again, but only after a while
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ui_dispatcher.emit();
    }
    else 
    {
        std::lock_guard<std::mutex> lock(upload_threads_mutex);
        if (notify_count == upload_threads.size())
        {
            std::cerr << "WARNING: Upload thread count has not been initialized correctly!" << std::endl;

            notify_count = 0;

            //Close upload window again, but only after a while
            std::this_thread::sleep_for(std::chrono::seconds(2));
            ui_dispatcher.emit();
        }
    }
}

bool Upload::check_if_online(uint8_t hlc_id)
{
    if (!get_hlc_ids)
    {
        std::cerr << "ERROR: Callback for getting HLC IDs missing in Upload class" << std::endl;
    }

    //Check if the HLC is still online (in get_hlc_ids)
    std::vector<uint8_t> hlc_ids = get_hlc_ids();
    return std::find(hlc_ids.begin(), hlc_ids.end(), static_cast<uint8_t>(hlc_id)) != hlc_ids.end();
}

bool Upload::upload_finished()
{
    return upload_done.load();
}