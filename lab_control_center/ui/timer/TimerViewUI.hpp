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

#include "defaults.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <ctime>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

#include "ObstacleSimulationManager.hpp"
#include "TimerModelRecord.hpp"
#include "TimerTrigger.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"

#include "ui/setup/CrashChecker.hpp"

/**
 * \class TimerViewUI
 * \brief LCC UI class that shows the timer (simulated or real) and allows to start / stop / restart it. 
 * All currently online participants that use the timer are shown as well
 * \ingroup lcc_ui
 */
class TimerViewUI {
private:
    //! GTK UI Builder 
    Glib::RefPtr<Gtk::Builder> ui_builder;
    //! Parent UI element, which can be used to place the Timer UI within another UI element and which contains all Timer UI elements
    Gtk::Box* parent;
    //! Button to start all currently connected timers
    Gtk::Button* button_start;
    //! Button to stop all currently connected timers
    Gtk::Button* button_stop;
    //! Button to reset the LCCs timer and stop all currently connected timers
    Gtk::Button* button_reset;
    //! TreeView that shows a list of currently active timers and additional information for each of them
    Gtk::TreeView* active_timers_treeview;
    //! Displays the current time (for simulated time)
    Gtk::Label* current_timestep_label;

    /**
     * \brief Reset function - this function is called whenever the timer type is changed, 
     * because in that case the simulation can be restarted, old participants become irrelevant
     * etc. This function is called when the reset button is pressed
     */
    void button_reset_callback();

    //! TreeView Layout
    TimerModelRecord timer_record;
    //! Timer status storage for the UI (shown in the TreeView)
    Glib::RefPtr<Gtk::ListStore> timer_list_storage;

    //Timing functions
    /**
     * \brief Send a start signal
     */
    void button_start_callback();
    /**
     * \brief Send a stop signal once
     */
    void button_stop_callback();

    //! Storage / logic for timer
    std::shared_ptr<TimerTrigger> timer_trigger;

    //UI thread
    /**
     * \brief Called by ui_thread regularly to trigger dispatcher_callback
     */
    void update_ui();
    /**
     * \brief Triggered by GTK's UI dispatcher, to operate on the UI thread. 
     * Allows to update entries in the TreeView as well as the current time.
     */
    void dispatcher_callback();
    //! To communicate between thread and GUI
    Glib::Dispatcher ui_dispatcher;
    //! UI thread that triggers the ui_dispatcher regularly
    std::thread ui_thread;
    //! Run condition for the UI thread, to stop it when set to false
    std::atomic_bool run_thread;

    //Helper functions
    /**
     * \brief Takes a participant status and translates it to a string
     */
    std::string participant_status_to_ustring(ParticipantStatus response);
    //! Used by participant_status_to_ustring to determine if the timer is currently running (start was clicked and reset wasn't)
    std::atomic_bool system_is_running;

    /**
     * \brief Start the UI thread
     */
    void start_ui_thread();
    /**
     * \brief Stop the UI thread (only called by destructor and reset_button_callback (where it is restarted again))
     */
    void stop_ui_thread();
    /**
     * \brief Reset information presented in the UI, allow to click the start button again
     */
    void reset_ui();
    
    /**
     * \brief Get the time diff to the current time as string in (minutes:)seconds (minutes if seconds > 60)
     */
    std::string get_human_readable_time_diff(uint64_t other_time);

    //! Deprecated / no longer required. Class to simulate obstacles based on the loaded commonroad file
    std::shared_ptr<ObstacleSimulationManager> obstacle_simulation_manager;

    //! Stop checking for crashes if the timer is stopped
    std::shared_ptr<CrashChecker> crash_checker;

public:
    /**
     * \brief Constructor
     * \param timerTrigger E.g. to send stop signals
     * \param _obstacle_simulation_manager Used to simulate obstacles defined in currently loaded commonroad file - reference here for start(), stop()
     */
    TimerViewUI(
        std::shared_ptr<TimerTrigger> timerTrigger,
        std::shared_ptr<ObstacleSimulationManager> _obstacle_simulation_manager
    );

    /**
     * \brief Destructor, kills the UI thread
     */
    ~TimerViewUI();

    /**
     * \brief Get the parent widget of TimerViewUI to put it in a parent container
     */
    Gtk::Widget* get_parent();

    /**
     * \brief Checker needs to be set up in SetupView, but the crash checker can also be killed by a simple timer stop or reset
     */
    void register_crash_checker(std::shared_ptr<CrashChecker> _crash_checker);

    /**
     * \brief Reset function - this function is called whenever the timer type is changed, 
     * because in that case the simulation can be restarted, old participants become irrelevant
     * etc. 
     */
    void reset(bool use_simulated_time, bool send_stop_signal);
};