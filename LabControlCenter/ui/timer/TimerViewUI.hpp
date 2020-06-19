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

#include "TimerModelRecord.hpp"
#include "TimerTrigger.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "ReadyStatus.hpp"
#include "SystemTrigger.hpp"


class TimerViewUI {
private:
    Glib::RefPtr<Gtk::Builder> ui_builder;
    Gtk::Box* parent;
    Gtk::Button* button_start;
    Gtk::Button* button_stop;
    Gtk::Button* button_reset;
    Gtk::TreeView* active_timers_treeview;
    Gtk::Label* current_timestep_label;

    /**
     * \brief Reset function - this function is called whenever the timer type is changed, 
     * because in that case the simulation can be restarted, old participants become irrelevant
     * etc. This function is called when the reset button is pressed
     */
    void button_reset_callback();

    //TreeView Layout, status storage for the UI
    TimerModelRecord timer_record;
    Glib::RefPtr<Gtk::ListStore> timer_list_storage;
    void update_ui();

    //Timing functions
    /**
     * \brief Send a start signal
     */
    void button_start_callback();
    /**
     * \brief Send a stop signal once
     */
    void button_stop_callback();

    //Storage / logic for timer
    std::shared_ptr<TimerTrigger> timer_trigger;

    //UI thread
    void dispatcher_callback();
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    //Helper functions
    std::string participant_status_to_ustring(ParticipantStatus response);
    std::atomic_bool system_is_running;
    void start_ui_thread();
    void stop_ui_thread();
    void reset_ui();
    
    /**
     * \brief Get the time diff to the current time as string in (minutes:)seconds (minutes if seconds > 60)
     */
    std::string get_human_readable_time_diff(uint64_t other_time);

public:
    TimerViewUI(std::shared_ptr<TimerTrigger> timerTrigger);
    ~TimerViewUI();
    Gtk::Widget* get_parent();

    /**
     * \brief Reset function - this function is called whenever the timer type is changed, 
     * because in that case the simulation can be restarted, old participants become irrelevant
     * etc. 
     */
    void reset(bool use_simulated_time, bool send_stop_signal);
};