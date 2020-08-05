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
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <ctime>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <glib.h>

#include "LogLevelSetter.hpp"
#include "LoggerModelRecord.hpp"
#include "TimerTrigger.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "src/LogStorage.hpp"

/**
 * \brief Class for the Logs Tab in the LCC. Show most recent logs (->max_log_amount), is connected to LogStorage & allows for search in more than most_recent_logs
 * using Regex, allows to set the log_level for the whole domain
 */
class LoggerViewUI {
private:
    Glib::RefPtr<Gtk::Builder> ui_builder;
    Gtk::Box* parent;
    Gtk::TreeView* logs_treeview;
    Gtk::Label* logs_label_header;
    Gtk::ScrolledWindow* logs_scrolled_window;
    Gtk::CheckButton* autoscroll_check_button;
    Gtk::SearchEntry* logs_search_entry;
    Gtk::ComboBoxText* logs_search_type;
    Gtk::ComboBoxText* log_level_combobox;

    //Max. amount of logs shown in the UI (performance reasons)
    const long max_log_amount = 100;

    //Callback function for log_level_combobox
    void on_log_level_changed();
    std::atomic_ushort log_level;
    //Labels for log_level_combobox (the string vector is transformed to the ustring vector in the settings, where we also add the selection options to the UI)
    std::vector<std::string> log_labels = {"0 - No logs", "1 - Critical errors only", "2 - Errors only", "3 - Verbose"};
    std::vector<Glib::ustring> log_level_labels; //Vector used so that we can infer the level number from the index (no need to transform the ustring to a number)

    //TreeView Layout, status storage for the UI
    LoggerModelRecord log_record;
    Glib::RefPtr<Gtk::ListStore> log_list_store;

    //UI update functions and objects
    void update_ui();
    void dispatcher_callback();
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

    //Callback function for autoscroll
    void on_size_change_autoscroll(Gtk::Allocation& allocation);

    //Object that holds (new) log data
    std::shared_ptr<LogStorage> log_storage;

    //Filtering and filter types
    Glib::ustring type_id_ustring = "ID";
    Glib::ustring type_content_ustring = "Content";
    Glib::ustring type_timestamp_ustring = "Timestamp";
    Glib::ustring type_all_ustring = "All";
    void on_filter_type_changed();
    void stop_search();
    void search_changed();
    std::atomic_bool filter_active;
    //Promise and future for search thread
    std::promise<std::vector<Log>> search_promise;
    std::future<std::vector<Log>> search_future;
    //Extra mutex because promise can be reset right after the UI thread checked if a future exists
    std::mutex promise_reset_mutex;
    std::atomic_bool search_thread_running;
    std::atomic_bool search_reset; //Set after search was finished, to retrieve all old log messages in the UI thread again
    std::thread search_thread;
    void start_new_search_thread();
    void kill_search_thread();

    //Add log entry to UI (only call from UI thread!)
    void add_log_entry(const Log& entry);

    //Delete old logs
    void delete_old_logs(const long max_amount);
    void reset_list_store();

    //Callback for tooltip
    bool tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip);

    //Check for scroll event to turn off automatic scrolling
    bool scroll_callback(GdkEventScroll* scroll_event);

    //Variable for the reset action (is performed within the UI)
    std::atomic_bool reset_logs;

public:
    LoggerViewUI(std::shared_ptr<LogStorage> logStorage);
    ~LoggerViewUI();
    Gtk::Widget* get_parent();

    //Might be called from outside, e.g. when a new 'simulation' is run
    void reset();
};