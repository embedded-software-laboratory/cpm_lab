#pragma once

#include "defaults.hpp"
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

#include "LoggerModelRecord.hpp"
#include "TimerTrigger.hpp"

#include "cpm/AsyncReader.hpp"
#include "cpm/get_topic.hpp"
#include "cpm/ParticipantSingleton.hpp"
#include "dds/pub/DataWriter.hpp"

#include "src/LogStorage.hpp"


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

    //TreeView Layout, status storage for the UI
    LoggerModelRecord log_record;
    Glib::RefPtr<Gtk::ListStore> log_list_store;

    //UI update functions and objects
    void update_ui();
    void dispatcher_callback();
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    std::thread ui_thread;
    std::atomic_bool run_thread;

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
    std::atomic_bool search_result_used;
    //Promise and future for search thread
    std::promise<std::vector<Log>> search_promise;
    std::future<std::vector<Log>> search_future;
    std::atomic_bool search_thread_running;
    std::thread search_thread;
    void start_new_search_thread();
    void kill_search_thread();

    //Delete old logs
    void delete_old_logs(const long max_amount);

public:
    LoggerViewUI(std::shared_ptr<LogStorage> logStorage);
    ~LoggerViewUI();
    Gtk::Widget* get_parent();
};