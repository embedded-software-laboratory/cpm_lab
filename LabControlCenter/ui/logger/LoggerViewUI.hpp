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

    //Filtering
    Glib::RefPtr<Gtk::TreeModelFilter> filter;
    bool filter_func(const Gtk::TreeModel::const_iterator& iter);

public:
    LoggerViewUI(std::shared_ptr<LogStorage> logStorage);
    ~LoggerViewUI();
    Gtk::Widget* get_parent();
};