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
    Gtk::TreeView* active_timers_treeview;
    Gtk::Label* current_timestep_label;

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

public:
    TimerViewUI(std::shared_ptr<TimerTrigger> timerTrigger);
    ~TimerViewUI();
    Gtk::Widget* get_parent();
};