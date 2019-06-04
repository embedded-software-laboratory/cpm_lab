#pragma once

#include "defaults.hpp"
#include <cassert>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

#include "TimerModelRecord.hpp"

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

    bool use_simulated_time = false;

    //Communication objects and callbacks
    void ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples);
    cpm::AsyncReader<ReadyStatus> ready_status_reader;
    dds::pub::DataWriter<SystemTrigger> system_trigger_writer;
    std::map<string, uint64_t> ready_status_storage;
    std::mutex ready_status_storage_mutex;
    uint64_t current_simulated_time; //Only makes sense if simulated time is used

    //TreeView Layout, status storage for the UI
    TimerModelRecord timer_record;
    Glib::RefPtr<Gtk::ListStore> timer_list_storage;
    void insert_or_change_treeview(std::string id_string, std::string waiting_start_string, std::string waiting_response_string, std::string next_step_string);

    //Timing functions
    void send_next_signal();
    void send_stop_signal();

public:
    TimerViewUI(bool simulated_time);
    Gtk::Widget* get_parent();
};