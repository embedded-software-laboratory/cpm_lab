#pragma once

#include "defaults.hpp"
#include <cassert>
#include <memory>
#include <map>
#include <string>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

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
    uint64_t current_simulated_time; //Only makes sense if simulated time is used

public:
    TimerViewUI(bool simulated_time);
    Gtk::Widget* get_parent();
};