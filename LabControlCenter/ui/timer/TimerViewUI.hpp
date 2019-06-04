#pragma once

#include "defaults.hpp"
#include <cassert>
#include <memory>
#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/params/ParamViewUI.hpp"

class TimerViewUI {
private:
    Glib::RefPtr<Gtk::Builder> ui_builder;
    Gtk::Box* parent;
    Gtk::Button* button_start;
    Gtk::Button* button_stop;
    Gtk::TreeView* active_timers_treeview;
    Gtk::Label* current_timestep_label;

public:
    TimerViewUI();
    Gtk::Widget* get_parent();
};