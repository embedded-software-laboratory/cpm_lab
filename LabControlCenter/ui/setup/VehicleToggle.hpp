#pragma once

/**
 * \brief This class contains all UI elements that control the desired setup state for one vehicle.
 * This allows for less redundant UI code.
 */

#include <cassert>
#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <functional>
#include <iostream>
#include <sstream>

class VehicleToggle 
{
public:
    VehicleToggle(unsigned int _id);

    enum ToggleState{Off, Simulated, On};

    //Getter
    ToggleState get_state() const;
    unsigned int get_id();
    Gtk::Widget* get_parent();

    //Setter
    void set_state(ToggleState state);
    void set_sensitive(bool sensitive);

private:
    void on_state_changed();
    ToggleState current_state;

    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::FlowBox* parent = nullptr;

    Gtk::Label* label = nullptr;

    Gtk::RadioButton* vehicle_off = nullptr;
    Gtk::RadioButton* vehicle_sim = nullptr;
    Gtk::RadioButton* vehicle_on = nullptr;

    //Given values
    unsigned int id;
    std::function<void(ToggleState)> selection_callback;
};