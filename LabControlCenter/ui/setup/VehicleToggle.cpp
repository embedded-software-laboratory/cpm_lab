#include "VehicleToggle.hpp"

VehicleToggle::VehicleToggle(uint8_t _id) :
    id(_id)
{
    builder = Gtk::Builder::create_from_file("ui/setup/vehicle_toggle.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("label", label);
    builder->get_widget("vehicle_off", vehicle_off);
    builder->get_widget("vehicle_sim", vehicle_sim);
    builder->get_widget("vehicle_on", vehicle_on);

    assert(parent);
    assert(label);
    assert(vehicle_off);
    assert(vehicle_sim);
    assert(vehicle_on);

    //Set label for vehicle toggle box
    std::stringstream label_str;
    label_str << "Vehicle " << id;
    label->set_text(label_str.str().c_str());

    //Group buttons
    vehicle_sim->join_group(*vehicle_off);
    vehicle_on->join_group(*vehicle_off);
    vehicle_off->set_active(true);

    //Register button callbacks
    vehicle_off->signal_toggled().connect(sigc::mem_fun(this, &VehicleToggle::on_state_changed));
    vehicle_sim->signal_toggled().connect(sigc::mem_fun(this, &VehicleToggle::on_state_changed));
    vehicle_on->signal_toggled().connect(sigc::mem_fun(this, &VehicleToggle::on_state_changed));

    current_state = ToggleState::On;
}

void VehicleToggle::on_state_changed()
{
    if(vehicle_sim->get_active())
    {
        current_state = ToggleState::Simulated;
    }
    else if(vehicle_on->get_active())
    {
        current_state = ToggleState::Off;   
    }
    else 
    {
        current_state = ToggleState::On;
    }
}

VehicleToggle::ToggleState VehicleToggle::get_state() const
{
    return current_state;
}

uint8_t VehicleToggle::get_id()
{
    return id;
}

Gtk::Widget* VehicleToggle::get_parent()
{
    return parent;
}

void VehicleToggle::set_state(ToggleState state)
{
    current_state = state;

    switch(state)
    {
        case ToggleState::On:
            vehicle_on->set_active(true);
            break;

        case ToggleState::Simulated:
            vehicle_sim->set_active(true);
            break;

        default:
            vehicle_off->set_active(true);
    }
}

void VehicleToggle::set_sensitive(bool sensitive)
{
    parent->set_sensitive(sensitive);
}
