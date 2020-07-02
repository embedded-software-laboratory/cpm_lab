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

#include "VehicleToggle.hpp"

VehicleToggle::VehicleToggle(unsigned int _id) :
    id(_id)
{
    builder = Gtk::Builder::create_from_file("ui/setup/vehicle_toggle.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("label", label);
    builder->get_widget("vehicle_switch", vehicle_switch);

    assert(parent);
    assert(label);
    assert(vehicle_switch);

    //Set label for vehicle toggle box
    std::stringstream label_str;
    label_str << "Vehicle " << id;
    label->set_text(label_str.str().c_str());

    current_state = ToggleState::Off;

    //Register switch callback
    vehicle_switch->property_active().signal_changed().connect(sigc::mem_fun(this, &VehicleToggle::on_state_changed));
}

void VehicleToggle::on_state_changed()
{
    if(vehicle_switch->get_active())
    {
        current_state = ToggleState::Simulated;
    }
    else 
    {
        current_state = ToggleState::Off;
    }
}

VehicleToggle::ToggleState VehicleToggle::get_state() const
{
    return current_state;
}

unsigned int VehicleToggle::get_id()
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
        case ToggleState::Simulated:
            vehicle_switch->set_active(true);
            break;

        default:
            vehicle_switch->set_active(false);
    }
}

void VehicleToggle::set_sensitive(bool sensitive)
{
    parent->set_sensitive(sensitive);
}
