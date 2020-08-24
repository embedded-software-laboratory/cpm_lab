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
    builder->get_widget("vehicle_button", vehicle_button);

    assert(parent);
    assert(label);
    assert(vehicle_button);

    //Set label for vehicle toggle box
    std::stringstream label_str;
    label_str << "Vehicle " << id;
    label->set_text(label_str.str().c_str());

    current_state = ToggleState::Off;
    update_style();

    //Register switch callback
    vehicle_button->signal_clicked().connect(sigc::mem_fun(this, &VehicleToggle::on_state_changed));

    signal_thread_stop.store(false);
    is_sensitive.store(true); //Sensitive by default
    ui_dispatcher.connect(sigc::mem_fun(*this, &VehicleToggle::ui_dispatch));
}

VehicleToggle::~VehicleToggle()
{
    //Tell thread to abort early
    signal_thread_stop.store(true);

    if(set_insensitive_thread.joinable())
    {
        set_insensitive_thread.join();
    }
}

void VehicleToggle::on_state_changed()
{
    if(current_state == ToggleState::Off)
    {
        current_state = ToggleState::Simulated;
    }
    else if (current_state == ToggleState::Simulated)
    {
        current_state = ToggleState::Off;
    }

    update_style();

    if(selection_callback) selection_callback(id, current_state);
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

    update_style();
}

void VehicleToggle::update_style()
{
    if(current_state == ToggleState::Simulated)
    {
        vehicle_button->get_style_context()->remove_class("vehicle_toggle_off");
        vehicle_button->get_style_context()->add_class("vehicle_toggle_sim");
        vehicle_button->set_label("Turn Off");
    }
    else if (current_state == ToggleState::Off)
    {
        vehicle_button->get_style_context()->remove_class("vehicle_toggle_sim");
        vehicle_button->get_style_context()->add_class("vehicle_toggle_off");
        vehicle_button->set_label("Simulate");
    }
    else 
    {
        vehicle_button->get_style_context()->remove_class("vehicle_toggle_off");
        vehicle_button->get_style_context()->remove_class("vehicle_toggle_sim");
        vehicle_button->set_label("Reboot");
    }
}

void VehicleToggle::set_sensitive(bool sensitive)
{
    parent->set_sensitive(sensitive);
}

void VehicleToggle::set_insensitive(uint timeout_seconds)
{
    //Tell thread to abort early
    signal_thread_stop.store(true);

    if(set_insensitive_thread.joinable())
    {
        set_insensitive_thread.join();
    }

    signal_thread_stop.store(false);

    set_insensitive_thread = std::thread(
        [this, timeout_seconds] ()
        {
            is_sensitive.store(false);
            ui_dispatcher.emit();

            int dt_ms = 100;
            for (uint t_ms = 0; t_ms < timeout_seconds*1000; t_ms += dt_ms)
            {
                //Abort waiting early on destruction
                if (signal_thread_stop.load())
                {
                    return;
                }                
                std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
            }

            is_sensitive.store(true);
            ui_dispatcher.emit();
        }
    );
}

void VehicleToggle::ui_dispatch()
{
    set_sensitive(is_sensitive.load());
}

void VehicleToggle::set_selection_callback(std::function<void(unsigned int,ToggleState)> _selection_callback)
{
    selection_callback = _selection_callback;
}