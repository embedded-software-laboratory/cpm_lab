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

#pragma once

/**
 * \brief This class contains all UI elements that control the desired setup state for one vehicle.
 * This allows for less redundant UI code.
 */

#include <cassert>
#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <thread>

class VehicleToggle 
{
public:
    VehicleToggle(unsigned int _id);
    ~VehicleToggle();

    enum ToggleState{Off, Simulated, Real};

    //Getter
    ToggleState get_state() const;
    unsigned int get_id();
    Gtk::Widget* get_parent();

    //Setter
    void set_state(ToggleState state);
    void set_sensitive(bool sensitive);

    /**
     * \brief Set the toggle insensitive for the given timeout, then set it to sensitive again
     * \param timeout_seconds Time in seconds for the toggle to not respond to input
     */
    void set_insensitive(uint timeout_seconds);
    void set_selection_callback(std::function<void(unsigned int, ToggleState)> _selection_callback); //If set, callback gets called on state change

private:
    void on_state_changed();
    ToggleState current_state;

    /**
     * \brief Set the right style / label / ... depending on the current state
     */
    void update_style();

    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::FlowBox* parent = nullptr;

    Gtk::Label* label = nullptr;

    Gtk::Button* vehicle_button = nullptr;

    //Given values
    unsigned int id;
    std::function<void(unsigned int, ToggleState)> selection_callback;

    //Timing for set_insensitive
    std::atomic_bool signal_thread_stop;
    std::atomic_bool is_sensitive;
    std::thread set_insensitive_thread;

    //UI thread for set_insensitive
    //Also: Upload threads and GUI thread (to keep upload work separate from GUI)
    Glib::Dispatcher ui_dispatcher; //to communicate between thread and GUI
    void ui_dispatch(); //dispatcher callback for the UI thread
};