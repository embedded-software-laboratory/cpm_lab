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

#include <cassert>
#include <gtkmm/builder.h>
#include <gtkmm.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <thread>

/**
 * \brief This class contains all UI elements that control the desired setup state for one vehicle (to turn it off, start it or reboot a real vehicle)
 * This allows for less redundant UI code.
 * \ingroup lcc_ui
 */
class VehicleToggle 
{
public:
    /**
     * \brief Create a vehicle toggle UI element for a vehicle with the given ID
     * \param _id The vehicle ID to create the element for
     */
    VehicleToggle(unsigned int _id);

    /**
     * \brief Kills all threads used internally (thread + ui dispatcher are used for set_sensitive)
     */
    ~VehicleToggle();

    /**
     * \enum ToggleState
     * \brief To remember the current state of the toggle:
     * - Off: Can be turned to Simulated by clicking on the toggle. No real or simulated vehicle with the given ID is currently running.
     * 
     * - Simulated: Can be turned to off by clicking on the toggle. A simulated vehicle with the given ID is currently running.
     * 
     * - Real: A real vehicle with the given ID is currently running. By clicking on the toggle, it can be rebooted.
     */
    enum ToggleState{Off, Simulated, Real};

    //Getter
    /**
     * \brief Get the current toggle state
     */
    ToggleState get_state() const;
    /**
     * \brief Get the ID of the vehicle this toggle is "responsible" for
     */
    unsigned int get_id();
    /**
     * \brief Get the parent widget of this UI element, to be able to place it within another UI element
     */
    Gtk::Widget* get_parent();

    //Setter
    /**
     * \brief Set the toggle state
     * \param state The new toggle state
     */
    void set_state(ToggleState state);
    /**
     * \brief Make the toggle interactable or grey it out.
     * \param sensitive If true, make the toggle interactable, else grey it out.
     */
    void set_sensitive(bool sensitive);

    /**
     * \brief Set the toggle insensitive for the given timeout, then set it to sensitive again.
     * The caller does not need to wait for the given timeout.
     * Internally uses a thread and the GTK UI dispatcher to achieve this behaviour.
     * \param timeout_seconds Time in seconds for the toggle to not respond to input
     */
    void set_insensitive(uint timeout_seconds);

    /**
     * \brief If set, the given callback gets called on state change.
     * Params: Vehicle ID, new Toggle state
     */
    void set_selection_callback(std::function<void(unsigned int, ToggleState)> _selection_callback);

private:
    /**
     * \brief Callback that gets called when the toggle state changes.
     * Switches between simulated / off if not real, changes the toggle style (e.g. to "Turn Off"), calls selection_callback.
     */
    void on_state_changed();

    //! Stores the current toggle state
    ToggleState current_state;

    /**
     * \brief Set the right style / label / ... depending on the current state
     */
    void update_style();

    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> builder;

    //! Parent box that contains the button and the vehicle label
    Gtk::FlowBox* parent = nullptr;

    //! Label for the vehicle ID
    Gtk::Label* label = nullptr;

    //! Button / toggle
    Gtk::Button* vehicle_button = nullptr;

    //Given values
    //! Vehicle ID the toggle is responsible for
    unsigned int id;
    //! Selection callback, can be set, is called when the button / toggle was clicked
    std::function<void(unsigned int, ToggleState)> selection_callback;

    //Timing for set_insensitive
    //! To stop the set_insensitive_thread thread early, e.g. on object destruction
    std::atomic_bool signal_thread_stop;
    //! To store the current sensitivity / interactability of the toggle, for set_insensitive_thread
    std::atomic_bool is_sensitive;
    //! Thread that is used to make the toggle insensitive until a given timeout has been reached
    std::thread set_insensitive_thread;

    //! To communicate between thread and GUI
    Glib::Dispatcher ui_dispatcher; 

    /**
     * \brief Called by the UI thread, used for set_insensitive.
     * Calls set_sensitive, given is_sensitive.
     */
    void ui_dispatch();
};