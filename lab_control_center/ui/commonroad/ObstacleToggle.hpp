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

#include <functional>
#include <iostream>
#include <sstream>

/**
 * \brief This class contains all UI elements that control the desired setup state for one obstacle.
 * An obstacle can either be Simulated (shown in the MapView) or On (Its trajectory is sent to a vehicle with the same ID).
 * This allows for less redundant UI code.
 * \ingroup lcc_ui
 */
class ObstacleToggle 
{
public:
    /**
     * \brief Constructor to create an obstacle toggle UI element
     * \param _id ID of the obstacle that should be toggled between trajectory or visualization/simulation in the MapView
     */
    ObstacleToggle(unsigned int _id);

    /**
     * \enum ToggleState
     * \brief The possible states of an object: Simulated (shown as object in the MapView) or On (Trajectory that is sent to a vehicle with the same ID)
     */
    enum ToggleState{Simulated, On};

    //Getter
    /**
     * \brief Get the parent widget of the toggle, to integrate the UI element into another UI part
     */
    Gtk::Widget* get_parent();
    /**
     * \brief Get the current toggle state
     */
    ToggleState get_state();
    /**
     * \brief Get the ID for which the toggle state was set
     */
    unsigned int get_id();

    //Setter
    /**
     * \brief Set the state of the obstacle toggle to be simulated (shown in the MapView) or on (send a trajectory)
     * \param state The state to set the toggle to
     */
    void set_state(ToggleState state);
    /**
     * \brief Set a callback that gets called on state change
     * \param _selection_callback Callback function that gets called
     */
    void set_selection_callback(std::function<void(unsigned int, ToggleState)> _selection_callback);

private:
    /**
     * \brief Internal callback function that reacts to a state change of the toggle.
     * Internally updates current_state and calls selection_callback.
     */
    void on_state_changed();

    //! The current toggle state
    ToggleState current_state;

    //! UI builder object
    Glib::RefPtr<Gtk::Builder> builder;

    //! Box that contains the obstacle label and switch
    Gtk::FlowBox* parent = nullptr;

    //! Label for the switch, i.e. obstacle ID
    Gtk::Label* label = nullptr;

    //! Actual switch to toggle between the possible state
    Gtk::Switch* obstacle_switch = nullptr;

    //Given values
    //! ID of the obstacle
    unsigned int id;
    //! Callback function that can be set externally, to be called when the switch state changes
    std::function<void(unsigned int, ToggleState)> selection_callback;
};