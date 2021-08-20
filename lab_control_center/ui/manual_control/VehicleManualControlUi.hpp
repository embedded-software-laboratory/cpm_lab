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

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include "VehicleManualControl.hpp"


/**
 * \class VehicleManualControlUi
 * \brief UI class for controlling the vehicle manually (choose vehicle, use Joystick / Gamepad)
 * \ingroup lcc_ui
 */
class VehicleManualControlUi
{
    //! GTK UI builder
    Glib::RefPtr<Gtk::Builder> builder;

    //! Parent of the UI element, allows scrolling within
    Gtk::ScrolledWindow* parent = nullptr;
    //! Contains all other elements in form of a list
    Gtk::Widget* box1 = nullptr;
    //! Button to (re)start manual control for the selected vehicle
    Gtk::Button* button_restart  = nullptr;
    //! Button to stop manual control for the selected vehicle
    Gtk::Button* button_stop = nullptr;
    //! Entry for joystick device
    Gtk::Entry* entry_js_device = nullptr;
    //! Entry for ID of vehicle to control
    Gtk::Entry* entry_vehicle_id = nullptr;
    //! Progress bar that shows amount of throttle
    Gtk::ProgressBar* progressbar_throttle = nullptr;
    //! Progress bar that shows "amount" of steering
    Gtk::ProgressBar* progressbar_steering = nullptr;
    //! Connects to GTK's UI thread
    Glib::Dispatcher m_dispatcher;
    
    //! Links to manual control instance which provides control input that can be shown then e.g. in the progress bars
    shared_ptr<VehicleManualControl> vehicleManualControl = nullptr;

public:
    /**
     * \brief Constructor, maps e.g. start button to manual control action from vehicleManualControl, also sets up m_dispatcher for UI updates
     * \param vehicleManualControl Reference to class for manual control of the vehicle 
     */
    VehicleManualControlUi(shared_ptr<VehicleManualControl> vehicleManualControl);

    ~VehicleManualControlUi() {
        std::cout << "!!! --- VehicleManualControlUi destructor" << std::endl;
    }

    /**
     * \brief Function to get the parent widget, so that this UI element can be placed within another UI element
     */
    Gtk::Widget* get_parent();

    /**
     * \brief UI Update function, triggers m_dispatcher to update progress bar etc.
     */
    void update();
};