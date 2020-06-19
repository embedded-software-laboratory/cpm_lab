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



class VehicleManualControlUi
{
    Glib::RefPtr<Gtk::Builder> builder;

    Gtk::ScrolledWindow* parent = nullptr;
    Gtk::Widget* box1 = nullptr;
    Gtk::Button* button_restart  = nullptr;
    Gtk::Button* button_stop = nullptr;
    Gtk::Entry* entry_js_device = nullptr;
    Gtk::Entry* entry_vehicle_id = nullptr;
    Gtk::ProgressBar* progressbar_throttle = nullptr;
    Gtk::ProgressBar* progressbar_steering = nullptr;
    Glib::Dispatcher m_dispatcher;
    
    shared_ptr<VehicleManualControl> vehicleManualControl = nullptr;

public:
    VehicleManualControlUi(shared_ptr<VehicleManualControl> vehicleManualControl);

    Gtk::Widget* get_parent();
    void update();
};