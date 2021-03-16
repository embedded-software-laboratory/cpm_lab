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

#include "VehicleManualControlUi.hpp"
#include <cstdlib>

/**
 * \file VehicleManualControlUi.cpp
 * \ingroup lcc_ui
 */

VehicleManualControlUi::VehicleManualControlUi(shared_ptr<VehicleManualControl> _vehicleManualControl) : vehicleManualControl(_vehicleManualControl)
{
    assert(vehicleManualControl);

    builder = Gtk::Builder::create_from_file("ui/manual_control/manual_control_ui2.glade");

    builder->get_widget("parent", parent);
    builder->get_widget("box1", box1);
    builder->get_widget("button_restart", button_restart);
    builder->get_widget("button_stop", button_stop);
    builder->get_widget("entry_js_device", entry_js_device);
    builder->get_widget("entry_vehicle_id", entry_vehicle_id);
    builder->get_widget("progressbar_throttle", progressbar_throttle);
    builder->get_widget("progressbar_steering", progressbar_steering);

    assert(parent);
    assert(box1);
    assert(button_restart);
    assert(button_stop);
    assert(entry_js_device);
    assert(entry_vehicle_id);
    assert(progressbar_throttle);
    assert(progressbar_steering);


    button_restart->signal_clicked().connect([&]()
    {
        try 
        {
            vehicleManualControl->start(
                uint8_t(std::atoi(entry_vehicle_id->get_text().c_str())),
                entry_js_device->get_text()
            );
        }
        catch (const std::exception& e) 
        {
            std::cerr << e.what() << endl;
        }
        //std::cout << entry_js_device->get_text() << std::endl;        
    });

    button_stop->signal_clicked().connect([&]()
    {
        vehicleManualControl->stop();        
    });

    m_dispatcher.connect([&](){
        if(vehicleManualControl) {
            double throttle = 0;
            double steering = 0;
            vehicleManualControl->get_state(throttle, steering);
            progressbar_throttle->set_fraction(throttle * 0.5 + 0.5);
            progressbar_steering->set_fraction(-steering * 0.5 + 0.5);
        }
    });

    /*window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        vehicleManualControl->stop();
        exit(0);
        return false;
    });*/

}



Gtk::Widget* VehicleManualControlUi::get_parent()
{
    return parent;
}

void VehicleManualControlUi::update() 
{
    m_dispatcher.emit();
}