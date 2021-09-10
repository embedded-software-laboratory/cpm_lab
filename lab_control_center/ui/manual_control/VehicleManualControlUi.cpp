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