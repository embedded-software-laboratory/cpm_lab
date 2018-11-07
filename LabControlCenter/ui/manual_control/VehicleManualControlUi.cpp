#include "VehicleManualControlUi.hpp"
#include <cstdlib>


VehicleManualControlUi::VehicleManualControlUi(shared_ptr<VehicleManualControl> _vehicleManualControl) : vehicleManualControl(_vehicleManualControl)
{
    assert(vehicleManualControl);

    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/manual_control/manual_control_ui.glade");

    builder->get_widget("window1", window);
    builder->get_widget("button_restart", button_restart);
    builder->get_widget("button_stop", button_stop);
    builder->get_widget("entry_js_device", entry_js_device);
    builder->get_widget("entry_vehicle_id", entry_vehicle_id);
    builder->get_widget("progressbar_throttle", progressbar_throttle);
    builder->get_widget("progressbar_steering", progressbar_steering);

    assert(window);
    assert(button_restart);
    assert(button_stop);
    assert(entry_js_device);
    assert(entry_vehicle_id);
    assert(progressbar_throttle);
    assert(progressbar_steering);

    window->show();
    window->set_size_request(350, 200);

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

    window->signal_delete_event().connect([&](GdkEventAny*)->bool{
        vehicleManualControl->stop();
        exit(0);
        return false;
    });

}


Gtk::Window& VehicleManualControlUi::get_window() 
{
    return *window;
}


void VehicleManualControlUi::update() 
{
    m_dispatcher.emit();
}