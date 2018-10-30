#include "VehicleManualControlUi.hpp"
#include <cstdlib>


VehicleManualControlUi::VehicleManualControlUi(shared_ptr<VehicleManualControl> vehicleManualControl) : vehicleManualControl(vehicleManualControl)
{
    assert(vehicleManualControl);

    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui/manual_control/test.glade");

    builder->get_widget("window1", window);
    builder->get_widget("button_restart", button_restart);
    builder->get_widget("button_stop", button_stop);
    builder->get_widget("entry_js_device", entry_js_device);
    builder->get_widget("entry_vehicle_id", entry_vehicle_id);

    assert(window);
    assert(button_restart);
    assert(button_stop);
    assert(entry_js_device);
    assert(entry_vehicle_id);

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

}


Gtk::Window& VehicleManualControlUi::get_window() 
{
    return *window;
}