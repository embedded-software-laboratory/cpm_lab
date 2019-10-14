#include "SetupViewUI.hpp"
#include <cstdlib>


SetupViewUI::SetupViewUI(shared_ptr<VehicleManualControl> _vehicleManualControl) : 
    vehicleManualControl(_vehicleManualControl)
{
    assert(vehicleManualControl);

    builder = Gtk::Builder::create_from_file("ui/setup/setup.glade");

    builder->get_widget("setup_box", parent);

    assert(parent);
}



Gtk::Widget* SetupViewUI::get_parent()
{
    return parent;
}