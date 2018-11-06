#include "defaults.hpp"
#include "stdio.h"
#include "Joystick.hpp"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "VehicleCommand.hpp"
#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"
#include "example_trajectory.hpp"
#include "VehicleManualControl.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/manual_control/VehicleManualControlUi.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>



int main(int argc, char *argv[])
{
    auto participant = make_shared<dds::domain::DomainParticipant>(0);
    auto vehicleManualControl = make_shared<VehicleManualControl>(participant);
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);
    VehicleManualControlUi vehicleManualControlUi(vehicleManualControl);
    vehicleManualControl->set_callback([&](){vehicleManualControlUi.update();});

    MonitoringUi monitoringUi;

    app->signal_startup().connect([&]{ app->add_window(vehicleManualControlUi.get_window()); });
    return app->run(monitoringUi.get_window());

    while(1) sleep(1);
    return 0;
}