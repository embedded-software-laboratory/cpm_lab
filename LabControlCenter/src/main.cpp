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
#include "ui/manual_control/VehicleManualControlUi.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>



int main(int argc, char *argv[])
{
    auto participant = make_shared<dds::domain::DomainParticipant>(0);
    auto vehicleManualControl = make_shared<VehicleManualControl>(participant);
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);
    VehicleManualControlUi vehicleManualControlUi(vehicleManualControl);

    //app->signal_startup().connect([&]{ app->add_window(*window2); });
    return app->run(vehicleManualControlUi.get_window());




    //dds::topic::Topic<VehicleState> topic_vehicleState (*participant, "vehicleState");
    //dds::sub::DataReader<VehicleState> reader_vehicleState(dds::sub::Subscriber(*participant), topic_vehicleState);

    //vehicleManualControl.start(0, "/dev/input/js1");

    while(1) sleep(1);
    return 0;
}