#include "defaults.hpp"
#include "stdio.h"
#include "Joystick.hpp"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "VehicleCommand.hpp"
#include "TimeSeriesAggregator.hpp"
#include "VehicleState.hpp"
#include "AbsoluteTimer.hpp"
#include "example_trajectory.hpp"
#include "VehicleManualControl.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/map_view/MapViewUi.hpp"

#include <gtkmm/builder.h>
#include <gtkmm.h>
#include <gdkmm/display.h>

int main(int argc, char *argv[])
{
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("ui/style.css");
    Gtk::StyleContext::create()->add_provider_for_screen (Gdk::Display::get_default()->get_default_screen(),cssProvider,500);


    auto participant = make_shared<dds::domain::DomainParticipant>(0);
    auto vehicleManualControl = make_shared<VehicleManualControl>(participant);

    TimeSeriesAggregator timeSeriesAggregator(participant);


    MapViewUi mapViewUi(timeSeriesAggregator.get_vehicle_data());
    MonitoringUi monitoringUi(timeSeriesAggregator.get_vehicle_data());
    VehicleManualControlUi vehicleManualControlUi(vehicleManualControl);

    vehicleManualControl->set_callback([&](){vehicleManualControlUi.update();});

    app->signal_startup().connect([&]{
        app->add_window(monitoringUi.get_window());
        app->add_window(mapViewUi.get_window());
    });
    return app->run(vehicleManualControlUi.get_window());
}