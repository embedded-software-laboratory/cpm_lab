#include "defaults.hpp"
#include "stdio.h"
#include <unistd.h>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include "TimeSeriesAggregator.hpp"
#include "VehicleManualControl.hpp"
#include "ui/monitoring/MonitoringUi.hpp"
#include "ui/manual_control/VehicleManualControlUi.hpp"
#include "ui/map_view/MapViewUi.hpp"
#include "ParameterServer.hpp"
#include "ParameterStorage.hpp"
#include "ui/MainWindow.hpp"
#include "cpm/Logging.hpp"


#include <gtkmm/builder.h>
#include <gtkmm.h>

int main(int argc, char *argv[])
{
    Logging::Instance().set_id("LabControlCenter");

    ParameterStorage storage("parameters.yaml");
    ParameterServer server(storage);


    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("ui/style.css");
    Gtk::StyleContext::create()->add_provider_for_screen (Gdk::Display::get_default()->get_default_screen(),cssProvider,500);


    auto vehicleManualControl = make_shared<VehicleManualControl>();
    TimeSeriesAggregator timeSeriesAggregator;

    auto mapViewUi = make_shared<MapViewUi>([&](){return timeSeriesAggregator.get_vehicle_data();});
    auto monitoringUi = make_shared<MonitoringUi>([&](){return timeSeriesAggregator.get_vehicle_data();});
    auto vehicleManualControlUi = make_shared<VehicleManualControlUi>(vehicleManualControl);
    auto mainWindow = make_shared<MainWindow>(vehicleManualControlUi, monitoringUi);


    vehicleManualControl->set_callback([&](){vehicleManualControlUi->update();});



    /********* Start App **********/
    app->signal_startup().connect([&]{
        app->add_window(mapViewUi->get_window());
        app->add_window(monitoringUi->get_window());
    });



    return app->run(mainWindow->get_window());
}