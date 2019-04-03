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


#include <gtkmm/builder.h>
#include <gtkmm.h>

int main(int argc, char *argv[])
{
    ParameterStorage storage("parameters.yaml");
    ParameterServer server(storage);


    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv);
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("ui/style.css");
    Gtk::StyleContext::create()->add_provider_for_screen (Gdk::Display::get_default()->get_default_screen(),cssProvider,500);

    auto vehicleManualControl = make_shared<VehicleManualControl>();

    TimeSeriesAggregator timeSeriesAggregator;


    MapViewUi mapViewUi(timeSeriesAggregator.get_vehicle_data());
    MonitoringUi monitoringUi(timeSeriesAggregator.get_vehicle_data());


    auto vehicleManualControlUi = make_shared<VehicleManualControlUi>(vehicleManualControl);

    //VehicleManualControlUi vehicleManualControlUi(vehicleManualControl);

    vehicleManualControl->set_callback([&](){vehicleManualControlUi->update();});




    MainWindow mainWindow(vehicleManualControlUi);



    /********* Start App **********/
    app->signal_startup().connect([&]{
        //app->add_window(vehicleManualControlUi.get_window());
        //app->add_window(monitoringUi.get_window());
        app->add_window(mapViewUi.get_window());
        app->add_window(mainWindow.get_window());
    });



    return app->run(monitoringUi.get_window());
}