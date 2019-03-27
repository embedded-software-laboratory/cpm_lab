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
    VehicleManualControlUi vehicleManualControlUi(vehicleManualControl);

    vehicleManualControl->set_callback([&](){vehicleManualControlUi.update();});



    /******** Main Window *******/
    Glib::RefPtr<Gtk::Builder> builder_master_layout = Gtk::Builder::create_from_file("ui/master_layout.glade");

    Gtk::Window* window_LCC = nullptr;
    Gtk::Box* box_manual_control = nullptr;
    Gtk::Box* box_map = nullptr;
    Gtk::Box* box_data_grid = nullptr;
    Gtk::Paned* pane1 = nullptr;
    Gtk::Paned* pane2 = nullptr;
    builder_master_layout->get_widget("window_LCC", window_LCC);
    builder_master_layout->get_widget("box_manual_control", box_manual_control);
    builder_master_layout->get_widget("box_map", box_map);
    builder_master_layout->get_widget("box_data_grid", box_data_grid);
    builder_master_layout->get_widget("paned1", pane1);
    builder_master_layout->get_widget("paned2", pane2);


    assert(window_LCC);
    assert(box_manual_control);
    assert(box_map);
    assert(box_data_grid);
    assert(pane1);
    assert(pane2);

    window_LCC->show();
    window_LCC->set_size_request(800, 600);


    Gtk::Label* label = Gtk::manage(new Gtk::Label());

    box_map->pack_start(*label,true,true);
    box_data_grid->pack_start(*label,true,true);

    label->set_text("asdasd");

    label->set_width_chars(10);
    label->set_xalign(1);
    label->show_all();


    Glib::RefPtr<Gtk::Builder> builder_manual_control_ui = Gtk::Builder::create_from_file("ui/manual_control/manual_control_ui2.glade");

    Gtk::Box* box1 = nullptr;
    builder_manual_control_ui->get_widget("box1", box1);
    box_manual_control->pack_start(*box1,true,true);
    box1->show();





    /********* Start App **********/
    app->signal_startup().connect([&]{
        app->add_window(monitoringUi.get_window());
        app->add_window(mapViewUi.get_window());
        app->add_window(*window_LCC);
    });


    app->signal_activate().connect([&]{
        pane1->set_position(400);
        pane2->set_position(400);
    });


    return app->run(vehicleManualControlUi.get_window());
}