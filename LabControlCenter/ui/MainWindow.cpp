#include "MainWindow.hpp"



MainWindow::MainWindow(
    std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi,
    std::shared_ptr<MonitoringUi> monitoringUi
)
{
    builder_master_layout = Gtk::Builder::create_from_file("ui/master_layout.glade");



    builder_master_layout->get_widget("window_LCC", window_LCC);
    builder_master_layout->get_widget("paned1", pane1);
    builder_master_layout->get_widget("paned2", pane2);


    assert(window_LCC);
    assert(pane1);
    assert(pane2);

    window_LCC->show();
    window_LCC->set_size_request(800, 600);


    pane2->pack2(*(vehicleManualControlUi->get_parent()),true,true);
    pane2->pack1(*(monitoringUi->get_parent()),true,true);


    window_LCC->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });

    Glib::signal_timeout().connect([&]()->bool{
        pane1->set_position(400);
        pane2->set_position(400);
        return false;
    }, 200);
}


Gtk::Window& MainWindow::get_window()
{
    return *window_LCC;
}
