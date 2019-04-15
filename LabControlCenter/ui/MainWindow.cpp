#include "MainWindow.hpp"



MainWindow::MainWindow(
    std::shared_ptr<VehicleManualControlUi> vehicleManualControlUi,
    std::shared_ptr<MonitoringUi> monitoringUi,
    std::shared_ptr<MapViewUi> mapViewUi
)
{
    builder_master_layout = Gtk::Builder::create_from_file("ui/master_layout.glade");



    builder_master_layout->get_widget("window_LCC", window_LCC);
    builder_master_layout->get_widget("menu_bar", menu_bar);
    builder_master_layout->get_widget("paned0", pane0);
    builder_master_layout->get_widget("paned1", pane1);
    builder_master_layout->get_widget("paned2", pane2);


    assert(window_LCC);
    assert(menu_bar);
    assert(pane0);
    assert(pane1);
    assert(pane2);

    window_LCC->show();
    window_LCC->set_size_request(1800, 900);
    window_LCC->maximize();
    window_LCC->add_events(Gdk::SCROLL_MASK);


    pane2->pack2(*(vehicleManualControlUi->get_parent()),true,true);
    pane1->pack2(*(monitoringUi->get_parent()),true,true);
    pane2->pack1(*(mapViewUi->get_parent()),true,true);


    window_LCC->signal_delete_event().connect([&](GdkEventAny*)->bool{
        exit(0);
        return false;
    });

    Glib::signal_timeout().connect([&]()->bool{
        int width = 0;
        int height = 0;
        window_LCC->get_size(width, height);

        pane0->set_position(20);
        pane1->set_position(height-300);
        pane2->set_position(width-300);
        return false;
    }, 200);
}


Gtk::Window& MainWindow::get_window()
{
    return *window_LCC;
}
