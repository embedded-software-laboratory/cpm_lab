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
    builder_master_layout->get_widget("box", box);
    builder_master_layout->get_widget("paned1", pane1);
    builder_master_layout->get_widget("paned2", pane2);


    assert(window_LCC);
    assert(menu_bar);
    assert(box);
    assert(pane1);
    assert(pane2);

    //Show window, set size depending on monitor resolution
    window_LCC->show();
    int screen_width = window_LCC->get_screen()->get_width();
    int screen_height = window_LCC->get_screen()->get_height();
    window_LCC->set_default_size(3/4 * screen_width, 3/4 * screen_height);
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

        pane1->set_position(height-300);
        pane2->set_position(width-300);
        return false;
    }, 200);
}


Gtk::Window& MainWindow::get_window()
{
    return *window_LCC;
}
