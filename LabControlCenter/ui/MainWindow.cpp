#include "MainWindow.hpp"



MainWindow::MainWindow(
    std::shared_ptr<TabsViewUI> tabsViewUI,
    std::shared_ptr<MonitoringUi> monitoringUi,
    std::shared_ptr<MapViewUi> mapViewUi
) :
    tabs_view_ui(tabsViewUI),
    monitoring_ui(monitoringUi),
    map_view_ui(mapViewUi)
{
    std::cout << "Builder started" << std::endl;
    builder_master_layout = Gtk::Builder::create_from_file("ui/master_layout.glade");

    std::cout << "MainWindow started" << std::endl;


    builder_master_layout->get_widget("window_LCC", window_LCC);
    builder_master_layout->get_widget("box", box);
    builder_master_layout->get_widget("paned1", pane1);
    builder_master_layout->get_widget("paned2", pane2);

    builder_master_layout->get_widget("menu_bar", menu_bar);
    builder_master_layout->get_widget("menu_bar_params_reload", menu_bar_params_reload);
    builder_master_layout->get_widget("menu_bar_params_save", menu_bar_params_save);
    builder_master_layout->get_widget("menu_bar_params_save_as", menu_bar_params_save_as);
    builder_master_layout->get_widget("menu_bar_params_load_file", menu_bar_params_load_file);
    builder_master_layout->get_widget("menu_bar_params_load_multiple_files", menu_bar_params_load_multiple_files);
    builder_master_layout->get_widget("menu_bar_params_load_params", menu_bar_params_load_params);


    assert(window_LCC);
    assert(box);
    assert(pane1);
    assert(pane2);

    assert(menu_bar);
    assert(menu_bar_params_reload);
    assert(menu_bar_params_save);
    assert(menu_bar_params_save_as);
    assert(menu_bar_params_load_file);
    assert(menu_bar_params_load_multiple_files);
    assert(menu_bar_params_load_params);

    //Show window, set size depending on monitor resolution
    window_LCC->show();
    int screen_width = window_LCC->get_screen()->get_width();
    int screen_height = window_LCC->get_screen()->get_height();
    window_LCC->set_default_size(3/4 * screen_width, 3/4 * screen_height);
    window_LCC->maximize();
    window_LCC->add_events(Gdk::SCROLL_MASK);
    

    pane2->pack2(*(tabsViewUI->get_parent()),true,true);
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

    //Parameter Menu Bar signal handlers
    menu_bar_params_reload->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_reload_pressed));
    menu_bar_params_save->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_save_pressed));
    menu_bar_params_save_as->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_save_as_pressed));
    menu_bar_params_load_file->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_load_file_pressed));
    menu_bar_params_load_multiple_files->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_load_multiple_files_pressed));
    menu_bar_params_load_params->signal_activate().connect(sigc::mem_fun(this, &MainWindow::on_menu_params_load_params_pressed));

    std::cout << "MainWindow done" << std::endl;
}

void MainWindow::on_menu_params_reload_pressed() {
    tabs_view_ui->get_param_view()->params_reload_handler();
}

void MainWindow::on_menu_params_save_pressed() {
    tabs_view_ui->get_param_view()->params_save_handler();
}

void MainWindow::on_menu_params_save_as_pressed() {
    tabs_view_ui->get_param_view()->params_save_as_handler();
}

void MainWindow::on_menu_params_load_file_pressed() {
    //Code taken from official gtkmm example, modified
    Gtk::FileChooserDialog dialog("Please choose a parameter file", Gtk::FILE_CHOOSER_ACTION_OPEN);
    dialog.set_transient_for(*window_LCC);

    //Add response buttons the the dialog:
    dialog.add_button("Cancel", Gtk::RESPONSE_CANCEL);
    dialog.add_button("Open", Gtk::RESPONSE_OK);

    //Add filters, so that only certain file types can be selected:
    auto filter_cpp = Gtk::FileFilter::create();
    filter_cpp->set_name("yaml files");
    filter_cpp->add_mime_type("text/yaml");
    dialog.add_filter(filter_cpp);

    //Show the dialog and wait for a user response:
    int result = dialog.run();

    //Handle the response:
    std::string filename = "";
    switch(result)
    {
        case Gtk::RESPONSE_OK:
            std::cout << "Open clicked." << std::endl;

            //Notice that this is a std::string, not a Glib::ustring.
            filename = dialog.get_filename();
            std::cout << "File selected: " <<  filename << std::endl;
            break;
        case Gtk::RESPONSE_CANCEL:
            std::cout << "Cancel clicked." << std::endl;
            break;
        default:
            std::cout << "Unexpected button clicked." << std::endl;
            break;
    }

    tabs_view_ui->get_param_view()->params_load_file_handler();
}

void MainWindow::on_menu_params_load_multiple_files_pressed() {
    tabs_view_ui->get_param_view()->params_load_multiple_files_handler();
}

void MainWindow::on_menu_params_load_params_pressed() {
    tabs_view_ui->get_param_view()->params_load_params_handler();
}



Gtk::Window& MainWindow::get_window()
{
    return *window_LCC;
}
