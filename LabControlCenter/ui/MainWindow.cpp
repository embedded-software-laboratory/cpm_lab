#include "MainWindow.hpp"

using namespace std::placeholders;

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
    //Make according buttons unusable until the ui is closed; also grey out param tab / treeview content
    menu_bar_params_reload->set_sensitive(false);
    menu_bar_params_save->set_sensitive(false);
    menu_bar_params_save_as->set_sensitive(false);
    menu_bar_params_load_file->set_sensitive(false);
    menu_bar_params_load_multiple_files->set_sensitive(false);
    menu_bar_params_load_params->set_sensitive(false);
    tabs_view_ui->get_param_view()->make_insensitive();

    file_saver_window = make_shared<FileSaverUI>(std::bind(&MainWindow::file_saver_callback, this, _1, _2));
}

void MainWindow::on_menu_params_load_file_pressed() {
    //Make according buttons unusable until the ui is closed; also grey out param tab / treeview content
    menu_bar_params_reload->set_sensitive(false);
    menu_bar_params_save->set_sensitive(false);
    menu_bar_params_save_as->set_sensitive(false);
    menu_bar_params_load_file->set_sensitive(false);
    menu_bar_params_load_multiple_files->set_sensitive(false);
    menu_bar_params_load_params->set_sensitive(false);
    tabs_view_ui->get_param_view()->make_insensitive();

    file_chooser_window = make_shared<FileChooserUI>(std::bind(&MainWindow::file_chooser_callback, this, _1, _2));
}

void MainWindow::on_menu_params_load_multiple_files_pressed() {
    tabs_view_ui->get_param_view()->params_load_multiple_files_handler();
}

void MainWindow::on_menu_params_load_params_pressed() {
    tabs_view_ui->get_param_view()->params_load_params_handler();
}

void MainWindow::file_chooser_callback(std::string file_string, bool has_file) {
    std::cout << "called" << std::endl;

    //Make according buttons usable as the ui is closed, also for treeview content from param ui
    menu_bar_params_reload->set_sensitive(true);
    menu_bar_params_save->set_sensitive(true);
    menu_bar_params_save_as->set_sensitive(true);
    menu_bar_params_load_file->set_sensitive(true);
    menu_bar_params_load_multiple_files->set_sensitive(true);
    menu_bar_params_load_params->set_sensitive(true);
    tabs_view_ui->get_param_view()->make_sensitive();

    //Call Param UI to process changes
    if (has_file) {
        tabs_view_ui->get_param_view()->params_load_file_handler(file_string);
    }
}

void MainWindow::file_saver_callback(std::string file_string, bool has_file) {
    //Make according buttons usable as the ui is closed, also for treeview content from param ui
    menu_bar_params_reload->set_sensitive(true);
    menu_bar_params_save->set_sensitive(true);
    menu_bar_params_save_as->set_sensitive(true);
    menu_bar_params_load_file->set_sensitive(true);
    menu_bar_params_load_multiple_files->set_sensitive(true);
    menu_bar_params_load_params->set_sensitive(true);
    tabs_view_ui->get_param_view()->make_sensitive();

    //Call Param UI to process changes
    if (has_file) {
        tabs_view_ui->get_param_view()->params_save_as_handler(file_string);
    }
}

Gtk::Window& MainWindow::get_window()
{
    return *window_LCC;
}
