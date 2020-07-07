#include "LCCErrorViewUI.hpp"

using namespace std::placeholders;
LCCErrorViewUI::LCCErrorViewUI() :
    ui_dispatcher()
{
    ui_builder = Gtk::Builder::create_from_file("ui/lcc_errors/lcc_errors.glade");

    ui_builder->get_widget("parent", parent);
    ui_builder->get_widget("error_treeview", error_treeview);
    ui_builder->get_widget("error_label_header", error_label_header);
    ui_builder->get_widget("error_scrolled_window", error_scrolled_window);
    ui_builder->get_widget("autoscroll_check_button", autoscroll_check_button);
    ui_builder->get_widget("error_button_reset", error_button_reset);

    assert(parent);
    assert(error_treeview);
    assert(error_label_header);
    assert(error_scrolled_window);
    assert(autoscroll_check_button);
    assert(error_button_reset);

    //Create model for view
    error_list_store = Gtk::ListStore::create(error_record);

    //Use model_record, add it to the view
    error_treeview->append_column("Timestamp", error_record.timestamps);
    error_treeview->append_column("Error", error_record.error_content);
    error_treeview->set_model(error_list_store);

    error_treeview->get_column(0)->set_resizable(true);
    error_treeview->get_column(0)->set_expand(true);

    //Create UI thread and register dispatcher callback
    ui_dispatcher.connect(sigc::mem_fun(*this, &LCCErrorViewUI::dispatcher_callback));
    run_thread.store(true);
    ui_thread = std::thread(&LCCErrorViewUI::update_ui, this);

    //Scroll event callback
    error_treeview->signal_scroll_event().connect(sigc::mem_fun(*this, &LCCErrorViewUI::scroll_callback));

    //Size adjustment callback (for autoscroll)
    error_treeview->signal_size_allocate().connect(sigc::mem_fun(*this, &LCCErrorViewUI::on_size_change_autoscroll));

    //Set tooltip
    error_treeview->set_has_tooltip(true);
    error_treeview->signal_query_tooltip().connect(sigc::mem_fun(*this, &LCCErrorViewUI::tooltip_callback));

    //Set reset callback
    error_button_reset->signal_clicked().connect(sigc::mem_fun(this, &LCCErrorViewUI::reset));

    //Log reset triggered by another module or the reset button
    reset_logs.store(false);
}

LCCErrorViewUI::~LCCErrorViewUI() {
    run_thread.store(false);

    if(ui_thread.joinable()) {
        ui_thread.join();
    }
}

void LCCErrorViewUI::dispatcher_callback() {
    if (reset_logs.load())
    {
        LCCErrorLogger::Instance().reset();
        reset_list_store();
        std::cout << "LOGS RESET" << std::endl;
        reset_logs.store(false);
    }
    else
    {
        for(const auto& entry : LCCErrorLogger::Instance().get_new_errors()) {
            Glib::ustring timestamp_ustring(entry.second);
            Glib::ustring error_ustring(entry.first);

            Gtk::TreeModel::Row row;
            row = *(error_list_store->append());
            
            row[error_record.timestamps] = timestamp_ustring;
            row[error_record.error_content] = error_ustring;
        }
    }
}

//Suppress warning for unused parameter
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void LCCErrorViewUI::on_size_change_autoscroll(Gtk::Allocation& allocation)
{
    if (autoscroll_check_button->get_active()) {
        auto adjustment = error_scrolled_window->get_vadjustment();
        adjustment->set_value(adjustment->get_upper() - adjustment->get_page_size());
    }
}
#pragma GCC diagnostic pop

void LCCErrorViewUI::update_ui() {
    while (run_thread.load()) {
        ui_dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void LCCErrorViewUI::reset_list_store() {
    //Get current number of elements
    size_t count = 0;
    for (auto iter = error_list_store->children().begin(); iter != error_list_store->children().end(); ++iter) {
        ++count;
    }

    //Delete them all
    for (size_t i = 0; i < count; ++i) { 
        auto iter = error_list_store->children().begin();
        error_list_store->erase(iter);
    }
}

//Suppress warning for unused parameter (scroll_event)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

bool LCCErrorViewUI::scroll_callback(GdkEventScroll* scroll_event) {
    //React to a mouse scroll event (but propagate it further)
    autoscroll_check_button->set_active(false);
    return false;
}

#pragma GCC diagnostic pop

bool LCCErrorViewUI::tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip) {
    int cell_x, cell_y = 0;
    Gtk::TreeModel::Path path;
    Gtk::TreeViewColumn* column;
    bool path_exists;

    //Get the current path and column at the selected point
    if (keyboard_tooltip) {
        error_treeview->get_cursor(path, column);
        path_exists = column != nullptr;
    }
    else {
        int window_x, window_y;
        error_treeview->convert_widget_to_bin_window_coords(x, y, window_x, window_y);
        path_exists = error_treeview->get_path_at_pos(window_x, window_y, path, column, cell_x, cell_y);
    }

    if (path_exists) {
        //Get selected row
        Gtk::TreeModel::iterator iter = error_list_store->get_iter(path);
        Gtk::TreeModel::Row row = *iter;

        //Get tooltip text depending on current column
        Glib::ustring content_ustring;
        if (column->get_title() == "Error") {
            content_ustring = Glib::ustring(row[error_record.error_content]);
        } 
        else if (column->get_title() == "Timestamp") {
            content_ustring = Glib::ustring(row[error_record.timestamps]);
        }

        //Get text at iter
        tooltip->set_text(content_ustring);
        return true;
    }
    else {
        return false;
    }
}

Gtk::Widget* LCCErrorViewUI::get_parent() {
    return parent;
}

void LCCErrorViewUI::reset()
{
    reset_logs.store(true);
}