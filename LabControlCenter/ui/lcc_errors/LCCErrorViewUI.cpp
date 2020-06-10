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

    assert(parent);
    assert(error_treeview);
    assert(error_label_header);
    assert(error_scrolled_window);
    assert(autoscroll_check_button);

    //Create model for view
    error_list_store = Gtk::ListStore::create(error_record);

    //Use model_record, add it to the view
    error_treeview->append_column("Errors", error_record.error_content);
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

    //Log reset triggered by another module
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
        for(const std::string& entry : LCCErrorLogger::Instance().get_new_errors()) {
            Glib::ustring error_ustring(entry);

            Gtk::TreeModel::Row row;
            row = *(error_list_store->append());
            
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

Gtk::Widget* LCCErrorViewUI::get_parent() {
    return parent;
}

void LCCErrorViewUI::reset()
{
    reset_logs.store(true);
}