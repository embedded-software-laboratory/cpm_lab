#include "LoggerViewUI.hpp"

using namespace std::placeholders;
LoggerViewUI::LoggerViewUI(std::shared_ptr<LogStorage> logStorage) :
    ui_dispatcher(),
    log_storage(logStorage)
{
    ui_builder = Gtk::Builder::create_from_file("ui/logger/logger.glade");

    ui_builder->get_widget("parent", parent);
    ui_builder->get_widget("logs_treeview", logs_treeview);
    ui_builder->get_widget("logs_label_header", logs_label_header);
    ui_builder->get_widget("logs_scrolled_window", logs_scrolled_window);
    ui_builder->get_widget("autoscroll_check_button", autoscroll_check_button);

    assert(parent);
    assert(logs_treeview);
    assert(logs_label_header);
    assert(logs_scrolled_window);
    assert(autoscroll_check_button);

    //Create model for view
    log_list_store = Gtk::ListStore::create(log_record);
    log_list_store->set_sort_column(static_cast<Gtk::TreeModelColumnBase>(log_record.log_stamp), Gtk::SORT_ASCENDING);

    //Use model_record, add it to the view
    logs_treeview->append_column("ID", log_record.log_id);
    logs_treeview->append_column("Content", log_record.log_content);
    logs_treeview->append_column("At time", log_record.log_stamp);

    //Set columns resize property
    for (int i = 0; i < 3; ++i) {
        logs_treeview->get_column(i)->set_resizable(true);
        logs_treeview->get_column(i)->set_expand(true);
    }
    //Set column widths
    logs_treeview->get_column(0)->set_min_width(10);
    logs_treeview->get_column(0)->set_fixed_width(10);
    logs_treeview->get_column(1)->set_min_width(30);
    logs_treeview->get_column(1)->set_fixed_width(80);
    logs_treeview->get_column(2)->set_min_width(10);
    logs_treeview->get_column(2)->set_fixed_width(10);

    //Create thread and register dispatcher callback
    ui_dispatcher.connect(sigc::mem_fun(*this, &LoggerViewUI::dispatcher_callback));
    run_thread.store(true);
    ui_thread = std::thread(&LoggerViewUI::update_ui, this);

    //Filtering
    filter = Gtk::TreeModelFilter::create(log_list_store);
    filter->set_visible_func(sigc::mem_fun(*this, &LoggerViewUI::filter_func));
    logs_treeview->set_model(filter);
}

LoggerViewUI::~LoggerViewUI() {
    run_thread.store(false);

    if(ui_thread.joinable()) {
        ui_thread.join();
    }
}

void LoggerViewUI::dispatcher_callback() {
    //Update treeview
    for(const auto& entry : log_storage->get_new_logs()) {
        Glib::ustring log_id_ustring(entry.id());
        Glib::ustring log_msg_ustring(entry.content());

        Gtk::TreeModel::Row row;
        row = *(log_list_store->append());
        
        row[log_record.log_id] = log_id_ustring;
        row[log_record.log_content] = log_msg_ustring;
        row[log_record.log_stamp] = entry.stamp().nanoseconds();
    }

    if (autoscroll_check_button->get_active()) {
        auto adjustment = logs_scrolled_window->get_vadjustment();
        adjustment->set_value(adjustment->get_upper() - adjustment->get_page_size());
    }
}

void LoggerViewUI::update_ui() {
    while (run_thread.load()) {
        ui_dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

bool LoggerViewUI::filter_func(const Gtk::TreeModel::const_iterator& iter) {
    auto row = *iter;
    std::stringstream stream;
    stream << row[log_record.log_id];
    std::string row_id = stream.str();
    if (row_id.find("middleware") != string::npos) {
        std::cout << "False" << std::endl;
        return false;
    }
    else {
        std::cout << "True" << std::endl;
        return true;
    }
}

Gtk::Widget* LoggerViewUI::get_parent() {
    return parent;
}