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
    ui_builder->get_widget("logs_search_entry", logs_search_entry);
    ui_builder->get_widget("logs_search_type", logs_search_type);

    assert(parent);
    assert(logs_treeview);
    assert(logs_label_header);
    assert(logs_scrolled_window);
    assert(autoscroll_check_button);
    assert(logs_search_entry);
    assert(logs_search_type);

    //Create model for view
    log_list_store = Gtk::ListStore::create(log_record);
    log_list_store->set_sort_column(static_cast<Gtk::TreeModelColumnBase>(log_record.log_stamp), Gtk::SORT_ASCENDING);

    //Use model_record, add it to the view
    logs_treeview->append_column("ID", log_record.log_id);
    logs_treeview->append_column("Content", log_record.log_content);
    logs_treeview->append_column("At time", log_record.log_stamp);
    logs_treeview->set_model(log_list_store);

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

    //Search objects
    filter_active.store(false); //UI dispatcher works differently when filters are applied
    search_future = search_promise.get_future();
    search_result_used.store(false); //Used to see if the future has already been used
    search_thread_running.store(false); //Can be used to stop a search thread and to check if one is currently running

    //Search callback
    logs_search_entry->signal_search_changed().connect(sigc::mem_fun(*this, &LoggerViewUI::search_changed));
    logs_search_entry->signal_stop_search().connect(sigc::mem_fun(*this, &LoggerViewUI::stop_search));

    //Search types
    logs_search_type->append(type_id_ustring);
    logs_search_type->append(type_content_ustring);
    logs_search_type->append(type_timestamp_ustring);
    logs_search_type->append(type_all_ustring);
    logs_search_type->set_active_text(type_id_ustring);
    logs_search_type->signal_changed().connect(sigc::mem_fun(*this, &LoggerViewUI::on_filter_type_changed));

    //Create UI thread and register dispatcher callback
    ui_dispatcher.connect(sigc::mem_fun(*this, &LoggerViewUI::dispatcher_callback));
    run_thread.store(true);
    ui_thread = std::thread(&LoggerViewUI::update_ui, this);
}

LoggerViewUI::~LoggerViewUI() {
    run_thread.store(false);

    if(ui_thread.joinable()) {
        ui_thread.join();
    }

    search_thread_running.store(false);

    if (search_thread.joinable()) {
        search_thread.join();
    }
}

void LoggerViewUI::dispatcher_callback() {
    //Update treeview using the newest entries if no filter is applied
    if (!filter_active.load()) {
        //Delete old logs when limit is reached
        delete_old_logs(100);

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
    else {
        //Obtain entries that match the filter - use an async thread w. future to do so
        //This way a search can be aborted if the user changes the search in between - and it does not block the UI thread
        if(search_future.valid() && !search_result_used.load()) {
            for(const auto& entry : search_future.get()) {
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

            //Update the UI accordingly
            log_list_store.clear();

            search_result_used.store(true);
        }
    }

    //TODO: 
    // - Abstract basically same function in if and else part
    // - Let UI thread know whether search_future contains actually valid search results before they are printed
    // - Clear log_list_store once and maybe show "searching" - row to let the user know that a search is currently performed
}

void LoggerViewUI::update_ui() {
    while (run_thread.load()) {
        ui_dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void LoggerViewUI::delete_old_logs(const long max_amount) {
    //Get current number of elements
    long count = 0;
    for (auto iter = log_list_store->children().begin(); iter != log_list_store->children().end(); ++iter) {
        ++count;
    }

    //Only delete if more elements are in the list_store (currently hope that iteration starts at oldest element)
    if (count > max_amount) {
        long amount_left = count - max_amount;

        for (long i = 0; i < amount_left; ++i) { 
            auto iter = log_list_store->children().begin();
            log_list_store->erase(iter);
        }
    }
}

// bool LoggerViewUI::filter_func(const Gtk::TreeModel::const_iterator& iter) {
//     auto row = *iter;
//     std::stringstream stream;
//     if (logs_search_type->get_active_text() == type_id_ustring) {
//         stream << row[log_record.log_id];
//     }
//     else if (logs_search_type->get_active_text() == type_content_ustring) {
//         stream << row[log_record.log_content];
//     }
//     else if (logs_search_type->get_active_text() == type_timestamp_ustring) {
//         stream << row[log_record.log_stamp];
//     }
//     else {
//         stream << row[log_record.log_id] << row[log_record.log_content] << row[log_record.log_stamp];
//     } 

//     std::string row_id = stream.str();
//     std::string filter_value = std::string(logs_search_entry->get_text().c_str());
//     if (row_id.find(filter_value) != string::npos) {
//         return true;
//     }
//     else {
//         return false;
//     }
// }

void LoggerViewUI::stop_search() {
    //Return to the full log list
    filter_active.store(false);
    kill_search_thread();
}

void LoggerViewUI::search_changed() {
    //Set the view to the filter model
    //Re-apply the filter
    if (logs_search_entry->get_text().size() > 0) {
        filter_active.store(true);
        start_new_search_thread();
    }
    else {
        filter_active.store(false);
        kill_search_thread();
    }
}

void LoggerViewUI::on_filter_type_changed() {
    //Check if a filter is currently applied
    if (logs_search_entry->get_text().size() > 0) {
        filter_active.store(true);
        start_new_search_thread();
    }
}

void LoggerViewUI::start_new_search_thread() {
    //Kill old thread if one exists
    kill_search_thread();

    //Get the current filter string and the filter type
    std::string filter_value = std::string(logs_search_entry->get_text().c_str());
    auto filter_type_string = logs_search_type->get_active_text();

    LogStorage::FilterType filter_type;
    if (filter_type_string == type_id_ustring) {
        filter_type = LogStorage::FilterType::ID;
    }
    else if (filter_type_string == type_content_ustring) {
        filter_type = LogStorage::FilterType::Content;
    }
    else if (filter_type_string == type_timestamp_ustring) {
        filter_type = LogStorage::FilterType::Timestamp;
    }
    else {
        filter_type = LogStorage::FilterType::All;
    } 

    search_result_used.store(false);
    search_thread_running.store(true);
    search_thread = std::thread([&]() {
        search_promise.set_value(log_storage->perform_abortable_search(filter_value, filter_type, search_thread_running));
    });
}

void LoggerViewUI::kill_search_thread() {
    search_thread_running.store(false);
    if (search_thread.joinable()) {
        search_thread.join();
    }
}

Gtk::Widget* LoggerViewUI::get_parent() {
    return parent;
}