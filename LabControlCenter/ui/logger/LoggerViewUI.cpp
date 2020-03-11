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
    ui_builder->get_widget("log_level_combobox", log_level_combobox);

    assert(parent);
    assert(logs_treeview);
    assert(logs_label_header);
    assert(logs_scrolled_window);
    assert(autoscroll_check_button);
    assert(logs_search_entry);
    assert(logs_search_type);
    assert(log_level_combobox);

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
        //logs_treeview->get_column(i)->set_sizing(Gtk::TreeViewColumnSizing::TREE_VIEW_COLUMN_AUTOSIZE);
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
    search_reset.store(false);
    search_future = search_promise.get_future();
    search_thread_running.store(false); //Can be used to stop a search thread and to check if one is currently running

    //Search callback
    logs_search_entry->signal_search_changed().connect(sigc::mem_fun(*this, &LoggerViewUI::search_changed));
    logs_search_entry->signal_stop_search().connect(sigc::mem_fun(*this, &LoggerViewUI::stop_search));

    //Search types
    logs_search_type->append(type_id_ustring);
    logs_search_type->append(type_content_ustring);
    logs_search_type->append(type_timestamp_ustring);
    logs_search_type->append(type_all_ustring);
    logs_search_type->set_active_text(type_all_ustring);
    logs_search_type->signal_changed().connect(sigc::mem_fun(*this, &LoggerViewUI::on_filter_type_changed));

    //Create and set labels for the log_level_combobox
    for (unsigned short i = 0; i <= log_levels; ++i)
    {
        std::stringstream level_stream;
        level_stream << i;
        Glib::ustring level_string = Glib::ustring(level_stream.str());
        log_level_labels.push_back(level_string);
        log_level_combobox->append(level_string);
    }
    //Set default label for log_level_combobox
    if (log_levels == 0)
    {
        log_level_combobox->set_active_text(log_level_labels.at(0));
    }
    else 
    {
        log_level_combobox->set_active_text(log_level_labels.at(1));
    }
    //Set callback for log_level_combobox
    log_level_combobox->signal_changed().connect(sigc::mem_fun(*this, &LoggerViewUI::on_log_level_changed));

    //Tooltip callbacks (if content is too long, text on hover)
    logs_treeview->set_has_tooltip(true);
    logs_treeview->signal_query_tooltip().connect(sigc::mem_fun(*this, &LoggerViewUI::tooltip_callback));

    //Create UI thread and register dispatcher callback
    ui_dispatcher.connect(sigc::mem_fun(*this, &LoggerViewUI::dispatcher_callback));
    run_thread.store(true);
    ui_thread = std::thread(&LoggerViewUI::update_ui, this);

    //Scroll event callback
    logs_treeview->signal_scroll_event().connect(sigc::mem_fun(*this, &LoggerViewUI::scroll_callback));

    //Log reset triggered by another module
    reset_logs.store(false);
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

void LoggerViewUI::on_log_level_changed()
{
    //Get the newly set log level
    auto set_log_level = log_level_combobox->get_active_text();

    //Find its position in the log_level vector (which is equal to the log_level, see constructor)
    auto pos_it = std::find(log_level_labels.begin(), log_level_labels.end(), set_log_level);
    unsigned short log_level;
    if (pos_it != log_level_labels.end())
    {
        log_level = std::distance(log_level_labels.begin(), pos_it);
    }
    else
    {
        cpm::Logging::Instance().write(1, "ERROR: Log level set that does not exist!");
        log_level = 1;
    }
    
    //Set the new log level
    LogLevelSetter::Instance().set_log_level(log_level);
}

void LoggerViewUI::dispatcher_callback() {
    if (reset_logs.load())
    {
        log_storage->reset();
        reset_list_store();
        std::cout << "LOGS RESET" << std::endl;
        reset_logs.store(false);
    }
    else
    {
        //Update treeview using the newest entries if no filter is applied
        if (!filter_active.load()) {
            //Delete old logs when limit is reached
            delete_old_logs(max_log_amount);

            //Add only new entries if the whole log list is shown, or add all again after a search was performed
            if (search_reset.load()) {
                search_reset.store(false);
                reset_list_store();

                for(const auto& entry : log_storage->get_recent_logs(max_log_amount)) {
                    add_log_entry(entry);
                }
            }
            else {
                for(const auto& entry : log_storage->get_new_logs()) {
                    add_log_entry(entry);
                }
            }
        }
        else {
            //Obtain entries that match the filter - use an async thread w. future to do so
            //This way a search can be aborted if the user changes the search in between - and it does not block the UI thread
            //Lock mutex so that, when the future is valid, the promise can not be reset until it has been obtained
            std::unique_lock<std::mutex> lock(promise_reset_mutex);
            if(search_future.valid()) {
                //log_list_store.clear(); -> Sorgt nach Aufruf für Absturz bei append()!
                reset_list_store();
                Gtk::TreeModel::Row search_row;
                search_row = *(log_list_store->append());   
                search_row[log_record.log_id] = "";
                search_row[log_record.log_content] = "Searching...";
                search_row[log_record.log_stamp] = 0;

                if (search_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    std::vector<Log> search_results = search_future.get();

                    //Update the UI accordingly   
                    reset_list_store();            
                    for(const auto& entry : search_results) {
                        add_log_entry(entry);
                    }
                }
            }
        }
    }

    if (autoscroll_check_button->get_active()) {
        auto adjustment = logs_scrolled_window->get_vadjustment();
        adjustment->set_value(adjustment->get_upper() - adjustment->get_page_size());
    }
    //TODO: 
    // - More elegant solution for "searching...?"
    // - Search is never refreshed automatically - refresh button?
}

void LoggerViewUI::add_log_entry(const Log& entry) {
    //Note: We get a pango UTF-8 warning depending on which strings we are adding - we must make sure that they are UTF-8 encoded
    //This is done in LogStorage - a warning is added to the log, s.t. a user can correct it
    //The log is not deleted (the Gtk warning still shows up) s.t. the user can easier find out where to find their error
    std::string log_id_string(entry.id());
    std::string log_msg_string(entry.content());

    Glib::ustring log_id_ustring(log_id_string);
    Glib::ustring log_msg_ustring(log_msg_string);

    Gtk::TreeModel::Row row;
    row = *(log_list_store->append());
    
    row[log_record.log_id] = log_id_ustring;
    row[log_record.log_content] = log_msg_ustring;
    row[log_record.log_stamp] = entry.stamp().nanoseconds();
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

void LoggerViewUI::reset_list_store() {
    //Get current number of elements
    size_t count = 0;
    for (auto iter = log_list_store->children().begin(); iter != log_list_store->children().end(); ++iter) {
        ++count;
    }

    //Delete them all
    for (size_t i = 0; i < count; ++i) { 
        auto iter = log_list_store->children().begin();
        log_list_store->erase(iter);
    }
}

void LoggerViewUI::stop_search() {
    //Return to the full log list
    filter_active.store(false);
    search_reset.store(true);
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
        stop_search();
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

    search_thread_running.store(true);
    
    //Reset promise, lock mutex because UI might access it exactly when it is invalidated
    std::unique_lock<std::mutex> lock(promise_reset_mutex);
    search_promise = std::promise<std::vector<Log>>();
    search_future = search_promise.get_future();
    lock.unlock();

    search_thread = std::thread([filter_value, filter_type, this]() {
        search_promise.set_value(log_storage->perform_abortable_search(filter_value, filter_type, search_thread_running));
    });
}

void LoggerViewUI::kill_search_thread() {
    search_thread_running.store(false);
    if (search_thread.joinable()) {
        search_thread.join();
    }
}

bool LoggerViewUI::tooltip_callback(int x, int y, bool keyboard_tooltip, const Glib::RefPtr<Gtk::Tooltip>& tooltip) {
    // Gtk::TreeModel::iterator iter;
    // bool row_at_point = logs_treeview->get_tooltip_context_iter(x, y, keyboard_tooltip, iter);

    int cell_x, cell_y = 0;
    Gtk::TreeModel::Path path;
    Gtk::TreeViewColumn* column;
    bool path_exists;

    //Get the current path and column at the selected point
    if (keyboard_tooltip) {
        logs_treeview->get_cursor(path, column);
        path_exists = column != nullptr;
    }
    else {
        int window_x, window_y;
        logs_treeview->convert_widget_to_bin_window_coords(x, y, window_x, window_y);
        path_exists = logs_treeview->get_path_at_pos(window_x, window_y, path, column, cell_x, cell_y);
    }

    if (path_exists) {
        //Get selected row
        Gtk::TreeModel::iterator iter = log_list_store->get_iter(path);
        Gtk::TreeModel::Row row = *iter;

        //Get tooltip text depending on current column
        Glib::ustring content_ustring;
        if (column->get_title() == "Content") {
            content_ustring = Glib::ustring(row[log_record.log_content]);
        } 
        else if (column->get_title() == "ID") {
            content_ustring = Glib::ustring(row[log_record.log_id]);
        }
        else {
            uint64_t id(row[log_record.log_stamp]);
            std::stringstream stream;
            stream << id;
            content_ustring = Glib::ustring(stream.str().c_str());
        }

        //Get text at iter
        tooltip->set_text(content_ustring);
        return true;
    }
    else {
        return false;
    }
}

//Suppress warning for unused parameter (scroll_event)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

bool LoggerViewUI::scroll_callback(GdkEventScroll* scroll_event) {
    //React to a mouse scroll event (but propagate it further)
    autoscroll_check_button->set_active(false);
    return false;
}

#pragma GCC diagnostic pop

Gtk::Widget* LoggerViewUI::get_parent() {
    return parent;
}

void LoggerViewUI::reset()
{
    reset_logs.store(true);
}