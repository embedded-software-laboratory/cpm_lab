#include "TimerViewUI.hpp"

using namespace std::placeholders;
TimerViewUI::TimerViewUI(std::shared_ptr<TimerTrigger> timerTrigger) :
    timer_trigger(timerTrigger),
    ui_dispatcher()
 {
    ui_builder = Gtk::Builder::create_from_file("ui/timer/timer.glade");

    ui_builder->get_widget("parent", parent);
    ui_builder->get_widget("button_start", button_start);
    ui_builder->get_widget("button_stop", button_stop);
    ui_builder->get_widget("active_timers_treeview", active_timers_treeview);
    ui_builder->get_widget("current_timestep_label", current_timestep_label);

    assert(parent);
    assert(button_start);
    assert(button_stop);
    assert(active_timers_treeview);
    assert(current_timestep_label);

    //Create model for view
    timer_list_storage = Gtk::ListStore::create(timer_record);
    active_timers_treeview->set_model(timer_list_storage);

    //Use model_record, add it to the view
    active_timers_treeview->append_column("ID", timer_record.column_id);
    active_timers_treeview->append_column("Last message", timer_record.column_last_message);
    active_timers_treeview->append_column("Waiting for response", timer_record.column_waiting_for_response);
    active_timers_treeview->append_column("Next timestep", timer_record.column_next_step);

    //Set equal width for all columns
    for (int i = 0; i < 4; ++i) {
        active_timers_treeview->get_column(i)->set_resizable(true);
        active_timers_treeview->get_column(i)->set_min_width(20);
        active_timers_treeview->get_column(i)->set_fixed_width(50);
        active_timers_treeview->get_column(i)->set_expand(true);
    }

    //Register callbacks for button presses
    button_start->signal_clicked().connect(sigc::mem_fun(this, &TimerViewUI::button_start_callback));
    button_stop->signal_clicked().connect(sigc::mem_fun(this, &TimerViewUI::button_stop_callback));

    //Create thread and register dispatcher callback
    ui_dispatcher.connect(sigc::mem_fun(*this, &TimerViewUI::dispatcher_callback));
    run_thread.store(true);
    ui_thread = std::thread(&TimerViewUI::update_ui, this);
}

TimerViewUI::~TimerViewUI() {
    run_thread.store(false);

    if(ui_thread.joinable()) {
        ui_thread.join();
    }
}

void TimerViewUI::dispatcher_callback() {
    //Update treeview
    for(const auto& entry : timer_trigger->get_participant_message_data()) {
        std::stringstream step_stream;
        step_stream << entry.second.next_timestep;

        Glib::ustring id_ustring(entry.first);
        Glib::ustring last_message_ustring(timer_trigger->get_human_readable_time_diff(entry.second.last_message_receive_stamp));
        Glib::ustring waiting_response_ustring(entry.second.waiting_for_response);
        Glib::ustring next_step_ustring(step_stream.str());

        Gtk::TreeModel::Row row;
        bool entry_exists = false;

        //Search if the entry already exists
        for (Gtk::TreeModel::iterator iter = timer_list_storage->children().begin(); iter != timer_list_storage->children().end(); ++iter) {
            row = *iter;
            if (row[timer_record.column_id] == id_ustring) {
                entry_exists = true;
                break;
            }
        }
        //Else create a new entry
        if (!entry_exists) {
            row = *(timer_list_storage->append());
        }

        row[timer_record.column_id] = id_ustring;
        row[timer_record.column_last_message] = last_message_ustring;
        row[timer_record.column_waiting_for_response] = waiting_response_ustring;
        row[timer_record.column_next_step] = next_step_ustring;
    }

    //Update current time in UI
    bool use_simulated_time;
    uint64_t current_simulated_time;
    timer_trigger->get_current_simulated_time(use_simulated_time, current_simulated_time);
    if (use_simulated_time) {
        std::stringstream time_stream;
        time_stream << current_simulated_time;
        Glib::ustring step_ustring(time_stream.str());
        current_timestep_label->set_label(step_ustring);
    }
}

void TimerViewUI::update_ui() {
    while (run_thread.load()) {
        ui_dispatcher.emit();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void TimerViewUI::button_start_callback() {
    timer_trigger->send_start_signal();

    button_start->set_sensitive(false);
}

void TimerViewUI::button_stop_callback() {
    timer_trigger->send_stop_signal();

    std::string label_msg = "stopped";
    Glib::ustring label_msg_ustring(label_msg);
    current_timestep_label->set_label(label_msg_ustring);

    button_start->set_sensitive(false);
}

Gtk::Widget* TimerViewUI::get_parent() {
    return parent;
}