#include "TimerViewUI.hpp"

using namespace std::placeholders;
TimerViewUI::TimerViewUI(bool simulated_time) :
    use_simulated_time(simulated_time),
    /*Set up communication*/
    ready_status_reader(std::bind(&TimerViewUI::ready_status_callback, this, _1), cpm::ParticipantSingleton::Instance(), cpm::get_topic<ReadyStatus>("ready"), true),
    system_trigger_writer(dds::pub::Publisher(cpm::ParticipantSingleton::Instance()), cpm::get_topic<SystemTrigger>("system_trigger"))
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
    active_timers_treeview->append_column("Is waiting for start", timer_record.column_waiting_for_start);
    active_timers_treeview->append_column("Waiting for its response", timer_record.column_waiting_for_response);
    active_timers_treeview->append_column("Next timestep", timer_record.column_next_step);

    //Set equal width for all columns
    for (int i = 0; i < 4; ++i) {
        active_timers_treeview->get_column(i)->set_resizable(true);
        active_timers_treeview->get_column(i)->set_min_width(20);
        active_timers_treeview->get_column(i)->set_fixed_width(50);
        active_timers_treeview->get_column(i)->set_expand(true);
    }
}

void TimerViewUI::insert_or_change_treeview(std::string id_string, std::string waiting_start_string, std::string waiting_response_string, std::string next_step_string) {
        Glib::ustring id_ustring(id_string);
        Glib::ustring waiting_start_ustring(waiting_start_string);
        Glib::ustring waiting_response_ustring(waiting_response_string);
        Glib::ustring next_step_ustring(next_step_string);

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
        row[timer_record.column_waiting_for_start] = waiting_start_ustring;
        row[timer_record.column_waiting_for_response] = waiting_response_ustring;
        row[timer_record.column_next_step] = next_step_ustring;
}

void TimerViewUI::ready_status_callback(dds::sub::LoanedSamples<ReadyStatus>& samples) {
    for (auto sample : samples) {
        if (sample.info().valid()) {
            std::string id = sample.data().source_id();
            std::string waiting_start = "";
            std::string waiting_response = "";
            std::stringstream step_stream;
            step_stream << sample.data().next_start_stamp().nanoseconds();
            std::string next_step = step_stream.str();

            std::unique_lock<std::mutex> lock(ready_status_storage_mutex);
            ready_status_storage[sample.data().source_id()] = sample.data().next_start_stamp().nanoseconds();
            
            insert_or_change_treeview(id, waiting_start, waiting_response, next_step);
            lock.unlock();
        }
    }
}

Gtk::Widget* TimerViewUI::get_parent() {
    return parent;
}